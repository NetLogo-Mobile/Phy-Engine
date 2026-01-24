#pragma once

#include <cstddef>
#include <cstdint>
#include <chrono>
#include <filesystem>
#include <algorithm>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <leveldb/db.h>
#include <leveldb/options.h>
#include <leveldb/status.h>
#include <leveldb/write_batch.h>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/model_refs/variant.h>

#include "builtin_registry.h"
#include "codec.h"
#include "model_registry.h"
#include "archive.h"
#include "status.h"

namespace phy_engine::pe_nl_fileformat
{
    enum class export_mode : std::uint8_t
    {
        full = 0,
        structure_only = 1,
        runtime_only = 2
    };

    enum class storage_layout : std::uint8_t
    {
        single_file = 0,
        directory = 1,
        auto_detect = 2
    };

    enum class checkpoint_match_mode : std::uint8_t
    {
        stable_id = 0,
        sequence = 1
    };

    struct save_options
    {
        bool overwrite{};
        export_mode mode{export_mode::full};
        storage_layout layout{storage_layout::single_file};
    };

    struct load_options
    {
        bool require_model_state{true};
        storage_layout layout{storage_layout::auto_detect};
        checkpoint_match_mode checkpoint_mode{checkpoint_match_mode::stable_id};
        bool checkpoint_allow_fallback_to_sequence{true};
    };

    namespace details
    {
        inline std::string encode_attributes(::phy_engine::model::model_base const& m);

        inline std::uint64_t fnv1a64_update(std::uint64_t h, void const* data, std::size_t n) noexcept
        {
            return ::phy_engine::pe_nl_fileformat::details::fnv1a_update(h, data, n);
        }

        inline std::uint64_t fnv1a64_update(std::uint64_t h, std::string_view s) noexcept
        {
            return fnv1a64_update(h, s.data(), s.size());
        }

        struct stable_graph_ids
        {
            // In creation order:
            // - nodes: nl.nodes (does NOT include ground)
            // - models: nl.models filtered to (type==normal && ptr!=nullptr)
            ::fast_io::vector<std::uint64_t> node_uid{};
            ::fast_io::vector<std::uint64_t> model_uid{};
            std::uint64_t ground_uid{};
            std::uint64_t structure_hash{};
        };

        inline std::uint64_t fnv_mix_u64(std::uint64_t h, std::uint64_t v) noexcept
        {
            return fnv1a64_update(h, &v, sizeof(v));
        }

        inline std::uint64_t fnv_mix_u8(std::uint64_t h, std::uint8_t v) noexcept { return fnv1a64_update(h, &v, sizeof(v)); }

        inline std::uint64_t fnv_mix_str(std::uint64_t h, std::string_view s) noexcept
        {
            std::uint64_t const n = static_cast<std::uint64_t>(s.size());
            h = fnv_mix_u64(h, n);
            h = fnv1a64_update(h, s);
            return h;
        }

        inline stable_graph_ids compute_stable_ids(::phy_engine::circult const& c)
        {
            constexpr std::uint64_t kNodeNull = std::numeric_limits<std::uint64_t>::max();
            constexpr std::uint64_t kNodeGround = std::numeric_limits<std::uint64_t>::max() - 1;

            stable_graph_ids out{};

            // 1) enumerate nodes/models
            ::fast_io::vector<::phy_engine::model::node_t const*> nodes{};
            for(auto const& blk: c.nl.nodes)
                for(auto p = blk.begin; p != blk.curr; ++p) { nodes.push_back(p); }
            std::uint64_t const node_count = nodes.size();

            ::fast_io::vector<::phy_engine::model::model_base const*> models{};
            for(auto const& blk: c.nl.models)
            {
                for(auto p = blk.begin; p != blk.curr; ++p)
                {
                    if(p->type != ::phy_engine::model::model_type::normal || p->ptr == nullptr) { continue; }
                    models.push_back(p);
                }
            }
            std::uint64_t const model_count = models.size();

            std::unordered_map<::phy_engine::model::node_t const*, std::uint64_t> node_to_id;
            node_to_id.reserve(static_cast<std::size_t>(node_count * 2));
            for(std::uint64_t i{}; i < node_count; ++i) { node_to_id.emplace(nodes.index_unchecked(static_cast<std::size_t>(i)), i); }

            std::uint64_t const ground_id = node_count;  // special slot in refinement arrays

            struct edge
            {
                std::uint64_t model{};
                std::uint64_t pin{};
                std::string_view pin_name{};
            };

            ::fast_io::vector<::fast_io::vector<edge>> node_inc{};
            node_inc.resize(static_cast<std::size_t>(node_count + 1));  // + ground slot

            ::fast_io::vector<::fast_io::vector<std::uint64_t>> model_pin_node{};
            model_pin_node.resize(static_cast<std::size_t>(model_count));

            ::fast_io::vector<std::string> model_name_bytes{};
            ::fast_io::vector<std::string> model_attr_bytes{};
            model_name_bytes.resize(static_cast<std::size_t>(model_count));
            model_attr_bytes.resize(static_cast<std::size_t>(model_count));

            ::fast_io::vector<std::uint64_t> model_base{};
            model_base.resize(static_cast<std::size_t>(model_count));
            ::fast_io::vector<std::uint64_t> node_base{};
            node_base.resize(static_cast<std::size_t>(node_count + 1));

            // 2) base labels + adjacency
            for(std::uint64_t mi{}; mi < model_count; ++mi)
            {
                auto const* mb = models.index_unchecked(static_cast<std::size_t>(mi));
                auto const mname = mb->ptr->get_model_name();
                model_name_bytes.index_unchecked(static_cast<std::size_t>(mi)) = u8sv_to_bytes(mname);
                model_attr_bytes.index_unchecked(static_cast<std::size_t>(mi)) = encode_attributes(*mb);

                auto pv = mb->ptr->generate_pin_view();
                model_pin_node.index_unchecked(static_cast<std::size_t>(mi)).resize(pv.size);

                // model base label = H("M" + model_name + attrs + pin_names)
                std::uint64_t h0 = ::phy_engine::pe_nl_fileformat::details::fnv1a_basis;
                h0 = fnv_mix_u8(h0, static_cast<std::uint8_t>('M'));
                h0 = fnv_mix_str(h0, model_name_bytes.index_unchecked(static_cast<std::size_t>(mi)));
                h0 = fnv_mix_u64(h0, static_cast<std::uint64_t>(pv.size));
                h0 = fnv1a64_update(h0, model_attr_bytes.index_unchecked(static_cast<std::size_t>(mi)));
                for(std::size_t pi{}; pi < pv.size; ++pi)
                {
                    auto const pn = pv.pins[pi].name;
                    auto const pn_bytes = u8sv_to_bytes(pn);
                    h0 = fnv_mix_str(h0, pn_bytes);

                    std::uint64_t nid = kNodeNull;
                    auto const* n = pv.pins[pi].nodes;
                    if(n == nullptr) { nid = kNodeNull; }
                    else if(n == __builtin_addressof(c.nl.ground_node)) { nid = kNodeGround; }
                    else
                    {
                        auto it = node_to_id.find(n);
                        nid = (it == node_to_id.end()) ? kNodeNull : it->second;
                    }
                    model_pin_node.index_unchecked(static_cast<std::size_t>(mi)).index_unchecked(pi) =
                        (nid == kNodeGround) ? ground_id : nid;

                    if(nid != kNodeNull)
                    {
                        auto const to_id = (nid == kNodeGround) ? ground_id : nid;
                        node_inc.index_unchecked(static_cast<std::size_t>(to_id)).push_back(edge{mi, static_cast<std::uint64_t>(pi), pn_bytes});
                    }
                }
                model_base.index_unchecked(static_cast<std::size_t>(mi)) = h0;
            }

            // node base labels
            for(std::uint64_t ni{}; ni < node_count; ++ni)
            {
                auto const* n = nodes.index_unchecked(static_cast<std::size_t>(ni));
                std::uint64_t deg = static_cast<std::uint64_t>(node_inc.index_unchecked(static_cast<std::size_t>(ni)).size());
                std::uint64_t analog_deg{};
                for(auto const& e: node_inc.index_unchecked(static_cast<std::size_t>(ni)))
                {
                    auto const* mb = models.index_unchecked(static_cast<std::size_t>(e.model));
                    if(mb->ptr->get_device_type() != ::phy_engine::model::model_device_type::digital) { ++analog_deg; }
                }
                std::uint64_t h0 = ::phy_engine::pe_nl_fileformat::details::fnv1a_basis;
                h0 = fnv_mix_u8(h0, static_cast<std::uint8_t>('N'));
                h0 = fnv_mix_u64(h0, deg);
                h0 = fnv_mix_u64(h0, analog_deg);
                // If pins are empty (floating), keep a distinct base.
                h0 = fnv_mix_u64(h0, n->pins.size());
                node_base.index_unchecked(static_cast<std::size_t>(ni)) = h0;
            }
            {
                std::uint64_t h0 = ::phy_engine::pe_nl_fileformat::details::fnv1a_basis;
                h0 = fnv_mix_u8(h0, static_cast<std::uint8_t>('G'));
                node_base.index_unchecked(static_cast<std::size_t>(ground_id)) = h0;
                out.ground_uid = h0;
            }

            // 3) WL-style refinement over bipartite graph
            ::fast_io::vector<std::uint64_t> node_lbl{};
            ::fast_io::vector<std::uint64_t> model_lbl{};
            node_lbl.resize(static_cast<std::size_t>(node_count + 1));
            model_lbl.resize(static_cast<std::size_t>(model_count));
            for(std::uint64_t i{}; i < node_count + 1; ++i) { node_lbl.index_unchecked(static_cast<std::size_t>(i)) = node_base.index_unchecked(static_cast<std::size_t>(i)); }
            for(std::uint64_t i{}; i < model_count; ++i) { model_lbl.index_unchecked(static_cast<std::size_t>(i)) = model_base.index_unchecked(static_cast<std::size_t>(i)); }

            constexpr int kRounds = 8;
            ::fast_io::vector<std::uint64_t> new_node{};
            ::fast_io::vector<std::uint64_t> new_model{};
            new_node.resize(static_cast<std::size_t>(node_count + 1));
            new_model.resize(static_cast<std::size_t>(model_count));

            ::fast_io::vector<std::uint64_t> tmp_edges{};

            for(int round = 0; round < kRounds; ++round)
            {
                bool changed{};

                // models (ordered pins)
                for(std::uint64_t mi{}; mi < model_count; ++mi)
                {
                    std::uint64_t h = ::phy_engine::pe_nl_fileformat::details::fnv1a_basis;
                    h = fnv_mix_u8(h, static_cast<std::uint8_t>('m'));
                    h = fnv_mix_u64(h, model_base.index_unchecked(static_cast<std::size_t>(mi)));
                    auto const& pin_nodes = model_pin_node.index_unchecked(static_cast<std::size_t>(mi));
                    h = fnv_mix_u64(h, static_cast<std::uint64_t>(pin_nodes.size()));
                    for(std::size_t pi{}; pi < pin_nodes.size(); ++pi)
                    {
                        auto const nid = pin_nodes.index_unchecked(pi);
                        std::uint64_t const nl = (nid == kNodeNull) ? 0ull : node_lbl.index_unchecked(static_cast<std::size_t>(nid));
                        h = fnv_mix_u64(h, static_cast<std::uint64_t>(pi));
                        h = fnv_mix_u64(h, nl);
                    }
                    new_model.index_unchecked(static_cast<std::size_t>(mi)) = h;
                    if(h != model_lbl.index_unchecked(static_cast<std::size_t>(mi))) { changed = true; }
                }

                // nodes (sorted multiset)
                for(std::uint64_t ni{}; ni < node_count + 1; ++ni)
                {
                    auto const& inc = node_inc.index_unchecked(static_cast<std::size_t>(ni));
                    tmp_edges.clear();
                    tmp_edges.reserve(inc.size());
                    for(auto const& e: inc)
                    {
                        std::uint64_t eh = ::phy_engine::pe_nl_fileformat::details::fnv1a_basis;
                        eh = fnv_mix_u8(eh, static_cast<std::uint8_t>('e'));
                        eh = fnv_mix_u64(eh, e.pin);
                        eh = fnv_mix_str(eh, e.pin_name);
                        eh = fnv_mix_u64(eh, new_model.index_unchecked(static_cast<std::size_t>(e.model)));
                        tmp_edges.push_back(eh);
                    }
                    std::sort(tmp_edges.begin(), tmp_edges.end());
                    std::uint64_t h = ::phy_engine::pe_nl_fileformat::details::fnv1a_basis;
                    h = fnv_mix_u8(h, static_cast<std::uint8_t>('n'));
                    h = fnv_mix_u64(h, node_base.index_unchecked(static_cast<std::size_t>(ni)));
                    h = fnv_mix_u64(h, static_cast<std::uint64_t>(tmp_edges.size()));
                    for(auto const v: tmp_edges) { h = fnv_mix_u64(h, v); }
                    new_node.index_unchecked(static_cast<std::size_t>(ni)) = h;
                    if(h != node_lbl.index_unchecked(static_cast<std::size_t>(ni))) { changed = true; }
                }

                for(std::uint64_t mi{}; mi < model_count; ++mi) { model_lbl.index_unchecked(static_cast<std::size_t>(mi)) = new_model.index_unchecked(static_cast<std::size_t>(mi)); }
                for(std::uint64_t ni{}; ni < node_count + 1; ++ni) { node_lbl.index_unchecked(static_cast<std::size_t>(ni)) = new_node.index_unchecked(static_cast<std::size_t>(ni)); }

                if(!changed) { break; }
            };

            out.node_uid.resize(static_cast<std::size_t>(node_count));
            out.model_uid.resize(static_cast<std::size_t>(model_count));
            for(std::uint64_t ni{}; ni < node_count; ++ni) { out.node_uid.index_unchecked(static_cast<std::size_t>(ni)) = node_lbl.index_unchecked(static_cast<std::size_t>(ni)); }
            for(std::uint64_t mi{}; mi < model_count; ++mi) { out.model_uid.index_unchecked(static_cast<std::size_t>(mi)) = model_lbl.index_unchecked(static_cast<std::size_t>(mi)); }
            out.ground_uid = node_lbl.index_unchecked(static_cast<std::size_t>(ground_id));

            // Order-independent structure hash: multiset of node/model uids.
            ::std::vector<std::uint64_t> n_sorted(out.node_uid.begin(), out.node_uid.end());
            ::std::vector<std::uint64_t> m_sorted(out.model_uid.begin(), out.model_uid.end());
            std::sort(n_sorted.begin(), n_sorted.end());
            std::sort(m_sorted.begin(), m_sorted.end());

            std::uint64_t hh = ::phy_engine::pe_nl_fileformat::details::fnv1a_basis;
            hh = fnv_mix_u8(hh, static_cast<std::uint8_t>('S'));
            hh = fnv_mix_u64(hh, node_count);
            hh = fnv_mix_u64(hh, model_count);
            hh = fnv_mix_u64(hh, out.ground_uid);
            for(auto const v: n_sorted) { hh = fnv_mix_u64(hh, v); }
            for(auto const v: m_sorted) { hh = fnv_mix_u64(hh, v); }
            out.structure_hash = hh;

            return out;
        }

        inline std::string key_cat(std::string_view a, std::string_view b)
        {
            std::string k;
            k.reserve(a.size() + b.size());
            k.append(a.data(), a.data() + a.size());
            k.append(b.data(), b.data() + b.size());
            return k;
        }

        inline std::string key_u64(std::string_view prefix, std::uint64_t id, std::string_view suffix)
        {
            std::string k;
            k.reserve(prefix.size() + 32 + suffix.size());
            k.append(prefix.data(), prefix.data() + prefix.size());
            k.append(std::to_string(id));
            k.append(suffix.data(), suffix.data() + suffix.size());
            return k;
        }

        inline status put(leveldb::WriteBatch& b, std::string const& k, std::string const& v)
        {
            b.Put(k, v);
            return {};
        }

        inline status get(leveldb::DB& db, leveldb::ReadOptions const& ro, std::string const& k, std::string& v);

        inline void put_u64(leveldb::WriteBatch& b, std::string const& k, std::uint64_t v)
        {
            std::string out{};
            out.resize(sizeof(v));
            std::memcpy(out.data(), &v, sizeof(v));
            b.Put(k, out);
        }

        inline status get_u64(leveldb::DB& db, leveldb::ReadOptions const& ro, std::string const& k, std::uint64_t& v)
        {
            std::string s{};
            auto st = get(db, ro, k, s);
            if(!st) { return st; }
            if(s.size() != sizeof(v)) { return {errc::corrupt, "u64 value has wrong size"}; }
            std::memcpy(&v, s.data(), sizeof(v));
            return {};
        }

        inline status get(leveldb::DB& db, leveldb::ReadOptions const& ro, std::string const& k, std::string& v)
        {
            auto const s = db.Get(ro, k, &v);
            if(s.ok()) { return {}; }
            if(s.IsNotFound()) { return {errc::not_found, "missing key: " + k}; }
            return {errc::db_error, s.ToString()};
        }

        inline void append_variant(std::string& out, ::phy_engine::model::variant const& v)
        {
            details::append_u8(out, static_cast<std::uint8_t>(v.type));
            switch(v.type)
            {
                case ::phy_engine::model::variant_type::i8: details::append_trivial(out, v.i8); break;
                case ::phy_engine::model::variant_type::i16: details::append_trivial(out, v.i16); break;
                case ::phy_engine::model::variant_type::i32: details::append_trivial(out, v.i32); break;
                case ::phy_engine::model::variant_type::i64: details::append_trivial(out, v.i64); break;
                case ::phy_engine::model::variant_type::ui8: details::append_trivial(out, v.ui8); break;
                case ::phy_engine::model::variant_type::ui16: details::append_trivial(out, v.ui16); break;
                case ::phy_engine::model::variant_type::ui32: details::append_trivial(out, v.ui32); break;
                case ::phy_engine::model::variant_type::ui64: details::append_trivial(out, v.ui64); break;
                case ::phy_engine::model::variant_type::boolean: details::append_u8(out, v.boolean ? 1u : 0u); break;
                case ::phy_engine::model::variant_type::f: details::append_trivial(out, v.f); break;
                case ::phy_engine::model::variant_type::d: details::append_f64(out, v.d); break;
                case ::phy_engine::model::variant_type::digital: details::append_u8(out, static_cast<std::uint8_t>(v.digital)); break;
                case ::phy_engine::model::variant_type::invalid: [[fallthrough]];
                default: break;
            }
        }

        inline status read_variant(std::string_view in, std::size_t& off, ::phy_engine::model::variant& v)
        {
            if(off >= in.size()) { return {errc::corrupt, "unexpected EOF reading variant type"}; }
            v.type = static_cast<::phy_engine::model::variant_type>(static_cast<std::uint8_t>(in[off++]));
            switch(v.type)
            {
                case ::phy_engine::model::variant_type::i8: return details::read_trivial(in, off, v.i8);
                case ::phy_engine::model::variant_type::i16: return details::read_trivial(in, off, v.i16);
                case ::phy_engine::model::variant_type::i32: return details::read_trivial(in, off, v.i32);
                case ::phy_engine::model::variant_type::i64: return details::read_trivial(in, off, v.i64);
                case ::phy_engine::model::variant_type::ui8: return details::read_trivial(in, off, v.ui8);
                case ::phy_engine::model::variant_type::ui16: return details::read_trivial(in, off, v.ui16);
                case ::phy_engine::model::variant_type::ui32: return details::read_trivial(in, off, v.ui32);
                case ::phy_engine::model::variant_type::ui64: return details::read_trivial(in, off, v.ui64);
                case ::phy_engine::model::variant_type::boolean:
                {
                    if(off >= in.size()) { return {errc::corrupt, "unexpected EOF reading bool"}; }
                    v.boolean = static_cast<std::uint8_t>(in[off++]) != 0;
                    return {};
                }
                case ::phy_engine::model::variant_type::f: return details::read_trivial(in, off, v.f);
                case ::phy_engine::model::variant_type::d: return details::read_f64(in, off, v.d);
                case ::phy_engine::model::variant_type::digital:
                {
                    if(off >= in.size()) { return {errc::corrupt, "unexpected EOF reading digital"}; }
                    v.digital = static_cast<::phy_engine::model::digital_node_statement_t>(static_cast<std::uint8_t>(in[off++]));
                    return {};
                }
                case ::phy_engine::model::variant_type::invalid: [[fallthrough]];
                default: return {};
            }
        }

        inline std::string encode_attributes(::phy_engine::model::model_base const& m)
        {
            std::string out{};
            constexpr std::size_t kMaxScan{512};
            constexpr std::size_t kMaxConsecutiveEmpty{64};
            std::size_t empty_run{};
            bool seen_any{};

            // 1st pass: count present entries
            std::size_t count{};
            for(std::size_t idx{}; idx < kMaxScan; ++idx)
            {
                auto const n = m.ptr->get_attribute_name(idx);
                if(n.empty())
                {
                    if(seen_any && ++empty_run >= kMaxConsecutiveEmpty) { break; }
                    continue;
                }
                seen_any = true;
                empty_run = 0;
                ++count;
            }

            details::append_uleb128(out, count);

            empty_run = 0;
            seen_any = false;
            for(std::size_t idx{}; idx < kMaxScan; ++idx)
            {
                auto const n = m.ptr->get_attribute_name(idx);
                if(n.empty())
                {
                    if(seen_any && ++empty_run >= kMaxConsecutiveEmpty) { break; }
                    continue;
                }
                seen_any = true;
                empty_run = 0;

                details::append_uleb128(out, idx);
                details::append_string(out, details::u8sv_to_bytes(n));
                append_variant(out, m.ptr->get_attribute(idx));
            }
            return out;
        }

        inline status apply_attributes(::phy_engine::model::model_base& m, std::string_view in)
        {
            std::size_t off{};
            std::uint64_t count{};
            auto st = details::read_uleb128(in, off, count);
            if(!st) { return st; }
            for(std::uint64_t i{}; i < count; ++i)
            {
                std::uint64_t idx{};
                st = details::read_uleb128(in, off, idx);
                if(!st) { return st; }
                std::string name{};
                st = details::read_string(in, off, name);
                if(!st) { return st; }
                ::phy_engine::model::variant v{};
                st = read_variant(in, off, v);
                if(!st) { return st; }
                (void)name;  // kept for debug/backward compatibility; index is authoritative
                (void)m.ptr->set_attribute(static_cast<std::size_t>(idx), v);
            }
            if(off != in.size()) { return {errc::corrupt, "trailing bytes in attributes blob"}; }
            return {};
        }

        inline std::string encode_environment(::phy_engine::environment const& e)
        {
            std::string out{};
            details::append_f64(out, e.V_eps_max);
            details::append_f64(out, e.V_epsr_max);
            details::append_f64(out, e.I_eps_max);
            details::append_f64(out, e.I_epsr_max);
            details::append_f64(out, e.charge_eps_max);
            details::append_f64(out, e.g_min);
            details::append_f64(out, e.r_open);
            details::append_f64(out, e.t_TOEF);
            details::append_f64(out, e.temperature);
            details::append_f64(out, e.norm_temperature);
            return out;
        }

        inline status decode_environment(std::string_view in, ::phy_engine::environment& e)
        {
            std::size_t off{};
            auto st = details::read_f64(in, off, e.V_eps_max);
            if(!st) { return st; }
            st = details::read_f64(in, off, e.V_epsr_max);
            if(!st) { return st; }
            st = details::read_f64(in, off, e.I_eps_max);
            if(!st) { return st; }
            st = details::read_f64(in, off, e.I_epsr_max);
            if(!st) { return st; }
            st = details::read_f64(in, off, e.charge_eps_max);
            if(!st) { return st; }
            st = details::read_f64(in, off, e.g_min);
            if(!st) { return st; }
            st = details::read_f64(in, off, e.r_open);
            if(!st) { return st; }
            st = details::read_f64(in, off, e.t_TOEF);
            if(!st) { return st; }
            st = details::read_f64(in, off, e.temperature);
            if(!st) { return st; }
            st = details::read_f64(in, off, e.norm_temperature);
            if(!st) { return st; }
            if(off != in.size()) { return {errc::corrupt, "trailing bytes in environment"}; }
            return {};
        }

        inline std::string encode_analyzer(::phy_engine::analyzer::analyzer_storage_t const& a)
        {
            std::string out{};
            details::append_u8(out, static_cast<std::uint8_t>(a.ac.sweep));
            details::append_f64(out, a.ac.omega);
            details::append_f64(out, a.ac.omega_start);
            details::append_f64(out, a.ac.omega_stop);
            details::append_uleb128(out, a.ac.points);
            details::append_f64(out, a.dc.m_currentOmega);
            details::append_f64(out, a.tr.t_stop);
            details::append_f64(out, a.tr.t_step);
            return out;
        }

        inline status decode_analyzer(std::string_view in, ::phy_engine::analyzer::analyzer_storage_t& a)
        {
            std::size_t off{};
            if(off >= in.size()) { return {errc::corrupt, "unexpected EOF reading analyzer sweep"}; }
            a.ac.sweep = static_cast<::phy_engine::analyzer::AC::sweep_type>(static_cast<std::uint8_t>(in[off++]));
            auto st = details::read_f64(in, off, a.ac.omega);
            if(!st) { return st; }
            st = details::read_f64(in, off, a.ac.omega_start);
            if(!st) { return st; }
            st = details::read_f64(in, off, a.ac.omega_stop);
            if(!st) { return st; }
            std::uint64_t pts{};
            st = details::read_uleb128(in, off, pts);
            if(!st) { return st; }
            a.ac.points = static_cast<std::size_t>(pts);
            st = details::read_f64(in, off, a.dc.m_currentOmega);
            if(!st) { return st; }
            st = details::read_f64(in, off, a.tr.t_stop);
            if(!st) { return st; }
            st = details::read_f64(in, off, a.tr.t_step);
            if(!st) { return st; }
            if(off != in.size()) { return {errc::corrupt, "trailing bytes in analyzer"}; }
            return {};
        }
    }  // namespace details

    inline status save_to_leveldb(std::filesystem::path const& db_path,
                                  ::phy_engine::circult const& c,
                                  save_options opt = {},
                                  model_registry const& reg = default_registry())
    {
        bool const want_structure = (opt.mode == export_mode::full || opt.mode == export_mode::structure_only);
        bool const want_runtime = (opt.mode == export_mode::full || opt.mode == export_mode::runtime_only);
        bool const want_node_state = want_runtime && (opt.mode != export_mode::structure_only);
        bool const want_model_state = want_runtime && (opt.mode != export_mode::structure_only);

        auto const stable_ids = details::compute_stable_ids(c);

        leveldb::Options options;
        options.create_if_missing = true;

        if(opt.overwrite)
        {
            auto const ds = leveldb::DestroyDB(db_path.string(), options);
            if(!ds.ok() && !ds.IsNotFound()) { return {errc::db_error, ds.ToString()}; }
        }

        leveldb::DB* db_raw{};
        auto const os = leveldb::DB::Open(options, db_path.string(), &db_raw);
        if(!os.ok()) { return {errc::db_error, os.ToString()}; }
        std::unique_ptr<leveldb::DB> db{db_raw};

        leveldb::WriteBatch batch;

        // --- meta ---
        {
            std::string v{};
            details::append_trivial(v, static_cast<std::uint32_t>(1));  // format_version
            batch.Put("meta/format_version", v);
        }
        {
            std::string v{};
            details::append_u8(v, static_cast<std::uint8_t>(opt.mode));
            batch.Put("meta/mode", v);
        }
        {
            details::put_u64(batch, "meta/structure_hash", stable_ids.structure_hash);
        }
        {
            std::string v{};
            details::append_trivial(v, static_cast<std::uint32_t>(1));  // uid algo version
            batch.Put("meta/uid_algo_version", v);
        }
        {
            std::string v{};
            details::append_u8(v, want_node_state ? 1u : 0u);
            details::append_u8(v, want_model_state ? 1u : 0u);
            details::append_u8(v, want_runtime ? 1u : 0u);
            details::append_u8(v, want_structure ? 1u : 0u);
            batch.Put("meta/flags", v);
        }

        // --- circuit ---
        batch.Put("circuit/env", details::encode_environment(c.env));
        {
            std::string v{};
            details::append_trivial(v, static_cast<std::uint32_t>(c.at));
            batch.Put("circuit/analyze_type", v);
        }
        batch.Put("circuit/analyzer", details::encode_analyzer(c.analyzer_setting));

        if(want_runtime)
        {
            std::string v{};
            details::append_u8(v, c.has_prepare ? 1u : 0u);
            details::append_f64(v, c.tr_duration);
            details::append_f64(v, c.last_step);
            details::append_u8(v, static_cast<std::uint8_t>(c.cuda_policy));
            details::append_uleb128(v, c.cuda_node_threshold);
            batch.Put("runtime/basic", v);
        }

        // --- nodes (count always; state optional) ---
        std::unordered_map<::phy_engine::model::node_t const*, std::uint64_t> node_to_id;
        node_to_id.reserve(1024);
        std::uint64_t node_count{};
        for(auto const& blk: c.nl.nodes)
        {
            for(auto p = blk.begin; p != blk.curr; ++p) { node_to_id.emplace(p, node_count++); }
        }

        {
            std::string v{};
            details::append_uleb128(v, node_count);
            batch.Put("nodes/count", v);
        }

        // stable uid tables are always written (used by checkpoint mapping)
        details::put_u64(batch, "nodes/ground_uid", stable_ids.ground_uid);
        for(std::uint64_t id{}; id < node_count && id < static_cast<std::uint64_t>(stable_ids.node_uid.size()); ++id)
        {
            details::put_u64(batch, details::key_u64("nodes/", id, "/uid"), stable_ids.node_uid.index_unchecked(static_cast<std::size_t>(id)));
        }

        if(want_node_state)
        {
            for(auto const& blk: c.nl.nodes)
            {
                for(auto p = blk.begin; p != blk.curr; ++p)
                {
                    auto const id = node_to_id.find(p)->second;
                    std::string v{};
                    bool const is_analog = p->num_of_analog_node != 0;
                    details::append_u8(v, is_analog ? 1u : 0u);
                    if(is_analog)
                    {
                        details::append_f64(v, p->node_information.an.voltage.real());
                        details::append_f64(v, p->node_information.an.voltage.imag());
                    }
                    else
                    {
                        details::append_u8(v, static_cast<std::uint8_t>(p->node_information.dn.state));
                    }
                    details::append_uleb128(v, p->num_of_analog_node);
                    batch.Put(details::key_u64("nodes/", id, "/state"), v);
                }
            }

            // ground node
            {
                std::string v{};
                details::append_f64(v, c.nl.ground_node.node_information.an.voltage.real());
                details::append_f64(v, c.nl.ground_node.node_information.an.voltage.imag());
                batch.Put("nodes/ground", v);
            }
        }

        // --- models ---
        constexpr std::uint64_t kNodeNull = std::numeric_limits<std::uint64_t>::max();
        constexpr std::uint64_t kNodeGround = std::numeric_limits<std::uint64_t>::max() - 1;

        std::uint64_t model_count{};
        for(auto const& blk: c.nl.models)
        {
            for(auto p = blk.begin; p != blk.curr; ++p)
            {
                if(p->type != ::phy_engine::model::model_type::normal || p->ptr == nullptr) { continue; }
                ++model_count;
            }
        }

        {
            std::string v{};
            details::append_uleb128(v, model_count);
            batch.Put("models/count", v);
        }

        std::uint64_t mid{};
        for(auto const& blk: c.nl.models)
        {
            for(auto p = blk.begin; p != blk.curr; ++p)
            {
                if(p->type != ::phy_engine::model::model_type::normal || p->ptr == nullptr) { continue; }

                auto const mname = p->ptr->get_model_name();

                if(want_structure)
                {
                    batch.Put(details::key_u64("m/", mid, "/model_name"), details::u8sv_to_bytes(mname));
                    batch.Put(details::key_u64("m/", mid, "/identification_name"), details::u8sv_to_bytes(p->ptr->get_identification_name()));
                    batch.Put(details::key_u64("m/", mid, "/attrs"), details::encode_attributes(*p));

                    {
                        std::string w{};
                        details::append_uleb128(w, p->identification);
                        details::append_u8(w, p->has_init ? 1u : 0u);
                        details::append_string(w, details::u8sv_to_bytes(::fast_io::u8string_view{p->name.data(), p->name.size()}));
                        details::append_string(w, details::u8sv_to_bytes(::fast_io::u8string_view{p->describe.data(), p->describe.size()}));
                        batch.Put(details::key_u64("m/", mid, "/wrapper"), w);
                    }

                    {
                        auto pv = p->ptr->generate_pin_view();
                        std::string pins{};
                        details::append_uleb128(pins, pv.size);
                        for(std::size_t i{}; i < pv.size; ++i)
                        {
                            auto const* n = pv.pins[i].nodes;
                            std::uint64_t idv = kNodeNull;
                            if(n == nullptr) { idv = kNodeNull; }
                            else if(n == __builtin_addressof(c.nl.ground_node)) { idv = kNodeGround; }
                            else
                            {
                                auto it = node_to_id.find(n);
                                if(it == node_to_id.end()) { return {errc::corrupt, "pin node not found in node table"}; }
                                idv = it->second;
                            }
                            details::append_trivial(pins, idv);
                        }
                        batch.Put(details::key_u64("m/", mid, "/pins"), pins);
                    }
                }

                if(want_model_state)
                {
                    auto const* codec = reg.find(mname);
                    if(codec == nullptr) { return {errc::unsupported, "no model codec registered for: " + details::u8sv_to_bytes(mname)}; }
                    std::string state{};
                    auto st = codec->save_state(*p, state);
                    if(!st) { return st; }
                    batch.Put(details::key_u64("m/", mid, "/state"), state);
                }

                if(mid < static_cast<std::uint64_t>(stable_ids.model_uid.size()))
                {
                    details::put_u64(batch, details::key_u64("m/", mid, "/uid"), stable_ids.model_uid.index_unchecked(static_cast<std::size_t>(mid)));
                }

                ++mid;
            }
        }

        auto const ws = db->Write(leveldb::WriteOptions{}, &batch);
        if(!ws.ok()) { return {errc::db_error, ws.ToString()}; }
        return {};
    }

    inline status load_from_leveldb(std::filesystem::path const& db_path,
                                    ::phy_engine::circult& c,
                                    load_options opt = {},
                                    model_registry const& reg = default_registry())
    {
        leveldb::Options options;
        options.create_if_missing = false;

        leveldb::DB* db_raw{};
        auto const os = leveldb::DB::Open(options, db_path.string(), &db_raw);
        if(!os.ok()) { return {errc::db_error, os.ToString()}; }
        std::unique_ptr<leveldb::DB> db{db_raw};
        leveldb::ReadOptions ro;

        // meta/version + mode
        export_mode mode{export_mode::full};
        bool has_node_state{true};
        bool has_model_state{true};
        bool has_runtime{true};
        bool has_structure{true};
        std::uint64_t expected_structure_hash{};
        {
            std::string v{};
            auto st = details::get(*db, ro, "meta/format_version", v);
            if(!st) { return st; }
            std::size_t off{};
            std::uint32_t ver{};
            st = details::read_trivial(std::string_view{v}, off, ver);
            if(!st) { return st; }
            if(ver != 1) { return {errc::unsupported, "unsupported pe_nl format version"}; }
        }
        {
            std::string v{};
            auto st = details::get(*db, ro, "meta/mode", v);
            if(st)
            {
                if(v.empty()) { return {errc::corrupt, "meta/mode empty"}; }
                mode = static_cast<export_mode>(static_cast<std::uint8_t>(v[0]));
            }
            else if(st.code != errc::not_found)
            {
                return st;
            }
        }
        {
            auto st = details::get_u64(*db, ro, "meta/structure_hash", expected_structure_hash);
            if(!st && st.code != errc::not_found) { return st; }
        }
        {
            std::string v{};
            auto st = details::get(*db, ro, "meta/flags", v);
            if(st)
            {
                if(v.size() < 4) { return {errc::corrupt, "meta/flags too short"}; }
                has_node_state = static_cast<std::uint8_t>(v[0]) != 0;
                has_model_state = static_cast<std::uint8_t>(v[1]) != 0;
                has_runtime = static_cast<std::uint8_t>(v[2]) != 0;
                has_structure = static_cast<std::uint8_t>(v[3]) != 0;
            }
            else if(st.code != errc::not_found)
            {
                return st;
            }
        }

        // load counts first
        std::uint64_t node_count{};
        {
            std::string v{};
            auto st = details::get(*db, ro, "nodes/count", v);
            if(!st) { return st; }
            std::size_t off{};
            st = details::read_uleb128(v, off, node_count);
            if(!st) { return st; }
        }
        std::uint64_t model_count{};
        {
            std::string v{};
            auto st = details::get(*db, ro, "models/count", v);
            if(!st) { return st; }
            std::size_t off{};
            st = details::read_uleb128(v, off, model_count);
            if(!st) { return st; }
        }

        // For checkpoint apply (runtime_only), precompute target mapping before any state is applied.
        ::fast_io::vector<::phy_engine::model::node_t*> cur_nodes{};
        ::fast_io::vector<::phy_engine::model::model_base*> cur_models{};
        ::fast_io::vector<std::uint64_t> node_map{};
        ::fast_io::vector<std::uint64_t> model_map{};
        bool checkpoint_use_sequence{};

        if(mode == export_mode::runtime_only)
        {
            // Apply runtime-only checkpoint onto an existing circuit.
            for(auto const& blk: c.nl.nodes)
                for(auto p = blk.begin; p != blk.curr; ++p) { cur_nodes.push_back(const_cast<::phy_engine::model::node_t*>(p)); }
            for(auto const& blk: c.nl.models)
                for(auto p = blk.begin; p != blk.curr; ++p)
                    if(p->type == ::phy_engine::model::model_type::normal && p->ptr != nullptr) { cur_models.push_back(const_cast<::phy_engine::model::model_base*>(p)); }

            if(static_cast<std::uint64_t>(cur_nodes.size()) != node_count || static_cast<std::uint64_t>(cur_models.size()) != model_count)
            {
                return {errc::unsupported, "checkpoint counts mismatch"};
            }

            // If we can, use stable-id mapping (default) and fallback to sequence if needed.
            checkpoint_use_sequence = checkpoint_use_sequence || (opt.checkpoint_mode == checkpoint_match_mode::sequence);

            auto const cur_ids = details::compute_stable_ids(c);
            if(!checkpoint_use_sequence && expected_structure_hash != 0 && cur_ids.structure_hash != expected_structure_hash)
            {
                checkpoint_use_sequence = opt.checkpoint_allow_fallback_to_sequence;
                if(!checkpoint_use_sequence) { return {errc::unsupported, "checkpoint structure_hash mismatch"}; }
            }

            node_map.resize(static_cast<std::size_t>(node_count));
            model_map.resize(static_cast<std::size_t>(model_count));
            for(std::uint64_t i{}; i < node_count; ++i) { node_map.index_unchecked(static_cast<std::size_t>(i)) = i; }
            for(std::uint64_t i{}; i < model_count; ++i) { model_map.index_unchecked(static_cast<std::size_t>(i)) = i; }

            if(!checkpoint_use_sequence)
            {
                // Read checkpoint uids.
                ::fast_io::vector<std::uint64_t> ck_node_uid{};
                ::fast_io::vector<std::uint64_t> ck_model_uid{};
                ck_node_uid.resize(static_cast<std::size_t>(node_count));
                ck_model_uid.resize(static_cast<std::size_t>(model_count));

                bool uid_ok{true};
                for(std::uint64_t i{}; i < node_count; ++i)
                {
                    std::uint64_t v{};
                    auto st = details::get_u64(*db, ro, details::key_u64("nodes/", i, "/uid"), v);
                    if(!st) { uid_ok = false; break; }
                    ck_node_uid.index_unchecked(static_cast<std::size_t>(i)) = v;
                }
                if(uid_ok)
                {
                    for(std::uint64_t i{}; i < model_count; ++i)
                    {
                        std::uint64_t v{};
                        auto st = details::get_u64(*db, ro, details::key_u64("m/", i, "/uid"), v);
                        if(!st) { uid_ok = false; break; }
                        ck_model_uid.index_unchecked(static_cast<std::size_t>(i)) = v;
                    }
                }

                auto build_unique_map = [](auto const& from_uid, auto const& to_uid, auto& out_map) noexcept -> bool
                {
                    if(from_uid.size() != to_uid.size()) { return false; }
                    std::unordered_map<std::uint64_t, std::uint64_t> to_index{};
                    to_index.reserve(to_uid.size() * 2);
                    for(std::uint64_t i{}; i < static_cast<std::uint64_t>(to_uid.size()); ++i)
                    {
                        auto const uid = to_uid.index_unchecked(static_cast<std::size_t>(i));
                        if(to_index.find(uid) != to_index.end()) { return false; }
                        to_index.emplace(uid, i);
                    }
                    std::unordered_map<std::uint64_t, bool> from_seen{};
                    from_seen.reserve(from_uid.size() * 2);
                    for(std::uint64_t i{}; i < static_cast<std::uint64_t>(from_uid.size()); ++i)
                    {
                        auto const uid = from_uid.index_unchecked(static_cast<std::size_t>(i));
                        if(from_seen.find(uid) != from_seen.end()) { return false; }
                        from_seen.emplace(uid, true);
                        auto it = to_index.find(uid);
                        if(it == to_index.end()) { return false; }
                        out_map.index_unchecked(static_cast<std::size_t>(i)) = it->second;
                    }
                    return true;
                };

                if(uid_ok)
                {
                    bool ok_nodes = build_unique_map(ck_node_uid, cur_ids.node_uid, node_map);
                    bool ok_models = build_unique_map(ck_model_uid, cur_ids.model_uid, model_map);
                    if(!ok_nodes || !ok_models)
                    {
                        checkpoint_use_sequence = opt.checkpoint_allow_fallback_to_sequence;
                        if(!checkpoint_use_sequence) { return {errc::unsupported, "checkpoint stable-id mapping failed (duplicates or mismatch)"}; }
                        for(std::uint64_t i{}; i < node_count; ++i) { node_map.index_unchecked(static_cast<std::size_t>(i)) = i; }
                        for(std::uint64_t i{}; i < model_count; ++i) { model_map.index_unchecked(static_cast<std::size_t>(i)) = i; }
                    }
                }
                else
                {
                    checkpoint_use_sequence = opt.checkpoint_allow_fallback_to_sequence;
                    if(!checkpoint_use_sequence) { return {errc::unsupported, "checkpoint missing uid tables"}; }
                }
            }
        }
        else
        {
            // Full/structure load: Clear existing circuit first.
            c.reset();
            c.nl.models.clear();
            c.nl.nodes.clear();
            c.nl.ground_node.clear();
        }

        // env + settings
        {
            std::string v{};
            auto st = details::get(*db, ro, "circuit/env", v);
            if(!st) { return st; }
            st = details::decode_environment(v, c.env);
            if(!st) { return st; }
        }
        {
            std::string v{};
            auto st = details::get(*db, ro, "circuit/analyze_type", v);
            if(!st) { return st; }
            std::size_t off{};
            std::uint32_t at{};
            st = details::read_trivial(v, off, at);
            if(!st) { return st; }
            c.at = static_cast<::phy_engine::analyze_type>(at);
        }
        {
            std::string v{};
            auto st = details::get(*db, ro, "circuit/analyzer", v);
            if(!st) { return st; }
            st = details::decode_analyzer(v, c.analyzer_setting);
            if(!st) { return st; }
        }

        // runtime/basic (optional)
        if(has_runtime)
        {
            std::string v{};
            auto st = details::get(*db, ro, "runtime/basic", v);
            if(st)
            {
                std::size_t off{};
                if(off >= v.size()) { return {errc::corrupt, "runtime/basic too short"}; }
                c.has_prepare = static_cast<std::uint8_t>(v[off++]) != 0;
                st = details::read_f64(v, off, c.tr_duration);
                if(!st) { return st; }
                st = details::read_f64(v, off, c.last_step);
                if(!st) { return st; }
                if(off >= v.size()) { return {errc::corrupt, "runtime/basic missing cuda_policy"}; }
                c.cuda_policy = static_cast<::phy_engine::circult::cuda_solve_policy>(static_cast<std::uint8_t>(v[off++]));
                std::uint64_t thr{};
                st = details::read_uleb128(v, off, thr);
                if(!st) { return st; }
                c.cuda_node_threshold = static_cast<std::size_t>(thr);
            }
            else if(st.code != errc::not_found)
            {
                return st;
            }
        }

        ::fast_io::vector<::phy_engine::model::node_t*> id_to_node{};
        id_to_node.resize(static_cast<std::size_t>(node_count));
        ::fast_io::vector<::std::complex<double>> saved_node_voltage{};
        ::fast_io::vector<::phy_engine::model::digital_node_statement_t> saved_node_digital{};
        saved_node_voltage.resize(static_cast<std::size_t>(node_count));
        saved_node_digital.resize(static_cast<std::size_t>(node_count));

        for(std::uint64_t id{}; id < node_count; ++id)
        {
            // For runtime_only, reuse existing nodes; for full/structure, create new nodes.
            ::phy_engine::model::node_t* np{};
            if(mode == export_mode::runtime_only)
            {
                if(static_cast<std::size_t>(id) >= node_map.size()) { return {errc::corrupt, "checkpoint: node map out of range"}; }
                auto const tgt = node_map.index_unchecked(static_cast<std::size_t>(id));
                if(static_cast<std::size_t>(tgt) >= cur_nodes.size()) { return {errc::corrupt, "checkpoint: node target out of range"}; }
                np = cur_nodes.index_unchecked(static_cast<std::size_t>(tgt));
            }
            else
            {
                auto& n = ::phy_engine::netlist::create_node(c.nl);
                np = __builtin_addressof(n);
            }

            id_to_node.index_unchecked(static_cast<std::size_t>(id)) = np;

            if(!has_node_state) { continue; }

            std::string v{};
            auto st = details::get(*db, ro, details::key_u64("nodes/", id, "/state"), v);
            if(!st) { return st; }

            std::size_t off{};
            if(off >= v.size()) { return {errc::corrupt, "node state too short"}; }
            bool const is_analog = static_cast<std::uint8_t>(v[off++]) != 0;
            if(is_analog)
            {
                double re{};
                double im{};
                st = details::read_f64(v, off, re);
                if(!st) { return st; }
                st = details::read_f64(v, off, im);
                if(!st) { return st; }
                saved_node_voltage.index_unchecked(static_cast<std::size_t>(id)) = {re, im};
            }
            else
            {
                if(off >= v.size()) { return {errc::corrupt, "node state missing digital"}; }
                saved_node_digital.index_unchecked(static_cast<std::size_t>(id)) =
                    static_cast<::phy_engine::model::digital_node_statement_t>(static_cast<std::uint8_t>(v[off++]));
            }
            std::uint64_t nao{};
            st = details::read_uleb128(v, off, nao);
            if(!st) { return st; }
            (void)nao;
            if(off != v.size()) { return {errc::corrupt, "trailing bytes in node state"}; }
        }

        // ground (optional)
        if(has_node_state)
        {
            std::string v{};
            auto st = details::get(*db, ro, "nodes/ground", v);
            if(st)
            {
                std::size_t off{};
                double re{};
                double im{};
                st = details::read_f64(v, off, re);
                if(!st) { return st; }
                st = details::read_f64(v, off, im);
                if(!st) { return st; }
                c.nl.ground_node.node_information.an.voltage = {re, im};
            }
            else if(st.code != errc::not_found)
            {
                return st;
            }
        }

        constexpr std::uint64_t kNodeNull = std::numeric_limits<std::uint64_t>::max();
        constexpr std::uint64_t kNodeGround = std::numeric_limits<std::uint64_t>::max() - 1;

        for(std::uint64_t mid{}; mid < model_count; ++mid)
        {
            if(mode == export_mode::runtime_only)
            {
                if(static_cast<std::size_t>(mid) >= model_map.size()) { return {errc::corrupt, "checkpoint: model map out of range"}; }
                auto const tgt = model_map.index_unchecked(static_cast<std::size_t>(mid));
                if(static_cast<std::size_t>(tgt) >= cur_models.size()) { return {errc::corrupt, "checkpoint: model target out of range"}; }
                auto* mb = cur_models.index_unchecked(static_cast<std::size_t>(tgt));
                if(mb == nullptr || mb->ptr == nullptr) { return {errc::corrupt, "checkpoint: model ptr null"}; }

                if(has_model_state)
                {
                    std::string state{};
                    auto st = details::get(*db, ro, details::key_u64("m/", mid, "/state"), state);
                    if(!st) { return st; }
                    auto const* codec = reg.find(mb->ptr->get_model_name());
                    if(codec == nullptr) { return {errc::unsupported, "no model codec registered for checkpoint apply"}; }
                    // Preserve connectivity across load (pins must keep their node pointers and stable addresses).
                    auto pv_before = mb->ptr->generate_pin_view();
                    ::std::vector<::phy_engine::model::node_t*> saved_nodes{};
                    ::std::vector<::fast_io::u8string_view> saved_names{};
                    saved_nodes.resize(pv_before.size);
                    saved_names.resize(pv_before.size);
                    for(std::size_t i{}; i < pv_before.size; ++i)
                    {
                        saved_nodes[i] = pv_before.pins[i].nodes;
                        saved_names[i] = pv_before.pins[i].name;
                    }

                    auto st2 = codec->load_checkpoint_state ? codec->load_checkpoint_state(*mb, state) : codec->load_state(*mb, state);
                    if(!st2)
                    {
                        if(opt.require_model_state) { return st2; }
                    }
                    auto pv_after = mb->ptr->generate_pin_view();
                    if(pv_after.size != pv_before.size) { return {errc::unsupported, "checkpoint load changed pin count"}; }
                    for(std::size_t i{}; i < pv_after.size; ++i)
                    {
                        pv_after.pins[i].nodes = saved_nodes[i];
                        pv_after.pins[i].name = saved_names[i];
                        pv_after.pins[i].model = mb;
                    }
                }
                continue;
            }

            // Full/structure load: needs model_name to instantiate.
            std::string model_name_bytes{};
            auto st = details::get(*db, ro, details::key_u64("m/", mid, "/model_name"), model_name_bytes);
            if(!st) { return st; }
            auto const model_name_u8 = details::bytes_to_u8string(model_name_bytes);
            auto const* codec = reg.find(::fast_io::u8string_view{model_name_u8.data(), model_name_u8.size()});
            if(codec == nullptr) { return {errc::unsupported, "no codec for model: " + model_name_bytes}; }

            auto am = codec->add_model(c.nl);
            auto* mb = am.mod;
            if(mb == nullptr || mb->ptr == nullptr) { return {errc::corrupt, "failed to create model"}; }

            if(has_structure)
            {
                // wrapper
                {
                    std::string w{};
                    st = details::get(*db, ro, details::key_u64("m/", mid, "/wrapper"), w);
                    if(!st) { return st; }
                    std::size_t off{};
                    std::uint64_t ident{};
                    st = details::read_uleb128(w, off, ident);
                    if(!st) { return st; }
                    if(off >= w.size()) { return {errc::corrupt, "wrapper missing has_init"}; }
                    mb->identification = static_cast<std::size_t>(ident);
                    mb->has_init = static_cast<std::uint8_t>(w[off++]) != 0;
                    std::string name{};
                    std::string desc{};
                    st = details::read_string(w, off, name);
                    if(!st) { return st; }
                    st = details::read_string(w, off, desc);
                    if(!st) { return st; }
                    mb->name = details::bytes_to_u8string(name);
                    mb->describe = details::bytes_to_u8string(desc);
                    if(off != w.size()) { return {errc::corrupt, "trailing bytes in wrapper"}; }
                }

                // Apply attributes first (stable parameters), then state (runtime).
                {
                    std::string attrs{};
                    st = details::get(*db, ro, details::key_u64("m/", mid, "/attrs"), attrs);
                    if(!st) { return st; }
                    st = details::apply_attributes(*mb, attrs);
                    if(!st) { return st; }
                }

                // Reset pin pointers and connect according to saved mapping.
                {
                    auto pv = mb->ptr->generate_pin_view();
                    for(std::size_t i{}; i < pv.size; ++i)
                    {
                        pv.pins[i].nodes = nullptr;
                        pv.pins[i].model = mb;
                    }

                    std::string pins{};
                    st = details::get(*db, ro, details::key_u64("m/", mid, "/pins"), pins);
                    if(!st) { return st; }
                    std::size_t off{};
                    std::uint64_t pcount{};
                    st = details::read_uleb128(pins, off, pcount);
                    if(!st) { return st; }
                    if(pcount != pv.size) { return {errc::corrupt, "pin count mismatch"}; }
                    for(std::size_t i{}; i < pv.size; ++i)
                    {
                        std::uint64_t idv{};
                        st = details::read_trivial(pins, off, idv);
                        if(!st) { return st; }
                        if(idv == kNodeNull) { continue; }
                        if(idv == kNodeGround)
                        {
                            (void)::phy_engine::netlist::add_to_node(c.nl, am.mod_pos, i, c.nl.ground_node);
                            continue;
                        }
                        if(idv >= node_count) { return {errc::corrupt, "invalid node id in pin mapping"}; }
                        (void)::phy_engine::netlist::add_to_node(c.nl, am.mod_pos, i, *id_to_node.index_unchecked(static_cast<std::size_t>(idv)));
                    }
                    if(off != pins.size()) { return {errc::corrupt, "trailing bytes in pins mapping"}; }
                }
            }

            if(has_model_state)
            {
                std::string state{};
                st = details::get(*db, ro, details::key_u64("m/", mid, "/state"), state);
                if(st)
                {
                    auto st2 = codec->load_state(*mb, state);
                    if(!st2)
                    {
                        if(opt.require_model_state) { return st2; }
                    }
                }
                else if(st.code != errc::not_found)
                {
                    return st;
                }
            }
        }

        if(!has_node_state && mode == export_mode::runtime_only)
        {
            // Checkpoint without node state: do not modify existing node values.
            return {};
        }

        // After all pins are connected (or for checkpoint apply: structure already exists),
        // choose node union field based on derived num_of_analog_node.
        for(std::uint64_t id{}; id < node_count; ++id)
        {
            auto* n = id_to_node.index_unchecked(static_cast<std::size_t>(id));
            if(n == nullptr) { continue; }
            if(n->num_of_analog_node == 0)
            {
                if(has_node_state) { n->node_information.dn.state = saved_node_digital.index_unchecked(static_cast<std::size_t>(id)); }
                else { n->node_information.dn.state = ::phy_engine::model::digital_node_statement_t::indeterminate_state; }
            }
            else
            {
                if(has_node_state) { n->node_information.an.voltage = saved_node_voltage.index_unchecked(static_cast<std::size_t>(id)); }
                else { n->node_information.an.voltage = {}; }
            }
        }

        return {};
    }

    inline status save(std::filesystem::path const& out_path,
                       ::phy_engine::circult const& c,
                       save_options opt = {},
                       model_registry const& reg = default_registry())
    {
        storage_layout layout = opt.layout;
        if(layout == storage_layout::auto_detect)
        {
            layout = std::filesystem::is_directory(out_path) ? storage_layout::directory : storage_layout::single_file;
        }

        if(layout == storage_layout::directory)
        {
            return save_to_leveldb(out_path, c, opt, reg);
        }

        if(std::filesystem::exists(out_path) && !opt.overwrite) { return {errc::invalid_argument, "output file exists"}; }

        // Write LevelDB to temp dir, then pack into a single file.
        auto const tmp_root = std::filesystem::temp_directory_path();
        std::filesystem::path tmp_dir{};
        auto const now = static_cast<std::uint64_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
        for(int i = 0; i < 256; ++i)
        {
            tmp_dir = tmp_root / ("pe_nl_tmp_" + std::to_string(now) + "_" + std::to_string(i));
            std::error_code ec;
            if(std::filesystem::create_directory(tmp_dir, ec)) { break; }
        }
        if(tmp_dir.empty() || !std::filesystem::exists(tmp_dir)) { return {errc::io_error, "failed to create temp directory"}; }

        save_options dir_opt = opt;
        dir_opt.layout = storage_layout::directory;
        dir_opt.overwrite = true;
        auto st = save_to_leveldb(tmp_dir, c, dir_opt, reg);
        if(!st)
        {
            std::error_code ec;
            std::filesystem::remove_all(tmp_dir, ec);
            return st;
        }

        // Pack to a sibling temp file then rename for best-effort atomicity.
        auto const tmp_file = out_path.string() + ".tmp";
        st = pack_directory_to_file(tmp_dir, tmp_file);
        std::error_code ec;
        std::filesystem::remove_all(tmp_dir, ec);
        if(!st) { return st; }

        std::filesystem::remove(out_path, ec);  // ignore error
        ec.clear();
        std::filesystem::rename(tmp_file, out_path, ec);
        if(ec)
        {
            // Fallback: copy+remove
            std::filesystem::copy_file(tmp_file, out_path, std::filesystem::copy_options::overwrite_existing, ec);
            if(ec) { return {errc::io_error, "failed to move packed file into place"}; }
            std::filesystem::remove(tmp_file, ec);
        }
        return st;
    }

    inline status load(std::filesystem::path const& in_path,
                       ::phy_engine::circult& c,
                       load_options opt = {},
                       model_registry const& reg = default_registry())
    {
        storage_layout layout = opt.layout;
        if(layout == storage_layout::auto_detect)
        {
            layout = std::filesystem::is_directory(in_path) ? storage_layout::directory : storage_layout::single_file;
        }

        if(layout == storage_layout::directory)
        {
            return load_from_leveldb(in_path, c, opt, reg);
        }

        // Unpack archive to temp dir, then open LevelDB.
        auto const tmp_root = std::filesystem::temp_directory_path();
        std::filesystem::path tmp_dir{};
        auto const now = static_cast<std::uint64_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
        for(int i = 0; i < 256; ++i)
        {
            tmp_dir = tmp_root / ("pe_nl_unpack_" + std::to_string(now) + "_" + std::to_string(i));
            std::error_code ec;
            if(std::filesystem::create_directory(tmp_dir, ec)) { break; }
        }
        if(tmp_dir.empty() || !std::filesystem::exists(tmp_dir)) { return {errc::io_error, "failed to create temp directory"}; }

        auto st = unpack_file_to_directory(in_path, tmp_dir);
        if(!st)
        {
            std::error_code ec;
            std::filesystem::remove_all(tmp_dir, ec);
            return st;
        }

        st = load_from_leveldb(tmp_dir, c, opt, reg);
        std::error_code ec;
        std::filesystem::remove_all(tmp_dir, ec);
        return st;
    }
}  // namespace phy_engine::pe_nl_fileformat
