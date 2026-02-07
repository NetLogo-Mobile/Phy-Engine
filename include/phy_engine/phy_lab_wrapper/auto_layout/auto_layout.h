#pragma once

#include "../physicslab.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <queue>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace phy_engine::phy_lab_wrapper::auto_layout
{
    struct bounds2d
    {
        double min_x{};
        double min_y{};
        double max_x{};
        double max_y{};
    };

    inline status_or<bounds2d> normalize_bounds_ec(position a, position b, double margin_x = 0.0, double margin_y = 0.0) noexcept
    {
        if(!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(b.x) || !std::isfinite(b.y))
        {
            std::string msg = "auto_layout: bounds must be finite";
            ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
            return status{std::errc::invalid_argument, std::move(msg)};
        }
        if(!std::isfinite(margin_x) || !std::isfinite(margin_y) || margin_x < 0.0 || margin_y < 0.0)
        {
            std::string msg = "auto_layout: margins must be finite and >= 0";
            ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
            return status{std::errc::invalid_argument, std::move(msg)};
        }

        bounds2d out{
            .min_x = std::min(a.x, b.x) + margin_x,
            .min_y = std::min(a.y, b.y) + margin_y,
            .max_x = std::max(a.x, b.x) - margin_x,
            .max_y = std::max(a.y, b.y) - margin_y,
        };
        if(out.max_x < out.min_x || out.max_y < out.min_y)
        {
            std::string msg = "auto_layout: bounds too small after margins";
            ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
            return status{std::errc::invalid_argument, std::move(msg)};
        }
        return out;
    }

#if PHY_ENGINE_ENABLE_EXCEPTIONS
    inline bounds2d normalize_bounds(position a, position b, double margin_x = 0.0, double margin_y = 0.0)
    {
        auto r = normalize_bounds_ec(a, b, margin_x, margin_y);
        if(!r) { throw std::invalid_argument(r.st.message); }
        return std::move(*r.value);
    }
#endif

    enum class backend : int
    {
        cpu = 0,
        cuda = 1,
    };

    enum class mode : int
    {
        fast = 0,
        cluster = 1,
        spectral = 2,
        hierarchical = 3,
        force = 4,
    };

    struct footprint
    {
        std::size_t w{1};
        std::size_t h{1};
    };

    struct cuda_dispatch
    {
        using fn_t = void (*)(experiment& ex, bounds2d const& bounds, double z_fixed, void const* opt_opaque);
        fn_t fn{nullptr};
        void const* opt_opaque{nullptr};
    };

    struct options
    {
        backend backend_kind{backend::cpu};
        mode layout_mode{mode::fast};

        // Discretize positions in native coordinates.
        double step_x{element_xyz::x_unit};
        double step_y{element_xyz::y_unit};

        // Reserve cells around the bounds (in native units).
        double margin_x{0.0};
        double margin_y{0.0};

        // Treat non-participating elements as fixed obstacles.
        bool respect_fixed_elements{true};

        // Rough occupancy model (in grid cells).
        footprint small_element{1, 1};
        footprint big_element{2, 2};

        // Candidate search limits.
        std::size_t max_candidates_per_element{4096};
        std::optional<std::size_t> max_search_radius{};

        // Spectral / force settings.
        std::uint32_t random_seed{0xC0FFEEu};
        std::size_t spectral_iterations{64};   // power-iteration count
        std::size_t spectral_eigenvectors{3};  // compute k eigenvectors (uses v2,v3 as (x,y))
        std::size_t force_iterations{200};     // continuous refine steps
        std::size_t force_bins{32};            // spatial hash bins per axis
        double force_attraction{0.01};         // edge attraction strength
        double force_repulsion{0.0005};        // local repulsion strength

        // Cluster (chip-like) settings.
        std::size_t cluster_label_iterations{16};
        std::size_t cluster_max_nodes{64};
        std::size_t cluster_channel_spacing{1};  // empty cells between macro blocks
        double cluster_macro_ideal_weight{1.0};
        double cluster_macro_neighbor_weight{1.0};

        // Optional dispatch hook for a future CUDA implementation.
        cuda_dispatch cuda{};
    };

    struct stats
    {
        mode layout_mode{mode::fast};
        std::size_t grid_w{};
        std::size_t grid_h{};
        double step_x{};
        double step_y{};
        std::size_t fixed_obstacles{};
        std::size_t placed{};
        std::size_t skipped{};
    };

    // NOTE: All `*_3d` APIs use a fixed Z spacing in native coordinates.
    inline constexpr double z_step_3d{0.02};

    namespace detail
    {
        struct cell
        {
            int x{};
            int y{};
        };

        struct window
        {
            int min_x{};
            int min_y{};
            int max_x{};
            int max_y{};
        };

        inline bool in_range(int v, int lo, int hi) noexcept { return v >= lo && v <= hi; }

        inline footprint element_footprint(element const& e, options const& opt) noexcept { return e.is_big_element() ? opt.big_element : opt.small_element; }

        inline std::size_t grid_index(std::size_t x, std::size_t y, std::size_t w) noexcept { return y * w + x; }

        struct occupancy
        {
            std::size_t w{};
            std::size_t h{};
            std::vector<int> cells{};  // -1 empty; otherwise occupied.

            occupancy(std::size_t w_, std::size_t h_) : w(w_), h(h_), cells(w_ * h_, -1) {}

            [[nodiscard]] bool can_place(cell c, footprint fp) const noexcept
            {
                if(c.x < 0 || c.y < 0) { return false; }
                auto const ux = static_cast<std::size_t>(c.x);
                auto const uy = static_cast<std::size_t>(c.y);
                if(ux + fp.w > w || uy + fp.h > h) { return false; }
                for(std::size_t dy{}; dy < fp.h; ++dy)
                {
                    for(std::size_t dx{}; dx < fp.w; ++dx)
                    {
                        if(cells[grid_index(ux + dx, uy + dy, w)] != -1) { return false; }
                    }
                }
                return true;
            }

            void occupy(cell c, footprint fp, int tag)
            {
                auto const ux = static_cast<std::size_t>(c.x);
                auto const uy = static_cast<std::size_t>(c.y);
                for(std::size_t dy{}; dy < fp.h; ++dy)
                {
                    for(std::size_t dx{}; dx < fp.w; ++dx) { cells[grid_index(ux + dx, uy + dy, w)] = tag; }
                }
            }
        };

        inline std::size_t grid_w_from(bounds2d const& b, double step_x) noexcept
        {
            auto const span = b.max_x - b.min_x;
            if(!(span >= 0.0) || !(step_x > 0.0) || !std::isfinite(step_x))
            {
                ::phy_engine::phy_lab_wrapper::detail::set_last_error("auto_layout: invalid step_x");
                return 0;
            }
            return static_cast<std::size_t>(std::floor(span / step_x + 1e-12)) + 1;
        }

        inline std::size_t grid_h_from(bounds2d const& b, double step_y) noexcept
        {
            auto const span = b.max_y - b.min_y;
            if(!(span >= 0.0) || !(step_y > 0.0) || !std::isfinite(step_y))
            {
                ::phy_engine::phy_lab_wrapper::detail::set_last_error("auto_layout: invalid step_y");
                return 0;
            }
            return static_cast<std::size_t>(std::floor(span / step_y + 1e-12)) + 1;
        }

        inline cell snap_native_to_cell(bounds2d const& b, double step_x, double step_y, position p)
        {
            auto const fx = (p.x - b.min_x) / step_x;
            auto const fy = (p.y - b.min_y) / step_y;
            if(!std::isfinite(fx) || !std::isfinite(fy)) { return cell{-1, -1}; }
            auto const ix = static_cast<int>(std::llround(fx));
            auto const iy = static_cast<int>(std::llround(fy));
            return cell{ix, iy};
        }

        inline position cell_to_native(bounds2d const& b, double step_x, double step_y, cell c, double z_fixed) noexcept
        {
            return position{
                b.min_x + static_cast<double>(c.x) * step_x,
                b.min_y + static_cast<double>(c.y) * step_y,
                z_fixed,
            };
        }

        inline void set_element_native_position(experiment const& ex, element& e, position native_pos)
        {
            if(e.is_element_xyz())
            {
                auto const origin = ex.element_xyz_origin();
                position el_pos{
                    (native_pos.x - origin.x) / element_xyz::x_unit,
                    (native_pos.y - origin.y) / element_xyz::y_unit,
                    (native_pos.z - origin.z) / element_xyz::z_unit,
                };
                if(e.is_big_element())
                {
                    // Invert `element_xyz::to_native` exactly so big-element Y amendments round-trip.
                    el_pos.y -= (element_xyz::y_amend_big_element / element_xyz::y_unit);
                }
                e.set_element_position(el_pos, true);
                return;
            }
            e.set_element_position(native_pos, false);
        }

        inline std::vector<std::vector<std::size_t>> build_adjacency(experiment const& ex, std::unordered_map<std::string, std::size_t> const& id_to_index)
        {
            std::vector<std::vector<std::size_t>> adj(ex.elements().size());
            for(auto const& w: ex.wires())
            {
                auto it_s = id_to_index.find(w.source.element_identifier);
                auto it_t = id_to_index.find(w.target.element_identifier);
                if(it_s == id_to_index.end() || it_t == id_to_index.end()) { continue; }
                auto const s = it_s->second;
                auto const t = it_t->second;
                if(s == t) { continue; }
                adj[s].push_back(t);
                adj[t].push_back(s);
            }
            for(auto& v: adj)
            {
                std::sort(v.begin(), v.end());
                v.erase(std::unique(v.begin(), v.end()), v.end());
            }
            return adj;
        }

        inline double placement_cost(cell candidate,
                                     cell ideal,
                                     std::vector<std::size_t> const& neighbors,
                                     std::vector<std::optional<cell>> const& placed,
                                     double neighbor_w,
                                     double ideal_w)
        {
            double cost = 0.0;
            for(auto ni: neighbors)
            {
                if(!placed[ni]) { continue; }
                auto const p = *placed[ni];
                cost += neighbor_w * static_cast<double>(std::abs(candidate.x - p.x) + std::abs(candidate.y - p.y));
            }
            cost += ideal_w * static_cast<double>(std::abs(candidate.x - ideal.x) + std::abs(candidate.y - ideal.y));
            return cost;
        }

        inline std::optional<cell> choose_cell(detail::occupancy const& occ,
                                               cell ideal,
                                               footprint fp,
                                               std::vector<std::size_t> const& neighbors,
                                               std::vector<std::optional<cell>> const& placed,
                                               std::size_t max_candidates,
                                               std::optional<std::size_t> max_radius_opt,
                                               double neighbor_w,
                                               double ideal_w,
                                               std::optional<window> win = std::nullopt)
        {
            auto const max_radius = [&]() -> std::size_t
            {
                if(max_radius_opt) { return *max_radius_opt; }
                return static_cast<std::size_t>(std::max<int>(static_cast<int>(occ.w), static_cast<int>(occ.h)));
            }();

            std::optional<cell> best{};
            double best_cost = std::numeric_limits<double>::infinity();
            std::size_t visited{};

            auto consider = [&](cell c)
            {
                if(visited >= max_candidates) { return; }
                ++visited;
                if(win)
                {
                    if(c.x < win->min_x || c.y < win->min_y || c.x > win->max_x || c.y > win->max_y) { return; }
                }
                if(!occ.can_place(c, fp)) { return; }
                auto const cost = placement_cost(c, ideal, neighbors, placed, neighbor_w, ideal_w);
                if(!best || cost < best_cost ||
                   (cost == best_cost && (std::abs(c.x - ideal.x) + std::abs(c.y - ideal.y)) < (std::abs(best->x - ideal.x) + std::abs(best->y - ideal.y))) ||
                   (cost == best_cost && c.x < best->x) || (cost == best_cost && c.x == best->x && c.y < best->y))
                {
                    best = c;
                    best_cost = cost;
                }
            };

            auto const lo_x = 0;
            auto const lo_y = 0;
            auto const hi_x = static_cast<int>(occ.w) - 1;
            auto const hi_y = static_cast<int>(occ.h) - 1;

            ideal.x = std::clamp(ideal.x, lo_x, hi_x);
            ideal.y = std::clamp(ideal.y, lo_y, hi_y);

            for(std::size_t r{}; r <= max_radius && visited < max_candidates; ++r)
            {
                auto const ir = static_cast<int>(r);
                if(r == 0)
                {
                    consider(ideal);
                    continue;
                }

                // Top and bottom edges of the ring.
                for(int dx = -ir; dx <= ir && visited < max_candidates; ++dx)
                {
                    cell c1{ideal.x + dx, ideal.y - ir};
                    cell c2{ideal.x + dx, ideal.y + ir};
                    if(in_range(c1.x, lo_x, hi_x) && in_range(c1.y, lo_y, hi_y)) { consider(c1); }
                    if(in_range(c2.x, lo_x, hi_x) && in_range(c2.y, lo_y, hi_y)) { consider(c2); }
                }
                // Left and right edges (excluding corners already checked).
                for(int dy = -ir + 1; dy <= ir - 1 && visited < max_candidates; ++dy)
                {
                    cell c1{ideal.x - ir, ideal.y + dy};
                    cell c2{ideal.x + ir, ideal.y + dy};
                    if(in_range(c1.x, lo_x, hi_x) && in_range(c1.y, lo_y, hi_y)) { consider(c1); }
                    if(in_range(c2.x, lo_x, hi_x) && in_range(c2.y, lo_y, hi_y)) { consider(c2); }
                }

                if(best_cost == 0.0) { break; }
            }
            return best;
        }

        inline cell default_seed_cell(std::size_t w, std::size_t h) noexcept { return cell{static_cast<int>(w / 2), static_cast<int>(h / 2)}; }

        struct context
        {
            experiment& ex;
            bounds2d bounds;
            std::size_t w{};
            std::size_t h{};
            occupancy occ;
            std::unordered_map<std::string, std::size_t> id_to_index;
            std::vector<std::vector<std::size_t>> adj;
            std::vector<std::optional<cell>> placed;
            std::vector<std::size_t> movable;
            std::size_t fixed_obstacles{};

            context(experiment& ex_, bounds2d bounds_, std::size_t w_, std::size_t h_) : ex(ex_), bounds(bounds_), w(w_), h(h_), occ(w_, h_) {}
        };

        inline status_or<context> build_context_ec(experiment& ex, position corner0, position corner1, options const& opt) noexcept
        {
            auto bounds_r = normalize_bounds_ec(corner0, corner1, opt.margin_x, opt.margin_y);
            if(!bounds_r) { return bounds_r.st; }
            auto const bounds = *bounds_r.value;

            if(!(opt.step_x > 0.0) || !std::isfinite(opt.step_x))
            {
                std::string msg = "auto_layout: invalid step_x";
                ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                return status{std::errc::invalid_argument, std::move(msg)};
            }
            if(!(opt.step_y > 0.0) || !std::isfinite(opt.step_y))
            {
                std::string msg = "auto_layout: invalid step_y";
                ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                return status{std::errc::invalid_argument, std::move(msg)};
            }

            auto const w = grid_w_from(bounds, opt.step_x);
            auto const h = grid_h_from(bounds, opt.step_y);
            if(w == 0 || h == 0)
            {
                std::string msg = "auto_layout: empty grid";
                ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                return status{std::errc::invalid_argument, std::move(msg)};
            }

            constexpr std::size_t kMaxGridCells = 2'000'000;
            if(w > 0 && h > 0 && w > (kMaxGridCells / h))
            {
                std::string msg = "auto_layout: grid too large (reduce bounds or increase step)";
                ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                return status{std::errc::invalid_argument, std::move(msg)};
            }

            context ctx(ex, bounds, w, h);

            ctx.id_to_index.reserve(ex.elements().size());
            for(std::size_t i{}; i < ex.elements().size(); ++i)
            {
                auto id_r = ex.elements()[i].identifier_ec();
                if(!id_r) { return id_r.st; }
                ctx.id_to_index.emplace(*id_r.value, i);
            }

            ctx.adj = build_adjacency(ex, ctx.id_to_index);
            ctx.placed.assign(ex.elements().size(), std::nullopt);
            ctx.movable.reserve(ex.elements().size());

            for(std::size_t i{}; i < ex.elements().size(); ++i)
            {
                auto const& e = ex.elements()[i];
                if(e.participate_in_layout())
                {
                    ctx.movable.push_back(i);
                    continue;
                }
                if(!opt.respect_fixed_elements) { continue; }

                auto const native_pos =
                    e.is_element_xyz() ? element_xyz::to_native(e.element_position(), ex.element_xyz_origin(), e.is_big_element()) : e.element_position();
                auto c = snap_native_to_cell(bounds, opt.step_x, opt.step_y, native_pos);
                // Fixed elements outside the layout bounds should not be treated as obstacles.
                if(c.x < 0 || c.y < 0) { continue; }
                auto const fp = element_footprint(e, opt);
                if(w < fp.w || h < fp.h)
                {
                    std::string msg = "auto_layout: bounds too small for element footprint";
                    ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                    return status{std::errc::invalid_argument, std::move(msg)};
                }
                c.x = std::clamp(c.x, 0, static_cast<int>(w - fp.w));
                c.y = std::clamp(c.y, 0, static_cast<int>(h - fp.h));
                if(!ctx.occ.can_place(c, fp)) { continue; }
                ctx.occ.occupy(c, fp, static_cast<int>(i));
                ctx.placed[i] = c;
                ++ctx.fixed_obstacles;
            }
            return ctx;
        }

        inline status apply_placements_ec(experiment& ex,
                                         bounds2d const& bounds,
                                         double step_x,
                                         double step_y,
                                         double z_fixed,
                                         std::vector<std::size_t> const& movable,
                                         std::vector<std::optional<cell>> const& placed) noexcept
        {
            for(auto idx: movable)
            {
                if(!placed[idx]) { continue; }
                auto const native_pos = cell_to_native(bounds, step_x, step_y, *placed[idx], z_fixed);
                auto id_r = ex.elements()[idx].identifier_ec();
                if(!id_r) { return id_r.st; }
                auto* el = ex.find_element(*id_r.value);
                if(el == nullptr)
                {
                    auto msg = "auto_layout: unknown element identifier: " + *id_r.value;
                    ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                    return status{std::errc::invalid_argument, std::move(msg)};
                }
                set_element_native_position(ex, *el, native_pos);
            }
            return {};
        }

        inline std::optional<window> full_window(std::size_t w, std::size_t h, footprint fp) noexcept
        {
            if(w < fp.w || h < fp.h) { return std::nullopt; }
            return window{
                .min_x = 0,
                .min_y = 0,
                .max_x = static_cast<int>(w - fp.w),
                .max_y = static_cast<int>(h - fp.h),
            };
        }

        inline status_or<stats> layout_cpu_fast_ec(experiment& ex, position corner0, position corner1, double z_fixed, options const& opt) noexcept
        {
            if(!std::isfinite(z_fixed))
            {
                std::string msg = "auto_layout: z_fixed must be finite";
                ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                return status{std::errc::invalid_argument, std::move(msg)};
            }

            auto ctx_r = build_context_ec(ex, corner0, corner1, opt);
            if(!ctx_r) { return ctx_r.st; }
            auto ctx = std::move(*ctx_r.value);

            auto degree = [&](std::size_t idx) -> std::size_t { return ctx.adj[idx].size(); };
            std::stable_sort(ctx.movable.begin(),
                             ctx.movable.end(),
                             [&](std::size_t a, std::size_t b)
                             {
                                 auto const& ea = ex.elements()[a];
                                 auto const& eb = ex.elements()[b];
                                 if(ea.is_big_element() != eb.is_big_element()) { return ea.is_big_element() > eb.is_big_element(); }
                                 auto const da = degree(a);
                                 auto const db = degree(b);
                                 if(da != db) { return da > db; }
                                 return ea.identifier() < eb.identifier();
                             });

            auto seed = default_seed_cell(ctx.w, ctx.h);

            std::size_t placed_count{};
            std::size_t skipped_count{};

            for(auto idx: ctx.movable)
            {
                auto const fp = element_footprint(ex.elements()[idx], opt);

                cell ideal = seed;
                {
                    std::int64_t sum_x{};
                    std::int64_t sum_y{};
                    std::size_t cnt{};
                    for(auto nb: ctx.adj[idx])
                    {
                        if(!ctx.placed[nb]) { continue; }
                        sum_x += ctx.placed[nb]->x;
                        sum_y += ctx.placed[nb]->y;
                        ++cnt;
                    }
                    if(cnt != 0)
                    {
                        ideal.x = static_cast<int>(std::llround(static_cast<double>(sum_x) / static_cast<double>(cnt)));
                        ideal.y = static_cast<int>(std::llround(static_cast<double>(sum_y) / static_cast<double>(cnt)));
                    }
                }

                auto chosen = choose_cell(ctx.occ, ideal, fp, ctx.adj[idx], ctx.placed, opt.max_candidates_per_element, opt.max_search_radius, 1.0, 0.1);
                if(!chosen)
                {
                    ++skipped_count;
                    continue;
                }

                ctx.occ.occupy(*chosen, fp, static_cast<int>(idx));
                ctx.placed[idx] = *chosen;
                ++placed_count;
            }

            auto st = apply_placements_ec(ex, ctx.bounds, opt.step_x, opt.step_y, z_fixed, ctx.movable, ctx.placed);
            if(!st) { return st; }

            return stats{
                .layout_mode = opt.layout_mode,
                .grid_w = ctx.w,
                .grid_h = ctx.h,
                .step_x = opt.step_x,
                .step_y = opt.step_y,
                .fixed_obstacles = ctx.fixed_obstacles,
                .placed = placed_count,
                .skipped = skipped_count,
            };
        }

        struct weighted_edge
        {
            std::size_t to{};
            double w{};
        };

        using weighted_adj = std::vector<std::vector<weighted_edge>>;

        inline double dot(std::vector<double> const& a, std::vector<double> const& b) noexcept
        {
            double s = 0.0;
            auto const n = std::min(a.size(), b.size());
            for(std::size_t i{}; i < n; ++i) { s += a[i] * b[i]; }
            return s;
        }

        inline void normalize_l2(std::vector<double>& v) noexcept
        {
            double ss = 0.0;
            for(double x: v) { ss += x * x; }
            if(ss <= 0.0) { return; }
            double const inv = 1.0 / std::sqrt(ss);
            for(double& x: v) { x *= inv; }
        }

        inline void orthogonalize(std::vector<double>& v, std::vector<std::vector<double>> const& basis) noexcept
        {
            for(auto const& b: basis)
            {
                auto const proj = dot(v, b);
                for(std::size_t i{}; i < v.size(); ++i) { v[i] -= proj * b[i]; }
            }
        }

        inline std::vector<double> degrees(weighted_adj const& g)
        {
            std::vector<double> deg(g.size(), 0.0);
            for(std::size_t i{}; i < g.size(); ++i)
            {
                double s = 0.0;
                for(auto const& e: g[i]) { s += e.w; }
                deg[i] = s;
            }
            return deg;
        }

        inline void multiply_norm_adj(weighted_adj const& g, std::vector<double> const& deg, std::vector<double> const& in, std::vector<double>& out) noexcept
        {
            out.assign(g.size(), 0.0);
            for(std::size_t i{}; i < g.size(); ++i)
            {
                auto const di = deg[i];
                if(di <= 0.0) { continue; }
                auto const inv_sqrt_di = 1.0 / std::sqrt(di);
                for(auto const& e: g[i])
                {
                    auto const j = e.to;
                    auto const dj = deg[j];
                    if(dj <= 0.0) { continue; }
                    auto const inv_sqrt_dj = 1.0 / std::sqrt(dj);
                    out[i] += e.w * (inv_sqrt_di * inv_sqrt_dj) * in[j];
                }
            }
        }

        inline std::vector<std::vector<double>> top_eigenvectors(weighted_adj const& g, std::size_t k, std::size_t iters, std::uint32_t seed)
        {
            auto const n = g.size();
            if(n == 0 || k == 0) { return {}; }

            std::mt19937 rng(seed);
            std::uniform_real_distribution<double> dist(-1.0, 1.0);
            auto const deg = degrees(g);

            std::vector<std::vector<double>> vecs;
            vecs.reserve(k);

            std::vector<double> v(n);
            std::vector<double> w;
            w.reserve(n);

            for(std::size_t ei{}; ei < k; ++ei)
            {
                for(double& x: v) { x = dist(rng); }
                orthogonalize(v, vecs);
                normalize_l2(v);

                for(std::size_t it{}; it < iters; ++it)
                {
                    multiply_norm_adj(g, deg, v, w);
                    orthogonalize(w, vecs);
                    normalize_l2(w);

                    // If we collapsed (e.g., disconnected graph), re-seed.
                    double ss = 0.0;
                    for(double x: w) { ss += x * x; }
                    if(ss <= 1e-18)
                    {
                        for(double& x: v) { x = dist(rng); }
                        orthogonalize(v, vecs);
                        normalize_l2(v);
                        continue;
                    }
                    v.swap(w);
                }

                // One more multiply to reduce numerical drift.
                multiply_norm_adj(g, deg, v, w);
                orthogonalize(w, vecs);
                normalize_l2(w);
                if(w.size() == n) { vecs.push_back(w); }
                else
                {
                    vecs.push_back(v);
                }
            }
            return vecs;
        }

        struct embedding2d
        {
            std::vector<double> x;
            std::vector<double> y;
        };

        struct embedding3d
        {
            std::vector<double> x;
            std::vector<double> y;
            std::vector<double> z;
        };

        inline void normalize_to_unit(std::vector<double>& v) noexcept
        {
            if(v.empty()) { return; }
            auto [mn_it, mx_it] = std::minmax_element(v.begin(), v.end());
            auto const mn = *mn_it;
            auto const mx = *mx_it;
            auto const span = mx - mn;
            if(!(span > 0.0) || !std::isfinite(span))
            {
                std::fill(v.begin(), v.end(), 0.5);
                return;
            }
            for(double& x: v) { x = (x - mn) / span; }
        }

        inline embedding2d spectral_embedding(weighted_adj const& g, options const& opt)
        {
            auto const n = g.size();
            embedding2d out{};
            out.x.assign(n, 0.5);
            out.y.assign(n, 0.5);
            if(n == 0) { return out; }
            if(n == 1) { return out; }

            auto const k = std::min<std::size_t>(std::max<std::size_t>(3, opt.spectral_eigenvectors), n);
            auto vecs = top_eigenvectors(g, k, opt.spectral_iterations, opt.random_seed);
            if(vecs.size() >= 2) { out.x = vecs[1]; }
            if(vecs.size() >= 3) { out.y = vecs[2]; }
            normalize_to_unit(out.x);
            normalize_to_unit(out.y);
            return out;
        }

        inline embedding3d spectral_embedding3d(weighted_adj const& g, options const& opt)
        {
            auto const n = g.size();
            embedding3d out{};
            out.x.assign(n, 0.5);
            out.y.assign(n, 0.5);
            out.z.assign(n, 0.5);
            if(n == 0) { return out; }
            if(n == 1) { return out; }

            auto const k = std::min<std::size_t>(std::max<std::size_t>(4, opt.spectral_eigenvectors), n);
            auto vecs = top_eigenvectors(g, k, opt.spectral_iterations, opt.random_seed);
            if(vecs.size() >= 2) { out.x = vecs[1]; }
            if(vecs.size() >= 3) { out.y = vecs[2]; }
            if(vecs.size() >= 4) { out.z = vecs[3]; }
            normalize_to_unit(out.x);
            normalize_to_unit(out.y);
            normalize_to_unit(out.z);
            return out;
        }

        inline status_or<stats> layout_cpu_spectral_ec(experiment& ex, position corner0, position corner1, double z_fixed, options const& opt) noexcept
        {
            if(!std::isfinite(z_fixed))
            {
                std::string msg = "auto_layout: z_fixed must be finite";
                ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                return status{std::errc::invalid_argument, std::move(msg)};
            }

            auto ctx_r = build_context_ec(ex, corner0, corner1, opt);
            if(!ctx_r) { return ctx_r.st; }
            auto ctx = std::move(*ctx_r.value);
            if(ctx.movable.empty())
            {
                return stats{
                    .layout_mode = opt.layout_mode,
                    .grid_w = ctx.w,
                    .grid_h = ctx.h,
                    .step_x = opt.step_x,
                    .step_y = opt.step_y,
                    .fixed_obstacles = ctx.fixed_obstacles,
                    .placed = 0,
                    .skipped = 0,
                };
            }

            std::vector<int> to_sub(ex.elements().size(), -1);
            for(std::size_t i{}; i < ctx.movable.size(); ++i) { to_sub[ctx.movable[i]] = static_cast<int>(i); }

            weighted_adj g(ctx.movable.size());
            for(std::size_t si{}; si < ctx.movable.size(); ++si)
            {
                auto const oi = ctx.movable[si];
                for(auto const nb: ctx.adj[oi])
                {
                    auto const sj = to_sub[nb];
                    if(sj < 0) { continue; }
                    if(static_cast<std::size_t>(sj) == si) { continue; }
                    g[si].push_back(weighted_edge{static_cast<std::size_t>(sj), 1.0});
                }
            }

            auto const emb = spectral_embedding(g, opt);

            auto degree = [&](std::size_t idx) -> std::size_t { return ctx.adj[idx].size(); };
            std::stable_sort(ctx.movable.begin(),
                             ctx.movable.end(),
                             [&](std::size_t a, std::size_t b)
                             {
                                 auto const& ea = ex.elements()[a];
                                 auto const& eb = ex.elements()[b];
                                 if(ea.is_big_element() != eb.is_big_element()) { return ea.is_big_element() > eb.is_big_element(); }
                                 auto const da = degree(a);
                                 auto const db = degree(b);
                                 if(da != db) { return da > db; }
                                 return ea.identifier() < eb.identifier();
                             });

            std::size_t placed_count{};
            std::size_t skipped_count{};

            for(auto idx: ctx.movable)
            {
                auto const fp = element_footprint(ex.elements()[idx], opt);
                auto const win = full_window(ctx.w, ctx.h, fp);
                if(!win)
                {
                    std::string msg = "auto_layout: bounds too small for element footprint";
                    ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                    return status{std::errc::invalid_argument, std::move(msg)};
                }

                auto const si = to_sub[idx];
                auto const u = (si >= 0 && static_cast<std::size_t>(si) < emb.x.size()) ? emb.x[static_cast<std::size_t>(si)] : 0.5;
                auto const v = (si >= 0 && static_cast<std::size_t>(si) < emb.y.size()) ? emb.y[static_cast<std::size_t>(si)] : 0.5;

                cell ideal{
                    static_cast<int>(std::llround(u * static_cast<double>(ctx.w - 1))),
                    static_cast<int>(std::llround(v * static_cast<double>(ctx.h - 1))),
                };

                auto chosen = choose_cell(ctx.occ, ideal, fp, ctx.adj[idx], ctx.placed, opt.max_candidates_per_element, opt.max_search_radius, 0.4, 1.0, win);
                if(!chosen)
                {
                    ++skipped_count;
                    continue;
                }

                ctx.occ.occupy(*chosen, fp, static_cast<int>(idx));
                ctx.placed[idx] = *chosen;
                ++placed_count;
            }

            auto st = apply_placements_ec(ex, ctx.bounds, opt.step_x, opt.step_y, z_fixed, ctx.movable, ctx.placed);
            if(!st) { return st; }

            return stats{
                .layout_mode = opt.layout_mode,
                .grid_w = ctx.w,
                .grid_h = ctx.h,
                .step_x = opt.step_x,
                .step_y = opt.step_y,
                .fixed_obstacles = ctx.fixed_obstacles,
                .placed = placed_count,
                .skipped = skipped_count,
            };
        }

        inline bool is_input_like(std::string_view model_id) noexcept { return model_id == "Logic Input" || model_id == "8bit Input"; }

        inline bool is_output_like(std::string_view model_id) noexcept { return model_id == "Logic Output" || model_id == "8bit Display"; }

        inline status_or<stats> layout_cpu_hierarchical_ec(experiment& ex, position corner0, position corner1, double z_fixed, options const& opt) noexcept
        {
            if(!std::isfinite(z_fixed))
            {
                std::string msg = "auto_layout: z_fixed must be finite";
                ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                return status{std::errc::invalid_argument, std::move(msg)};
            }

            auto ctx_r = build_context_ec(ex, corner0, corner1, opt);
            if(!ctx_r) { return ctx_r.st; }
            auto ctx = std::move(*ctx_r.value);
            if(ctx.movable.empty())
            {
                return stats{
                    .layout_mode = opt.layout_mode,
                    .grid_w = ctx.w,
                    .grid_h = ctx.h,
                    .step_x = opt.step_x,
                    .step_y = opt.step_y,
                    .fixed_obstacles = ctx.fixed_obstacles,
                    .placed = 0,
                    .skipped = 0,
                };
            }

            auto const n = ex.elements().size();
            std::vector<int> level(n, -1);
            std::queue<std::size_t> q;

            for(std::size_t i{}; i < n; ++i)
            {
                auto const mid = ex.elements()[i].data().value("ModelID", "");
                if(is_input_like(mid))
                {
                    level[i] = 0;
                    q.push(i);
                }
            }

            if(q.empty())
            {
                // Fallback: use highest-degree movable node as source.
                auto best = ctx.movable.front();
                for(auto idx: ctx.movable)
                {
                    if(ctx.adj[idx].size() > ctx.adj[best].size()) { best = idx; }
                }
                level[best] = 0;
                q.push(best);
            }

            while(!q.empty())
            {
                auto const v = q.front();
                q.pop();
                auto const lv = level[v];
                for(auto nb: ctx.adj[v])
                {
                    if(level[nb] != -1) { continue; }
                    level[nb] = lv + 1;
                    q.push(nb);
                }
            }

            int max_level{};
            for(auto idx: ctx.movable)
            {
                auto const lv = (level[idx] < 0) ? 0 : level[idx];
                level[idx] = lv;
                if(lv > max_level) { max_level = lv; }
            }

            auto const L = static_cast<std::size_t>(max_level);
            std::vector<std::vector<std::size_t>> layers(L + 1);
            for(auto idx: ctx.movable) { layers[static_cast<std::size_t>(level[idx])].push_back(idx); }

            auto degree = [&](std::size_t idx) -> std::size_t { return ctx.adj[idx].size(); };
            for(auto& layer_nodes: layers)
            {
                std::stable_sort(layer_nodes.begin(),
                                 layer_nodes.end(),
                                 [&](std::size_t a, std::size_t b)
                                 {
                                     auto const& ea = ex.elements()[a];
                                     auto const& eb = ex.elements()[b];
                                     if(ea.is_big_element() != eb.is_big_element()) { return ea.is_big_element() > eb.is_big_element(); }
                                     auto const da = degree(a);
                                     auto const db = degree(b);
                                     if(da != db) { return da > db; }
                                     return ea.identifier() < eb.identifier();
                                 });
            }

            // Order refinement: barycenter passes to reduce crossings.
            std::vector<std::size_t> pos_in_layer(n, 0);
            auto rebuild_pos = [&](std::size_t l)
            {
                auto const& nodes = layers[l];
                for(std::size_t i{}; i < nodes.size(); ++i) { pos_in_layer[nodes[i]] = i; }
            };
            for(std::size_t l{}; l < layers.size(); ++l) { rebuild_pos(l); }

            auto barycenter_key = [&](std::size_t idx, int ref_level, std::size_t fallback) -> double
            {
                double sum = 0.0;
                std::size_t cnt{};
                for(auto nb: ctx.adj[idx])
                {
                    if(level[nb] != ref_level) { continue; }
                    sum += static_cast<double>(pos_in_layer[nb]);
                    ++cnt;
                }
                if(cnt == 0) { return static_cast<double>(fallback); }
                return sum / static_cast<double>(cnt);
            };

            for(std::size_t it{}; it < 4; ++it)
            {
                for(std::size_t l{1}; l < layers.size(); ++l)
                {
                    auto& nodes = layers[l];
                    std::stable_sort(nodes.begin(),
                                     nodes.end(),
                                     [&](std::size_t a, std::size_t b)
                                     {
                                         auto const ka = barycenter_key(a, static_cast<int>(l) - 1, pos_in_layer[a]);
                                         auto const kb = barycenter_key(b, static_cast<int>(l) - 1, pos_in_layer[b]);
                                         if(ka != kb) { return ka < kb; }
                                         return ex.elements()[a].identifier() < ex.elements()[b].identifier();
                                     });
                    rebuild_pos(l);
                }
                for(std::size_t l = layers.size(); l-- > 1;)
                {
                    auto& nodes = layers[l - 1];
                    std::stable_sort(nodes.begin(),
                                     nodes.end(),
                                     [&](std::size_t a, std::size_t b)
                                     {
                                         auto const ka = barycenter_key(a, static_cast<int>(l), pos_in_layer[a]);
                                         auto const kb = barycenter_key(b, static_cast<int>(l), pos_in_layer[b]);
                                         if(ka != kb) { return ka < kb; }
                                         return ex.elements()[a].identifier() < ex.elements()[b].identifier();
                                     });
                    rebuild_pos(l - 1);
                }
            }

            // Place layer-by-layer from sources to sinks.
            std::size_t placed_count{};
            std::size_t skipped_count{};

            for(std::size_t l{}; l < layers.size(); ++l)
            {
                auto const& nodes = layers[l];
                if(nodes.empty()) { continue; }
                auto const k = nodes.size();
                for(std::size_t rank{}; rank < k; ++rank)
                {
                    auto const idx = nodes[rank];
                    auto const fp = element_footprint(ex.elements()[idx], opt);
                    auto const win = full_window(ctx.w, ctx.h, fp);
                    if(!win)
                    {
                        std::string msg = "auto_layout: bounds too small for element footprint";
                        ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                        return status{std::errc::invalid_argument, std::move(msg)};
                    }

                    int x{};
                    if(max_level == 0) { x = static_cast<int>(ctx.w / 2); }
                    else
                    {
                        double const t = static_cast<double>(l) / static_cast<double>(max_level);
                        x = static_cast<int>(std::llround(t * static_cast<double>(ctx.w - 1)));
                    }

                    double const ty = (static_cast<double>(rank) + 0.5) / static_cast<double>(k);
                    int const y = static_cast<int>(std::llround(ty * static_cast<double>(ctx.h - 1)));
                    cell ideal{x, y};

                    auto chosen =
                        choose_cell(ctx.occ, ideal, fp, ctx.adj[idx], ctx.placed, opt.max_candidates_per_element, opt.max_search_radius, 0.3, 2.0, win);
                    if(!chosen)
                    {
                        ++skipped_count;
                        continue;
                    }

                    ctx.occ.occupy(*chosen, fp, static_cast<int>(idx));
                    ctx.placed[idx] = *chosen;
                    ++placed_count;
                }
            }

            auto st = apply_placements_ec(ex, ctx.bounds, opt.step_x, opt.step_y, z_fixed, ctx.movable, ctx.placed);
            if(!st) { return st; }

            return stats{
                .layout_mode = opt.layout_mode,
                .grid_w = ctx.w,
                .grid_h = ctx.h,
                .step_x = opt.step_x,
                .step_y = opt.step_y,
                .fixed_obstacles = ctx.fixed_obstacles,
                .placed = placed_count,
                .skipped = skipped_count,
            };
        }

        struct edge_pair
        {
            std::size_t a{};
            std::size_t b{};
            double w{1.0};
        };

        struct anchor_edge
        {
            std::size_t a{};
            double x{};
            double y{};
            double w{1.0};
        };

        inline status_or<stats> layout_cpu_force_ec(experiment& ex, position corner0, position corner1, double z_fixed, options const& opt) noexcept
        {
            if(!std::isfinite(z_fixed))
            {
                std::string msg = "auto_layout: z_fixed must be finite";
                ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                return status{std::errc::invalid_argument, std::move(msg)};
            }

            auto ctx_r = build_context_ec(ex, corner0, corner1, opt);
            if(!ctx_r) { return ctx_r.st; }
            auto ctx = std::move(*ctx_r.value);
            if(ctx.movable.empty())
            {
                return stats{
                    .layout_mode = opt.layout_mode,
                    .grid_w = ctx.w,
                    .grid_h = ctx.h,
                    .step_x = opt.step_x,
                    .step_y = opt.step_y,
                    .fixed_obstacles = ctx.fixed_obstacles,
                    .placed = 0,
                    .skipped = 0,
                };
            }

            std::vector<int> to_sub(ex.elements().size(), -1);
            for(std::size_t i{}; i < ctx.movable.size(); ++i) { to_sub[ctx.movable[i]] = static_cast<int>(i); }

            // Build induced graph for movable nodes.
            weighted_adj g(ctx.movable.size());
            std::vector<edge_pair> edges;
            for(std::size_t si{}; si < ctx.movable.size(); ++si)
            {
                auto const oi = ctx.movable[si];
                for(auto const nb: ctx.adj[oi])
                {
                    auto const sj = to_sub[nb];
                    if(sj < 0) { continue; }
                    if(static_cast<std::size_t>(sj) == si) { continue; }
                    g[si].push_back(weighted_edge{static_cast<std::size_t>(sj), 1.0});
                    if(si < static_cast<std::size_t>(sj)) { edges.push_back(edge_pair{si, static_cast<std::size_t>(sj), 1.0}); }
                }
            }

            // Edges to fixed anchors (if fixed obstacles have known cells).
            std::vector<anchor_edge> anchors;
            anchors.reserve(ctx.movable.size());
            for(std::size_t si{}; si < ctx.movable.size(); ++si)
            {
                auto const oi = ctx.movable[si];
                for(auto const nb: ctx.adj[oi])
                {
                    if(to_sub[nb] >= 0) { continue; }
                    if(!ctx.placed[nb]) { continue; }
                    auto const c = *ctx.placed[nb];
                    double const ax = (ctx.w <= 1) ? 0.5 : (static_cast<double>(c.x) / static_cast<double>(ctx.w - 1));
                    double const ay = (ctx.h <= 1) ? 0.5 : (static_cast<double>(c.y) / static_cast<double>(ctx.h - 1));
                    anchors.push_back(anchor_edge{si, ax, ay, 1.0});
                }
            }

            // Initialize continuous coordinates from spectral embedding (stable).
            auto const emb = spectral_embedding(g, opt);
            std::vector<double> x = emb.x;
            std::vector<double> y = emb.y;

            auto const bins = std::max<std::size_t>(4, opt.force_bins);
            auto const iters = std::max<std::size_t>(1, opt.force_iterations);
            auto const k_attr = opt.force_attraction;
            auto const k_rep = opt.force_repulsion;

            std::vector<double> fx(ctx.movable.size(), 0.0);
            std::vector<double> fy(ctx.movable.size(), 0.0);
            std::vector<std::vector<std::size_t>> buckets(bins * bins);

            auto clamp01 = [](double v) -> double { return std::min(1.0, std::max(0.0, v)); };
            auto bin_idx = [&](double px, double py) -> std::size_t
            {
                auto bx = static_cast<int>(std::floor(clamp01(px) * static_cast<double>(bins)));
                auto by = static_cast<int>(std::floor(clamp01(py) * static_cast<double>(bins)));
                bx = std::clamp(bx, 0, static_cast<int>(bins - 1));
                by = std::clamp(by, 0, static_cast<int>(bins - 1));
                return static_cast<std::size_t>(by) * bins + static_cast<std::size_t>(bx);
            };

            for(std::size_t it{}; it < iters; ++it)
            {
                for(auto& b: buckets) { b.clear(); }
                for(std::size_t i{}; i < ctx.movable.size(); ++i) { buckets[bin_idx(x[i], y[i])].push_back(i); }

                std::fill(fx.begin(), fx.end(), 0.0);
                std::fill(fy.begin(), fy.end(), 0.0);

                // Repulsion (local, approximate).
                for(std::size_t by{}; by < bins; ++by)
                {
                    for(std::size_t bx{}; bx < bins; ++bx)
                    {
                        auto const b0 = by * bins + bx;
                        auto const& v0 = buckets[b0];
                        for(auto i: v0)
                        {
                            auto const i_bx = static_cast<int>(bx);
                            auto const i_by = static_cast<int>(by);
                            for(int dy = -1; dy <= 1; ++dy)
                            {
                                for(int dx = -1; dx <= 1; ++dx)
                                {
                                    auto const nbx = i_bx + dx;
                                    auto const nby = i_by + dy;
                                    if(nbx < 0 || nby < 0 || nbx >= static_cast<int>(bins) || nby >= static_cast<int>(bins)) { continue; }
                                    auto const& v1 = buckets[static_cast<std::size_t>(nby) * bins + static_cast<std::size_t>(nbx)];
                                    for(auto j: v1)
                                    {
                                        if(j <= i) { continue; }
                                        double dxp = x[i] - x[j];
                                        double dyp = y[i] - y[j];
                                        double const d2 = dxp * dxp + dyp * dyp + 1e-6;
                                        double const inv = k_rep / d2;
                                        fx[i] += dxp * inv;
                                        fy[i] += dyp * inv;
                                        fx[j] -= dxp * inv;
                                        fy[j] -= dyp * inv;
                                    }
                                }
                            }
                        }
                    }
                }

                // Attraction (edges).
                for(auto const& e: edges)
                {
                    auto const a = e.a;
                    auto const b = e.b;
                    double const dxp = x[b] - x[a];
                    double const dyp = y[b] - y[a];
                    fx[a] += dxp * (k_attr * e.w);
                    fy[a] += dyp * (k_attr * e.w);
                    fx[b] -= dxp * (k_attr * e.w);
                    fy[b] -= dyp * (k_attr * e.w);
                }

                // Attraction to fixed anchors.
                for(auto const& a: anchors)
                {
                    double const dxp = a.x - x[a.a];
                    double const dyp = a.y - y[a.a];
                    fx[a.a] += dxp * (k_attr * a.w);
                    fy[a.a] += dyp * (k_attr * a.w);
                }

                // Integrate with annealing-like step.
                double const t = 1.0 - (static_cast<double>(it) / static_cast<double>(iters));
                double const step = 0.2 * t;
                for(std::size_t i{}; i < ctx.movable.size(); ++i)
                {
                    x[i] = clamp01(x[i] + std::clamp(fx[i], -step, step));
                    y[i] = clamp01(y[i] + std::clamp(fy[i], -step, step));
                }
            }

            // Snap to grid with collision-free discrete assignment.
            auto degree = [&](std::size_t idx) -> std::size_t { return ctx.adj[idx].size(); };
            std::stable_sort(ctx.movable.begin(),
                             ctx.movable.end(),
                             [&](std::size_t a, std::size_t b)
                             {
                                 auto const& ea = ex.elements()[a];
                                 auto const& eb = ex.elements()[b];
                                 if(ea.is_big_element() != eb.is_big_element()) { return ea.is_big_element() > eb.is_big_element(); }
                                 auto const da = degree(a);
                                 auto const db = degree(b);
                                 if(da != db) { return da > db; }
                                 return ea.identifier() < eb.identifier();
                             });

            std::size_t placed_count{};
            std::size_t skipped_count{};

            for(auto idx: ctx.movable)
            {
                auto const fp = element_footprint(ex.elements()[idx], opt);
                auto const win = full_window(ctx.w, ctx.h, fp);
                if(!win)
                {
                    std::string msg = "auto_layout: bounds too small for element footprint";
                    ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                    return status{std::errc::invalid_argument, std::move(msg)};
                }

                auto const si = to_sub[idx];
                auto const u = (si >= 0) ? x[static_cast<std::size_t>(si)] : 0.5;
                auto const v = (si >= 0) ? y[static_cast<std::size_t>(si)] : 0.5;

                cell ideal{
                    static_cast<int>(std::llround(u * static_cast<double>(ctx.w - 1))),
                    static_cast<int>(std::llround(v * static_cast<double>(ctx.h - 1))),
                };

                auto chosen = choose_cell(ctx.occ, ideal, fp, ctx.adj[idx], ctx.placed, opt.max_candidates_per_element, opt.max_search_radius, 0.35, 1.2, win);
                if(!chosen)
                {
                    ++skipped_count;
                    continue;
                }

                ctx.occ.occupy(*chosen, fp, static_cast<int>(idx));
                ctx.placed[idx] = *chosen;
                ++placed_count;
            }

            auto st = apply_placements_ec(ex, ctx.bounds, opt.step_x, opt.step_y, z_fixed, ctx.movable, ctx.placed);
            if(!st) { return st; }

            return stats{
                .layout_mode = opt.layout_mode,
                .grid_w = ctx.w,
                .grid_h = ctx.h,
                .step_x = opt.step_x,
                .step_y = opt.step_y,
                .fixed_obstacles = ctx.fixed_obstacles,
                .placed = placed_count,
                .skipped = skipped_count,
            };
        }

        struct cluster_block
        {
            std::vector<std::size_t> nodes{};  // movable-sub indices
            footprint inner{};
            footprint outer{};
            cell outer_origin{};
        };

        inline status_or<stats> layout_cpu_cluster_ec(experiment& ex, position corner0, position corner1, double z_fixed, options const& opt) noexcept
        {
            if(!std::isfinite(z_fixed))
            {
                std::string msg = "auto_layout: z_fixed must be finite";
                ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                return status{std::errc::invalid_argument, std::move(msg)};
            }

            auto ctx_r = build_context_ec(ex, corner0, corner1, opt);
            if(!ctx_r) { return ctx_r.st; }
            auto ctx = std::move(*ctx_r.value);
            if(ctx.movable.empty())
            {
                return stats{
                    .layout_mode = opt.layout_mode,
                    .grid_w = ctx.w,
                    .grid_h = ctx.h,
                    .step_x = opt.step_x,
                    .step_y = opt.step_y,
                    .fixed_obstacles = ctx.fixed_obstacles,
                    .placed = 0,
                    .skipped = 0,
                };
            }

            auto const m = ctx.movable.size();
            std::vector<int> to_sub(ex.elements().size(), -1);
            for(std::size_t i{}; i < m; ++i) { to_sub[ctx.movable[i]] = static_cast<int>(i); }

            std::vector<std::vector<std::size_t>> sub_adj(m);
            std::vector<std::size_t> sub_degree(m, 0);
            for(std::size_t si{}; si < m; ++si)
            {
                auto const oi = ctx.movable[si];
                for(auto const nb: ctx.adj[oi])
                {
                    auto const sj = to_sub[nb];
                    if(sj < 0) { continue; }
                    if(static_cast<std::size_t>(sj) == si) { continue; }
                    sub_adj[si].push_back(static_cast<std::size_t>(sj));
                }
                std::sort(sub_adj[si].begin(), sub_adj[si].end());
                sub_adj[si].erase(std::unique(sub_adj[si].begin(), sub_adj[si].end()), sub_adj[si].end());
                sub_degree[si] = sub_adj[si].size();
            }

            // Label propagation clustering (fast, deterministic).
            std::vector<std::size_t> label(m);
            for(std::size_t i{}; i < m; ++i) { label[i] = i; }

            std::vector<std::size_t> order(m);
            std::iota(order.begin(), order.end(), 0);
            std::stable_sort(order.begin(),
                             order.end(),
                             [&](std::size_t a, std::size_t b)
                             {
                                 if(sub_degree[a] != sub_degree[b]) { return sub_degree[a] > sub_degree[b]; }
                                 return a < b;
                             });

            for(std::size_t it{}; it < opt.cluster_label_iterations; ++it)
            {
                bool changed = false;
                for(auto si: order)
                {
                    auto const& nbs = sub_adj[si];
                    if(nbs.empty()) { continue; }
                    std::vector<std::size_t> labs;
                    labs.reserve(nbs.size());
                    for(auto nb: nbs) { labs.push_back(label[nb]); }
                    std::sort(labs.begin(), labs.end());

                    std::size_t best = labs.front();
                    std::size_t best_cnt = 1;
                    std::size_t cur = labs.front();
                    std::size_t cur_cnt = 1;
                    for(std::size_t i = 1; i < labs.size(); ++i)
                    {
                        if(labs[i] == cur)
                        {
                            ++cur_cnt;
                            continue;
                        }
                        if(cur_cnt > best_cnt || (cur_cnt == best_cnt && cur < best))
                        {
                            best = cur;
                            best_cnt = cur_cnt;
                        }
                        cur = labs[i];
                        cur_cnt = 1;
                    }
                    if(cur_cnt > best_cnt || (cur_cnt == best_cnt && cur < best)) { best = cur; }

                    if(best != label[si])
                    {
                        label[si] = best;
                        changed = true;
                    }
                }
                if(!changed) { break; }
            }

            // Compress labels to contiguous cluster ids.
            std::unordered_map<std::size_t, std::size_t> label_to_cluster;
            label_to_cluster.reserve(m);
            std::vector<std::size_t> cluster_of(m, 0);
            std::size_t cluster_count{};
            for(std::size_t si{}; si < m; ++si)
            {
                auto const l = label[si];
                auto [it, inserted] = label_to_cluster.emplace(l, cluster_count);
                if(inserted) { ++cluster_count; }
                cluster_of[si] = it->second;
            }

            std::vector<std::vector<std::size_t>> clusters(cluster_count);
            for(std::size_t si{}; si < m; ++si) { clusters[cluster_of[si]].push_back(si); }

            // Split overly large clusters into manageable blocks.
            if(opt.cluster_max_nodes > 0)
            {
                std::vector<std::vector<std::size_t>> split;
                split.reserve(clusters.size());

                std::vector<char> in_set(m, 0);
                std::vector<char> visited(m, 0);
                std::queue<std::size_t> q;

                for(auto const& c: clusters)
                {
                    if(c.size() <= opt.cluster_max_nodes)
                    {
                        split.push_back(c);
                        continue;
                    }

                    std::fill(in_set.begin(), in_set.end(), 0);
                    std::fill(visited.begin(), visited.end(), 0);
                    for(auto si: c) { in_set[si] = 1; }

                    std::vector<std::size_t> bfs_order;
                    bfs_order.reserve(c.size());

                    for(auto seed: c)
                    {
                        if(visited[seed]) { continue; }
                        visited[seed] = 1;
                        q.push(seed);
                        while(!q.empty())
                        {
                            auto v = q.front();
                            q.pop();
                            bfs_order.push_back(v);
                            for(auto nb: sub_adj[v])
                            {
                                if(!in_set[nb] || visited[nb]) { continue; }
                                visited[nb] = 1;
                                q.push(nb);
                            }
                        }
                    }

                    for(std::size_t i{}; i < bfs_order.size(); i += opt.cluster_max_nodes)
                    {
                        auto const end = std::min(bfs_order.size(), i + opt.cluster_max_nodes);
                        split.emplace_back(bfs_order.begin() + static_cast<std::ptrdiff_t>(i), bfs_order.begin() + static_cast<std::ptrdiff_t>(end));
                    }
                }
                clusters.swap(split);
                cluster_count = clusters.size();

                // Rebuild cluster_of (sub index -> cluster id).
                std::fill(cluster_of.begin(), cluster_of.end(), 0);
                for(std::size_t cid{}; cid < clusters.size(); ++cid)
                {
                    for(auto si: clusters[cid]) { cluster_of[si] = cid; }
                }
            }

            // Build macro blocks.
            std::vector<cluster_block> blocks(cluster_count);
            auto const spacing = opt.cluster_channel_spacing;
            for(std::size_t cid{}; cid < cluster_count; ++cid)
            {
                blocks[cid].nodes = clusters[cid];

                std::size_t area{};
                std::size_t max_w{};
                std::size_t max_h{};
                for(auto si: clusters[cid])
                {
                    auto const oi = ctx.movable[si];
                    auto const fp = element_footprint(ex.elements()[oi], opt);
                    area += fp.w * fp.h;
                    max_w = std::max(max_w, fp.w);
                    max_h = std::max(max_h, fp.h);
                }

                double const fill = 0.70;
                auto const target_area = static_cast<std::size_t>(std::ceil(static_cast<double>(std::max<std::size_t>(1, area)) / fill));
                auto inner_w = static_cast<std::size_t>(std::ceil(std::sqrt(static_cast<double>(target_area))));
                inner_w = std::max(inner_w, max_w);
                auto inner_h = static_cast<std::size_t>(std::ceil(static_cast<double>(target_area) / static_cast<double>(inner_w)));
                inner_h = std::max(inner_h, max_h);

                blocks[cid].inner = footprint{inner_w, inner_h};
                blocks[cid].outer = footprint{inner_w + 2 * spacing, inner_h + 2 * spacing};
                blocks[cid].outer_origin = cell{0, 0};
            }

            // Build weighted cluster adjacency graph.
            std::unordered_map<std::uint64_t, double> pair_w;
            pair_w.reserve(m * 2);

            auto key_pair = [](std::size_t a, std::size_t b) -> std::uint64_t
            {
                if(a > b) { std::swap(a, b); }
                return (static_cast<std::uint64_t>(a) << 32) | static_cast<std::uint64_t>(b);
            };

            for(std::size_t si{}; si < m; ++si)
            {
                auto const ci = cluster_of[si];
                for(auto sj: sub_adj[si])
                {
                    if(sj <= si) { continue; }
                    auto const cj = cluster_of[sj];
                    if(ci == cj) { continue; }
                    pair_w[key_pair(ci, cj)] += 1.0;
                }
            }

            weighted_adj cg(cluster_count);
            cg.reserve(cluster_count);
            for(auto const& kv: pair_w)
            {
                auto const a = static_cast<std::size_t>(kv.first >> 32);
                auto const b = static_cast<std::size_t>(kv.first & 0xFFFF'FFFFu);
                auto const w = kv.second;
                if(a >= cluster_count || b >= cluster_count) { continue; }
                cg[a].push_back(weighted_edge{b, w});
                cg[b].push_back(weighted_edge{a, w});
            }

            auto const cluster_emb = spectral_embedding(cg, opt);

            // Place macro blocks first (chip-like floorplan).
            occupancy occ_macro = ctx.occ;
            std::vector<std::optional<cell>> placed_cluster(cluster_count, std::nullopt);

            std::vector<std::size_t> cluster_order(cluster_count);
            std::iota(cluster_order.begin(), cluster_order.end(), 0);

            auto const cluster_deg = degrees(cg);
            std::stable_sort(cluster_order.begin(),
                             cluster_order.end(),
                             [&](std::size_t a, std::size_t b)
                             {
                                 auto const aa = blocks[a].outer.w * blocks[a].outer.h;
                                 auto const ab = blocks[b].outer.w * blocks[b].outer.h;
                                 if(aa != ab) { return aa > ab; }
                                 if(cluster_deg[a] != cluster_deg[b]) { return cluster_deg[a] > cluster_deg[b]; }
                                 return a < b;
                             });

            auto choose_macro = [&](std::size_t cid) -> std::optional<cell>
            {
                auto const fp = blocks[cid].outer;
                if(ctx.w < fp.w || ctx.h < fp.h) { return std::nullopt; }

                window const win{
                    .min_x = 0,
                    .min_y = 0,
                    .max_x = static_cast<int>(ctx.w - fp.w),
                    .max_y = static_cast<int>(ctx.h - fp.h),
                };

                auto const u = (cid < cluster_emb.x.size()) ? cluster_emb.x[cid] : 0.5;
                auto const v = (cid < cluster_emb.y.size()) ? cluster_emb.y[cid] : 0.5;
                cell ideal{
                    static_cast<int>(std::llround(u * static_cast<double>(ctx.w - 1))),
                    static_cast<int>(std::llround(v * static_cast<double>(ctx.h - 1))),
                };
                ideal.x = std::clamp(ideal.x, win.min_x, win.max_x);
                ideal.y = std::clamp(ideal.y, win.min_y, win.max_y);

                auto const max_radius = [&]() -> std::size_t
                {
                    if(opt.max_search_radius) { return *opt.max_search_radius; }
                    return static_cast<std::size_t>(std::max<std::size_t>(ctx.w, ctx.h));
                }();

                std::optional<cell> best{};
                double best_cost = std::numeric_limits<double>::infinity();
                std::size_t visited{};
                auto const max_candidates = std::min<std::size_t>(opt.max_candidates_per_element * 4, 65536);

                auto center = [](cell c, footprint f) -> std::pair<double, double>
                { return {static_cast<double>(c.x) + 0.5 * static_cast<double>(f.w), static_cast<double>(c.y) + 0.5 * static_cast<double>(f.h)}; };

                auto consider = [&](cell c)
                {
                    if(visited >= max_candidates) { return; }
                    ++visited;
                    if(c.x < win.min_x || c.y < win.min_y || c.x > win.max_x || c.y > win.max_y) { return; }
                    if(!occ_macro.can_place(c, fp)) { return; }

                    auto const [cx, cy] = center(c, fp);
                    double cost = 0.0;
                    cost += opt.cluster_macro_ideal_weight * static_cast<double>(std::abs(c.x - ideal.x) + std::abs(c.y - ideal.y));

                    for(auto const& e: cg[cid])
                    {
                        auto const other = e.to;
                        if(!placed_cluster[other]) { continue; }
                        auto const [ox, oy] = center(*placed_cluster[other], blocks[other].outer);
                        cost += opt.cluster_macro_neighbor_weight * e.w * (std::abs(cx - ox) + std::abs(cy - oy));
                    }

                    if(!best || cost < best_cost)
                    {
                        best = c;
                        best_cost = cost;
                    }
                };

                for(std::size_t r{}; r <= max_radius && visited < max_candidates; ++r)
                {
                    auto const ir = static_cast<int>(r);
                    if(r == 0)
                    {
                        consider(ideal);
                        continue;
                    }
                    for(int dx = -ir; dx <= ir && visited < max_candidates; ++dx)
                    {
                        cell c1{ideal.x + dx, ideal.y - ir};
                        cell c2{ideal.x + dx, ideal.y + ir};
                        consider(c1);
                        consider(c2);
                    }
                    for(int dy = -ir + 1; dy <= ir - 1 && visited < max_candidates; ++dy)
                    {
                        cell c1{ideal.x - ir, ideal.y + dy};
                        cell c2{ideal.x + ir, ideal.y + dy};
                        consider(c1);
                        consider(c2);
                    }
                }
                return best;
            };

            for(auto cid: cluster_order)
            {
                auto chosen = choose_macro(cid);
                if(!chosen)
                {
                    // Fallback: if macro placement fails, degrade to fast global placement.
                    options opt2 = opt;
                    opt2.layout_mode = mode::fast;
                    return layout_cpu_fast_ec(ex, corner0, corner1, z_fixed, opt2);
                }
                occ_macro.occupy(*chosen, blocks[cid].outer, static_cast<int>(cid));
                placed_cluster[cid] = *chosen;
                blocks[cid].outer_origin = *chosen;
            }

            // Place elements within each macro block.
            occupancy occ_elem = ctx.occ;
            std::size_t placed_count{};
            std::size_t skipped_count{};

            for(auto cid: cluster_order)
            {
                auto const origin = blocks[cid].outer_origin;
                cell inner_origin{origin.x + static_cast<int>(spacing), origin.y + static_cast<int>(spacing)};
                auto const inner = blocks[cid].inner;

                auto nodes = blocks[cid].nodes;
                std::stable_sort(nodes.begin(),
                                 nodes.end(),
                                 [&](std::size_t a, std::size_t b)
                                 {
                                     auto const oa = ctx.movable[a];
                                     auto const ob = ctx.movable[b];
                                     auto const& ea = ex.elements()[oa];
                                     auto const& eb = ex.elements()[ob];
                                     if(ea.is_big_element() != eb.is_big_element()) { return ea.is_big_element() > eb.is_big_element(); }
                                     if(sub_degree[a] != sub_degree[b]) { return sub_degree[a] > sub_degree[b]; }
                                     return ea.identifier() < eb.identifier();
                                 });

                cell const cluster_center{
                    inner_origin.x + static_cast<int>(inner.w / 2),
                    inner_origin.y + static_cast<int>(inner.h / 2),
                };

                for(auto si: nodes)
                {
                    auto const idx = ctx.movable[si];
                    auto const fp = element_footprint(ex.elements()[idx], opt);
                    if(inner.w < fp.w || inner.h < fp.h)
                    {
                        ++skipped_count;
                        continue;
                    }
                    window win{
                        .min_x = inner_origin.x,
                        .min_y = inner_origin.y,
                        .max_x = inner_origin.x + static_cast<int>(inner.w - fp.w),
                        .max_y = inner_origin.y + static_cast<int>(inner.h - fp.h),
                    };

                    cell ideal = cluster_center;
                    {
                        std::int64_t sum_x{};
                        std::int64_t sum_y{};
                        std::size_t cnt{};
                        for(auto nb: ctx.adj[idx])
                        {
                            if(!ctx.placed[nb]) { continue; }
                            sum_x += ctx.placed[nb]->x;
                            sum_y += ctx.placed[nb]->y;
                            ++cnt;
                        }
                        if(cnt != 0)
                        {
                            ideal.x = static_cast<int>(std::llround(static_cast<double>(sum_x) / static_cast<double>(cnt)));
                            ideal.y = static_cast<int>(std::llround(static_cast<double>(sum_y) / static_cast<double>(cnt)));
                        }
                    }

                    ideal.x = std::clamp(ideal.x, win.min_x, win.max_x);
                    ideal.y = std::clamp(ideal.y, win.min_y, win.max_y);

                    auto chosen =
                        choose_cell(occ_elem, ideal, fp, ctx.adj[idx], ctx.placed, opt.max_candidates_per_element, opt.max_search_radius, 1.0, 0.2, win);
                    if(!chosen)
                    {
                        ++skipped_count;
                        continue;
                    }

                    occ_elem.occupy(*chosen, fp, static_cast<int>(idx));
                    ctx.placed[idx] = *chosen;
                    ++placed_count;
                }
            }

            auto st = apply_placements_ec(ex, ctx.bounds, opt.step_x, opt.step_y, z_fixed, ctx.movable, ctx.placed);
            if(!st) { return st; }

            return stats{
                .layout_mode = opt.layout_mode,
                .grid_w = ctx.w,
                .grid_h = ctx.h,
                .step_x = opt.step_x,
                .step_y = opt.step_y,
                .fixed_obstacles = ctx.fixed_obstacles,
                .placed = placed_count,
                .skipped = skipped_count,
            };
        }

        inline status_or<stats> layout_cpu_packing_ec(experiment& ex, position corner0, position corner1, double z_fixed, options const& opt) noexcept
        {
            if(!std::isfinite(z_fixed))
            {
                std::string msg = "auto_layout: z_fixed must be finite";
                ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                return status{std::errc::invalid_argument, std::move(msg)};
            }

            auto ctx_r = build_context_ec(ex, corner0, corner1, opt);
            if(!ctx_r) { return ctx_r.st; }
            auto ctx = std::move(*ctx_r.value);

            auto degree = [&](std::size_t idx) -> std::size_t { return ctx.adj[idx].size(); };
            std::stable_sort(ctx.movable.begin(),
                             ctx.movable.end(),
                             [&](std::size_t a, std::size_t b)
                             {
                                 auto const& ea = ex.elements()[a];
                                 auto const& eb = ex.elements()[b];
                                 if(ea.is_big_element() != eb.is_big_element()) { return ea.is_big_element() > eb.is_big_element(); }
                                 auto const da = degree(a);
                                 auto const db = degree(b);
                                 if(da != db) { return da > db; }
                                 return ea.identifier() < eb.identifier();
                             });

            std::size_t placed_count{};
            std::size_t skipped_count{};

            for(auto idx: ctx.movable)
            {
                auto const fp = element_footprint(ex.elements()[idx], opt);
                auto const win = full_window(ctx.w, ctx.h, fp);
                if(!win)
                {
                    std::string msg = "auto_layout: bounds too small for element footprint";
                    ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                    return status{std::errc::invalid_argument, std::move(msg)};
                }

                std::optional<cell> chosen{};
                for(int y = win->min_y; y <= win->max_y && !chosen; ++y)
                {
                    for(int x = win->min_x; x <= win->max_x; ++x)
                    {
                        cell c{x, y};
                        if(!ctx.occ.can_place(c, fp)) { continue; }
                        chosen = c;
                        break;
                    }
                }

                if(!chosen)
                {
                    ++skipped_count;
                    continue;
                }

                ctx.occ.occupy(*chosen, fp, static_cast<int>(idx));
                ctx.placed[idx] = *chosen;
                ++placed_count;
            }

            auto st = apply_placements_ec(ex, ctx.bounds, opt.step_x, opt.step_y, z_fixed, ctx.movable, ctx.placed);
            if(!st) { return st; }

            return stats{
                .layout_mode = opt.layout_mode,
                .grid_w = ctx.w,
                .grid_h = ctx.h,
                .step_x = opt.step_x,
                .step_y = opt.step_y,
                .fixed_obstacles = ctx.fixed_obstacles,
                .placed = placed_count,
                .skipped = skipped_count,
            };
        }

        struct fixed_anchor
        {
            std::size_t idx{};
            cell c{};
            footprint fp{};
            double z_native{};
        };

        inline status_or<std::vector<fixed_anchor>> collect_fixed_anchors_ec(experiment const& ex, context& ctx, options const& opt) noexcept
        {
            std::vector<fixed_anchor> out{};
            out.reserve(ex.elements().size());

            for(std::size_t i{}; i < ex.elements().size(); ++i)
            {
                auto const& e = ex.elements()[i];
                if(e.participate_in_layout()) { continue; }

                auto const native_pos =
                    e.is_element_xyz() ? element_xyz::to_native(e.element_position(), ex.element_xyz_origin(), e.is_big_element()) : e.element_position();
                auto c = snap_native_to_cell(ctx.bounds, opt.step_x, opt.step_y, native_pos);
                // Fixed elements outside the layout bounds should not act as anchors.
                if(c.x < 0 || c.y < 0) { continue; }

                auto const fp = element_footprint(e, opt);
                if(ctx.w < fp.w || ctx.h < fp.h)
                {
                    std::string msg = "auto_layout: bounds too small for element footprint";
                    ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                    return status{std::errc::invalid_argument, std::move(msg)};
                }
                c.x = std::clamp(c.x, 0, static_cast<int>(ctx.w - fp.w));
                c.y = std::clamp(c.y, 0, static_cast<int>(ctx.h - fp.h));

                ctx.placed[i] = c;  // used as an anchor in placement_cost()
                out.push_back(fixed_anchor{
                    .idx = i,
                    .c = c,
                    .fp = fp,
                    .z_native = native_pos.z,
                });
            }
            return out;
        }

        inline bool same_z_plane(double a, double b) noexcept
        {
            // Allow a small tolerance so values that were previously snapped still match.
            return std::fabs(a - b) <= (z_step_3d * 0.5 + 1e-12);
        }

        inline status_or<stats> layout_cpu_b_3d_ec(experiment& ex, position corner0, position corner1, double z_base, options const& opt) noexcept
        {
            if(!std::isfinite(z_base))
            {
                std::string msg = "auto_layout: z_base must be finite";
                ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                return status{std::errc::invalid_argument, std::move(msg)};
            }

            // In 3D, different Z planes are allowed to overlap in (x,y), so we handle per-layer occupancy ourselves.
            auto opt_ctx = opt;
            opt_ctx.respect_fixed_elements = false;
            auto ctx_r = build_context_ec(ex, corner0, corner1, opt_ctx);
            if(!ctx_r) { return ctx_r.st; }
            auto ctx = std::move(*ctx_r.value);

            auto fixed_r = collect_fixed_anchors_ec(ex, ctx, opt);
            if(!fixed_r) { return fixed_r.st; }
            auto fixed = std::move(*fixed_r.value);

            if(ctx.movable.empty())
            {
                return stats{
                    .layout_mode = mode::hierarchical,
                    .grid_w = ctx.w,
                    .grid_h = ctx.h,
                    .step_x = opt.step_x,
                    .step_y = opt.step_y,
                    .fixed_obstacles = 0,
                    .placed = 0,
                    .skipped = 0,
                };
            }

            auto const n = ex.elements().size();
            std::vector<int> level(n, -1);
            std::queue<std::size_t> q;

            for(std::size_t i{}; i < n; ++i)
            {
                auto const mid = ex.elements()[i].data().value("ModelID", "");
                if(is_input_like(mid))
                {
                    level[i] = 0;
                    q.push(i);
                }
            }

            if(q.empty())
            {
                // Fallback: use highest-degree movable node as source.
                auto best = ctx.movable.front();
                for(auto idx: ctx.movable)
                {
                    if(ctx.adj[idx].size() > ctx.adj[best].size()) { best = idx; }
                }
                level[best] = 0;
                q.push(best);
            }

            while(!q.empty())
            {
                auto const v = q.front();
                q.pop();
                auto const lv = level[v];
                for(auto nb: ctx.adj[v])
                {
                    if(level[nb] != -1) { continue; }
                    level[nb] = lv + 1;
                    q.push(nb);
                }
            }

            int max_level{};
            for(auto idx: ctx.movable)
            {
                auto const lv = (level[idx] < 0) ? 0 : level[idx];
                level[idx] = lv;
                if(lv > max_level) { max_level = lv; }
            }

            auto const L = static_cast<std::size_t>(max_level);
            std::vector<std::vector<std::size_t>> layers(L + 1);
            for(auto idx: ctx.movable) { layers[static_cast<std::size_t>(level[idx])].push_back(idx); }

            auto degree = [&](std::size_t idx) -> std::size_t { return ctx.adj[idx].size(); };
            for(auto& layer_nodes: layers)
            {
                std::stable_sort(layer_nodes.begin(),
                                 layer_nodes.end(),
                                 [&](std::size_t a, std::size_t b)
                                 {
                                     auto const& ea = ex.elements()[a];
                                     auto const& eb = ex.elements()[b];
                                     if(ea.is_big_element() != eb.is_big_element()) { return ea.is_big_element() > eb.is_big_element(); }
                                     auto const da = degree(a);
                                     auto const db = degree(b);
                                     if(da != db) { return da > db; }
                                     return ea.identifier() < eb.identifier();
                                 });
            }

            std::size_t fixed_obstacles{};
            std::size_t placed_count{};
            std::size_t skipped_count{};

            for(std::size_t l{}; l < layers.size(); ++l)
            {
                auto const z_layer = z_base + static_cast<double>(l) * z_step_3d;

                occupancy occ_layer(ctx.w, ctx.h);
                if(opt.respect_fixed_elements)
                {
                    for(auto const& f: fixed)
                    {
                        if(!same_z_plane(f.z_native, z_layer)) { continue; }
                        if(!occ_layer.can_place(f.c, f.fp)) { continue; }
                        occ_layer.occupy(f.c, f.fp, static_cast<int>(f.idx));
                        ++fixed_obstacles;
                    }
                }

                auto const& nodes = layers[l];
                if(nodes.empty()) { continue; }

                auto const k = nodes.size();
                for(std::size_t rank{}; rank < k; ++rank)
                {
                    auto const idx = nodes[rank];
                    auto const fp = element_footprint(ex.elements()[idx], opt);
                    auto const win = full_window(ctx.w, ctx.h, fp);
                    if(!win)
                    {
                        std::string msg = "auto_layout: bounds too small for element footprint";
                        ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                        return status{std::errc::invalid_argument, std::move(msg)};
                    }

                    int const rank_y = static_cast<int>(std::llround(((static_cast<double>(rank) + 0.5) / static_cast<double>(k)) * static_cast<double>(ctx.h - 1)));

                    std::int64_t sum_x{};
                    std::int64_t sum_y{};
                    std::size_t cnt{};
                    for(auto nb: ctx.adj[idx])
                    {
                        if(!ctx.placed[nb]) { continue; }
                        sum_x += ctx.placed[nb]->x;
                        sum_y += ctx.placed[nb]->y;
                        ++cnt;
                    }

                    int ideal_x{};
                    int ideal_y{};

                    if(cnt != 0)
                    {
                        ideal_x = static_cast<int>(std::llround(static_cast<double>(sum_x) / static_cast<double>(cnt)));
                        double const avg_y = static_cast<double>(sum_y) / static_cast<double>(cnt);
                        ideal_y = static_cast<int>(std::llround(0.5 * avg_y + 0.5 * static_cast<double>(rank_y)));
                    }
                    else
                    {
                        if(max_level == 0) { ideal_x = static_cast<int>(ctx.w / 2); }
                        else
                        {
                            double const t = static_cast<double>(l) / static_cast<double>(max_level);
                            ideal_x = static_cast<int>(std::llround(t * static_cast<double>(ctx.w - 1)));
                        }
                        ideal_y = rank_y;
                    }

                    cell ideal{ideal_x, ideal_y};
                    ideal.x = std::clamp(ideal.x, win->min_x, win->max_x);
                    ideal.y = std::clamp(ideal.y, win->min_y, win->max_y);

                    auto chosen =
                        choose_cell(occ_layer, ideal, fp, ctx.adj[idx], ctx.placed, opt.max_candidates_per_element, opt.max_search_radius, 0.3, 2.0, win);
                    if(!chosen)
                    {
                        ++skipped_count;
                        continue;
                    }

                    occ_layer.occupy(*chosen, fp, static_cast<int>(idx));
                    ctx.placed[idx] = *chosen;
                    ++placed_count;

                    auto const native_pos = cell_to_native(ctx.bounds, opt.step_x, opt.step_y, *chosen, z_layer);
                    auto id_r = ex.elements()[idx].identifier_ec();
                    if(!id_r) { return id_r.st; }
                    auto* el = ex.find_element(*id_r.value);
                    if(el == nullptr)
                    {
                        auto msg = "auto_layout: unknown element identifier: " + *id_r.value;
                        ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                        return status{std::errc::invalid_argument, std::move(msg)};
                    }
                    set_element_native_position(ex, *el, native_pos);
                }
            }

            return stats{
                .layout_mode = mode::hierarchical,
                .grid_w = ctx.w,
                .grid_h = ctx.h,
                .step_x = opt.step_x,
                .step_y = opt.step_y,
                .fixed_obstacles = fixed_obstacles,
                .placed = placed_count,
                .skipped = skipped_count,
            };
        }

        inline status_or<stats> layout_cpu_c_3d_ec(experiment& ex, position corner0, position corner1, double z_base, options const& opt) noexcept
        {
            if(!std::isfinite(z_base))
            {
                std::string msg = "auto_layout: z_base must be finite";
                ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                return status{std::errc::invalid_argument, std::move(msg)};
            }

            auto opt_ctx = opt;
            opt_ctx.respect_fixed_elements = false;
            auto ctx_r = build_context_ec(ex, corner0, corner1, opt_ctx);
            if(!ctx_r) { return ctx_r.st; }
            auto ctx = std::move(*ctx_r.value);

            auto fixed_r = collect_fixed_anchors_ec(ex, ctx, opt);
            if(!fixed_r) { return fixed_r.st; }
            auto fixed = std::move(*fixed_r.value);

            if(ctx.movable.empty())
            {
                return stats{
                    .layout_mode = mode::force,
                    .grid_w = ctx.w,
                    .grid_h = ctx.h,
                    .step_x = opt.step_x,
                    .step_y = opt.step_y,
                    .fixed_obstacles = 0,
                    .placed = 0,
                    .skipped = 0,
                };
            }

            std::vector<int> to_sub(ex.elements().size(), -1);
            for(std::size_t i{}; i < ctx.movable.size(); ++i) { to_sub[ctx.movable[i]] = static_cast<int>(i); }

            // Build induced graph for movable nodes.
            weighted_adj g(ctx.movable.size());
            std::vector<edge_pair> edges;
            for(std::size_t si{}; si < ctx.movable.size(); ++si)
            {
                auto const oi = ctx.movable[si];
                for(auto const nb: ctx.adj[oi])
                {
                    auto const sj = to_sub[nb];
                    if(sj < 0) { continue; }
                    if(static_cast<std::size_t>(sj) == si) { continue; }
                    g[si].push_back(weighted_edge{static_cast<std::size_t>(sj), 1.0});
                    if(si < static_cast<std::size_t>(sj)) { edges.push_back(edge_pair{si, static_cast<std::size_t>(sj), 1.0}); }
                }
            }

            // Edges to fixed anchors (only attract in x/y).
            std::vector<anchor_edge> anchors;
            anchors.reserve(ctx.movable.size());
            for(std::size_t si{}; si < ctx.movable.size(); ++si)
            {
                auto const oi = ctx.movable[si];
                for(auto const nb: ctx.adj[oi])
                {
                    if(to_sub[nb] >= 0) { continue; }
                    if(!ctx.placed[nb]) { continue; }
                    auto const c = *ctx.placed[nb];
                    double const ax = (ctx.w <= 1) ? 0.5 : (static_cast<double>(c.x) / static_cast<double>(ctx.w - 1));
                    double const ay = (ctx.h <= 1) ? 0.5 : (static_cast<double>(c.y) / static_cast<double>(ctx.h - 1));
                    anchors.push_back(anchor_edge{si, ax, ay, 1.0});
                }
            }

            // Initialize continuous coordinates from spectral embedding.
            auto const emb = spectral_embedding3d(g, opt);
            std::vector<double> x = emb.x;
            std::vector<double> y = emb.y;
            std::vector<double> z = emb.z;

            auto const bins_xy = std::max<std::size_t>(4, opt.force_bins);
            auto const bins_z = std::max<std::size_t>(2, std::min<std::size_t>(8, bins_xy));
            auto const iters = std::max<std::size_t>(1, opt.force_iterations);
            auto const k_attr = opt.force_attraction;
            auto const k_rep = opt.force_repulsion;

            std::vector<double> fx(ctx.movable.size(), 0.0);
            std::vector<double> fy(ctx.movable.size(), 0.0);
            std::vector<double> fz(ctx.movable.size(), 0.0);
            std::vector<std::vector<std::size_t>> buckets(bins_xy * bins_xy * bins_z);

            auto clamp01 = [](double v) -> double { return std::min(1.0, std::max(0.0, v)); };
            auto bin_idx = [&](double px, double py, double pz) -> std::size_t
            {
                auto bx = static_cast<int>(std::floor(clamp01(px) * static_cast<double>(bins_xy)));
                auto by = static_cast<int>(std::floor(clamp01(py) * static_cast<double>(bins_xy)));
                auto bz = static_cast<int>(std::floor(clamp01(pz) * static_cast<double>(bins_z)));
                bx = std::clamp(bx, 0, static_cast<int>(bins_xy - 1));
                by = std::clamp(by, 0, static_cast<int>(bins_xy - 1));
                bz = std::clamp(bz, 0, static_cast<int>(bins_z - 1));
                return (static_cast<std::size_t>(bz) * bins_xy + static_cast<std::size_t>(by)) * bins_xy + static_cast<std::size_t>(bx);
            };

            for(std::size_t it{}; it < iters; ++it)
            {
                for(auto& b: buckets) { b.clear(); }
                for(std::size_t i{}; i < ctx.movable.size(); ++i) { buckets[bin_idx(x[i], y[i], z[i])].push_back(i); }

                std::fill(fx.begin(), fx.end(), 0.0);
                std::fill(fy.begin(), fy.end(), 0.0);
                std::fill(fz.begin(), fz.end(), 0.0);

                // Repulsion (local, approximate).
                for(std::size_t bz{}; bz < bins_z; ++bz)
                {
                    for(std::size_t by{}; by < bins_xy; ++by)
                    {
                        for(std::size_t bx{}; bx < bins_xy; ++bx)
                        {
                            auto const b0 = (bz * bins_xy + by) * bins_xy + bx;
                            auto const& v0 = buckets[b0];
                            for(auto i: v0)
                            {
                                auto const i_bx = static_cast<int>(bx);
                                auto const i_by = static_cast<int>(by);
                                auto const i_bz = static_cast<int>(bz);
                                for(int dz_i = -1; dz_i <= 1; ++dz_i)
                                {
                                    for(int dy_i = -1; dy_i <= 1; ++dy_i)
                                    {
                                        for(int dx_i = -1; dx_i <= 1; ++dx_i)
                                        {
                                            auto const nbx = i_bx + dx_i;
                                            auto const nby = i_by + dy_i;
                                            auto const nbz = i_bz + dz_i;
                                            if(nbx < 0 || nby < 0 || nbz < 0) { continue; }
                                            if(nbx >= static_cast<int>(bins_xy) || nby >= static_cast<int>(bins_xy) || nbz >= static_cast<int>(bins_z)) { continue; }
                                            auto const& v1 =
                                                buckets[(static_cast<std::size_t>(nbz) * bins_xy + static_cast<std::size_t>(nby)) * bins_xy + static_cast<std::size_t>(nbx)];
                                            for(auto j: v1)
                                            {
                                                if(j <= i) { continue; }
                                                double dxp = x[i] - x[j];
                                                double dyp = y[i] - y[j];
                                                double dzp = z[i] - z[j];
                                                double const d2 = dxp * dxp + dyp * dyp + dzp * dzp + 1e-6;
                                                double const inv = k_rep / d2;
                                                fx[i] += dxp * inv;
                                                fy[i] += dyp * inv;
                                                fz[i] += dzp * inv;
                                                fx[j] -= dxp * inv;
                                                fy[j] -= dyp * inv;
                                                fz[j] -= dzp * inv;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                // Attraction (edges).
                for(auto const& e: edges)
                {
                    auto const a = e.a;
                    auto const b = e.b;
                    double const dxp = x[b] - x[a];
                    double const dyp = y[b] - y[a];
                    double const dzp = z[b] - z[a];
                    fx[a] += dxp * (k_attr * e.w);
                    fy[a] += dyp * (k_attr * e.w);
                    fz[a] += dzp * (k_attr * e.w);
                    fx[b] -= dxp * (k_attr * e.w);
                    fy[b] -= dyp * (k_attr * e.w);
                    fz[b] -= dzp * (k_attr * e.w);
                }

                // Attraction to fixed anchors (x/y only).
                for(auto const& a: anchors)
                {
                    double const dxp = a.x - x[a.a];
                    double const dyp = a.y - y[a.a];
                    fx[a.a] += dxp * (k_attr * a.w);
                    fy[a.a] += dyp * (k_attr * a.w);
                }

                // Integrate with annealing-like step.
                double const t = 1.0 - (static_cast<double>(it) / static_cast<double>(iters));
                double const step = 0.2 * t;
                for(std::size_t i{}; i < ctx.movable.size(); ++i)
                {
                    x[i] = clamp01(x[i] + std::clamp(fx[i], -step, step));
                    y[i] = clamp01(y[i] + std::clamp(fy[i], -step, step));
                    z[i] = clamp01(z[i] + std::clamp(fz[i], -step, step));
                }
            }

            // Quantize Z into multiple planes, then snap each plane to the 2D grid collision-free.
            auto const planes = [&]() -> std::size_t
            {
                auto const n = ctx.movable.size();
                if(n == 0) { return 0; }
                auto p = static_cast<std::size_t>(std::ceil(std::sqrt(static_cast<double>(n))));
                p = std::max<std::size_t>(1, p);
                p = std::min<std::size_t>(p, n);
                p = std::min<std::size_t>(p, 64);
                return p;
            }();
            if(planes == 0) { return stats{.layout_mode = mode::force}; }

            auto plane_of = [&](double zn) -> std::size_t
            {
                if(planes <= 1) { return 0; }
                auto p = static_cast<int>(std::llround(clamp01(zn) * static_cast<double>(planes - 1)));
                p = std::clamp(p, 0, static_cast<int>(planes - 1));
                return static_cast<std::size_t>(p);
            };

            std::vector<std::vector<std::size_t>> by_plane(planes);
            for(auto idx: ctx.movable)
            {
                auto const si = to_sub[idx];
                if(si < 0) { continue; }
                by_plane[plane_of(z[static_cast<std::size_t>(si)])].push_back(idx);
            }

            auto degree = [&](std::size_t idx) -> std::size_t { return ctx.adj[idx].size(); };
            for(auto& nodes: by_plane)
            {
                std::stable_sort(nodes.begin(),
                                 nodes.end(),
                                 [&](std::size_t a, std::size_t b)
                                 {
                                     auto const& ea = ex.elements()[a];
                                     auto const& eb = ex.elements()[b];
                                     if(ea.is_big_element() != eb.is_big_element()) { return ea.is_big_element() > eb.is_big_element(); }
                                     auto const da = degree(a);
                                     auto const db = degree(b);
                                     if(da != db) { return da > db; }
                                     return ea.identifier() < eb.identifier();
                                 });
            }

            std::size_t fixed_obstacles{};
            std::size_t placed_count{};
            std::size_t skipped_count{};

            for(std::size_t plane{}; plane < planes; ++plane)
            {
                auto const z_layer = z_base + static_cast<double>(plane) * z_step_3d;

                occupancy occ_layer(ctx.w, ctx.h);
                if(opt.respect_fixed_elements)
                {
                    for(auto const& f: fixed)
                    {
                        if(!same_z_plane(f.z_native, z_layer)) { continue; }
                        if(!occ_layer.can_place(f.c, f.fp)) { continue; }
                        occ_layer.occupy(f.c, f.fp, static_cast<int>(f.idx));
                        ++fixed_obstacles;
                    }
                }

                for(auto idx: by_plane[plane])
                {
                    auto const fp = element_footprint(ex.elements()[idx], opt);
                    auto const win = full_window(ctx.w, ctx.h, fp);
                    if(!win)
                    {
                        std::string msg = "auto_layout: bounds too small for element footprint";
                        ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                        return status{std::errc::invalid_argument, std::move(msg)};
                    }

                    auto const si = to_sub[idx];
                    auto const u = (si >= 0) ? x[static_cast<std::size_t>(si)] : 0.5;
                    auto const v = (si >= 0) ? y[static_cast<std::size_t>(si)] : 0.5;

                    cell ideal{
                        static_cast<int>(std::llround(u * static_cast<double>(ctx.w - 1))),
                        static_cast<int>(std::llround(v * static_cast<double>(ctx.h - 1))),
                    };
                    ideal.x = std::clamp(ideal.x, win->min_x, win->max_x);
                    ideal.y = std::clamp(ideal.y, win->min_y, win->max_y);

                    auto chosen =
                        choose_cell(occ_layer, ideal, fp, ctx.adj[idx], ctx.placed, opt.max_candidates_per_element, opt.max_search_radius, 0.35, 1.2, win);
                    if(!chosen)
                    {
                        ++skipped_count;
                        continue;
                    }

                    occ_layer.occupy(*chosen, fp, static_cast<int>(idx));
                    ctx.placed[idx] = *chosen;
                    ++placed_count;

                    auto const native_pos = cell_to_native(ctx.bounds, opt.step_x, opt.step_y, *chosen, z_layer);
                    auto id_r = ex.elements()[idx].identifier_ec();
                    if(!id_r) { return id_r.st; }
                    auto* el = ex.find_element(*id_r.value);
                    if(el == nullptr)
                    {
                        auto msg = "auto_layout: unknown element identifier: " + *id_r.value;
                        ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                        return status{std::errc::invalid_argument, std::move(msg)};
                    }
                    set_element_native_position(ex, *el, native_pos);
                }
            }

            return stats{
                .layout_mode = mode::force,
                .grid_w = ctx.w,
                .grid_h = ctx.h,
                .step_x = opt.step_x,
                .step_y = opt.step_y,
                .fixed_obstacles = fixed_obstacles,
                .placed = placed_count,
                .skipped = skipped_count,
            };
        }
    }  // namespace detail

    inline status_or<stats> layout_ec(experiment& ex, position corner0, position corner1, double z_fixed, options const& opt = {}) noexcept
    {
        if(opt.backend_kind == backend::cuda)
        {
            if(!opt.cuda.fn)
            {
                std::string msg = "auto_layout: CUDA backend requested but no dispatch function is provided";
                ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                return status{std::errc::invalid_argument, std::move(msg)};
            }
            if(!std::isfinite(z_fixed))
            {
                std::string msg = "auto_layout: z_fixed must be finite";
                ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
                return status{std::errc::invalid_argument, std::move(msg)};
            }
            auto const bounds_r = normalize_bounds_ec(corner0, corner1, opt.margin_x, opt.margin_y);
            if(!bounds_r) { return bounds_r.st; }
            opt.cuda.fn(ex, *bounds_r.value, z_fixed, opt.cuda.opt_opaque);
            return stats{.layout_mode = opt.layout_mode};
        }

        switch(opt.layout_mode)
        {
            case mode::fast: return detail::layout_cpu_fast_ec(ex, corner0, corner1, z_fixed, opt);
            case mode::cluster: return detail::layout_cpu_cluster_ec(ex, corner0, corner1, z_fixed, opt);
            case mode::spectral: return detail::layout_cpu_spectral_ec(ex, corner0, corner1, z_fixed, opt);
            case mode::hierarchical: return detail::layout_cpu_hierarchical_ec(ex, corner0, corner1, z_fixed, opt);
            case mode::force: return detail::layout_cpu_force_ec(ex, corner0, corner1, z_fixed, opt);
            default: return detail::layout_cpu_fast_ec(ex, corner0, corner1, z_fixed, opt);
        }
    }

    // `*_3d` entry points:
    // - All 3D layouts use `z_step_3d` spacing in native coordinates.
    // - `layout_a_3d` / `layout_d_3d` advance `z_io` by `z_step_3d` on success.

    inline status_or<stats> layout_a_3d_ec(experiment& ex, position corner0, position corner1, double& z_io, options const& opt = {}) noexcept
    {
        if(!std::isfinite(z_io))
        {
            std::string msg = "auto_layout: z_io must be finite";
            ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
            return status{std::errc::invalid_argument, std::move(msg)};
        }
        double const z_next = z_io + z_step_3d;
        auto st = layout_ec(ex, corner0, corner1, z_next, opt);
        if(!st) { return st.st; }
        z_io = z_next;
        return std::move(*st.value);
    }

    inline status_or<stats> layout_b_3d_ec(experiment& ex, position corner0, position corner1, double z_base, options const& opt = {}) noexcept
    {
        return detail::layout_cpu_b_3d_ec(ex, corner0, corner1, z_base, opt);
    }

    inline status_or<stats> layout_c_3d_ec(experiment& ex, position corner0, position corner1, double z_base, options const& opt = {}) noexcept
    {
        return detail::layout_cpu_c_3d_ec(ex, corner0, corner1, z_base, opt);
    }

    inline status_or<stats> layout_d_3d_ec(experiment& ex, position corner0, position corner1, double& z_io, options const& opt = {}) noexcept
    {
        if(!std::isfinite(z_io))
        {
            std::string msg = "auto_layout: z_io must be finite";
            ::phy_engine::phy_lab_wrapper::detail::set_last_error(msg);
            return status{std::errc::invalid_argument, std::move(msg)};
        }
        double const z_next = z_io + z_step_3d;
        auto st = detail::layout_cpu_packing_ec(ex, corner0, corner1, z_next, opt);
        if(!st) { return st.st; }
        z_io = z_next;
        return std::move(*st.value);
    }

#if PHY_ENGINE_ENABLE_EXCEPTIONS
    inline stats layout(experiment& ex, position corner0, position corner1, double z_fixed, options const& opt = {})
    {
        auto r = layout_ec(ex, corner0, corner1, z_fixed, opt);
        if(!r) { throw std::runtime_error(r.st.message); }
        return std::move(*r.value);
    }

    inline stats layout_a_3d(experiment& ex, position corner0, position corner1, double& z_io, options const& opt = {})
    {
        auto r = layout_a_3d_ec(ex, corner0, corner1, z_io, opt);
        if(!r) { throw std::runtime_error(r.st.message); }
        return std::move(*r.value);
    }

    inline stats layout_b_3d(experiment& ex, position corner0, position corner1, double z_base, options const& opt = {})
    {
        auto r = layout_b_3d_ec(ex, corner0, corner1, z_base, opt);
        if(!r) { throw std::runtime_error(r.st.message); }
        return std::move(*r.value);
    }

    inline stats layout_c_3d(experiment& ex, position corner0, position corner1, double z_base, options const& opt = {})
    {
        auto r = layout_c_3d_ec(ex, corner0, corner1, z_base, opt);
        if(!r) { throw std::runtime_error(r.st.message); }
        return std::move(*r.value);
    }

    inline stats layout_d_3d(experiment& ex, position corner0, position corner1, double& z_io, options const& opt = {})
    {
        auto r = layout_d_3d_ec(ex, corner0, corner1, z_io, opt);
        if(!r) { throw std::runtime_error(r.st.message); }
        return std::move(*r.value);
    }
#endif
}  // namespace phy_engine::phy_lab_wrapper::auto_layout
