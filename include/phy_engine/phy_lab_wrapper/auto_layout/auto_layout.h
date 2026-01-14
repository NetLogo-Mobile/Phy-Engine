#pragma once

#include "../physicslab.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <limits>
#include <optional>
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

inline bounds2d normalize_bounds(position a, position b, double margin_x = 0.0, double margin_y = 0.0)
{
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(b.x) || !std::isfinite(b.y))
    {
        throw std::invalid_argument("auto_layout: bounds must be finite");
    }
    if (!std::isfinite(margin_x) || !std::isfinite(margin_y) || margin_x < 0.0 || margin_y < 0.0)
    {
        throw std::invalid_argument("auto_layout: margins must be finite and >= 0");
    }

    bounds2d out{
        .min_x = std::min(a.x, b.x) + margin_x,
        .min_y = std::min(a.y, b.y) + margin_y,
        .max_x = std::max(a.x, b.x) - margin_x,
        .max_y = std::max(a.y, b.y) - margin_y,
    };
    if (out.max_x < out.min_x || out.max_y < out.min_y)
    {
        throw std::invalid_argument("auto_layout: bounds too small after margins");
    }
    return out;
}

enum class backend : int
{
    cpu = 0,
    cuda = 1,
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
    backend backend{backend::cpu};

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

    // Optional dispatch hook for a future CUDA implementation.
    cuda_dispatch cuda{};
};

struct stats
{
    std::size_t grid_w{};
    std::size_t grid_h{};
    double step_x{};
    double step_y{};
    std::size_t fixed_obstacles{};
    std::size_t placed{};
    std::size_t skipped{};
};

namespace detail
{
struct cell
{
    int x{};
    int y{};
};

inline bool in_range(int v, int lo, int hi) noexcept { return v >= lo && v <= hi; }

inline footprint element_footprint(element const& e, options const& opt) noexcept
{
    return e.is_big_element() ? opt.big_element : opt.small_element;
}

inline std::size_t grid_index(std::size_t x, std::size_t y, std::size_t w) noexcept { return y * w + x; }

struct occupancy
{
    std::size_t w{};
    std::size_t h{};
    std::vector<int> cells{};  // -1 empty; otherwise occupied.

    occupancy(std::size_t w_, std::size_t h_) : w(w_), h(h_), cells(w_ * h_, -1) {}

    [[nodiscard]] bool can_place(cell c, footprint fp) const noexcept
    {
        if (c.x < 0 || c.y < 0) return false;
        auto const ux = static_cast<std::size_t>(c.x);
        auto const uy = static_cast<std::size_t>(c.y);
        if (ux + fp.w > w || uy + fp.h > h) return false;
        for (std::size_t dy{}; dy < fp.h; ++dy)
        {
            for (std::size_t dx{}; dx < fp.w; ++dx)
            {
                if (cells[grid_index(ux + dx, uy + dy, w)] != -1) return false;
            }
        }
        return true;
    }

    void occupy(cell c, footprint fp, int tag)
    {
        auto const ux = static_cast<std::size_t>(c.x);
        auto const uy = static_cast<std::size_t>(c.y);
        for (std::size_t dy{}; dy < fp.h; ++dy)
        {
            for (std::size_t dx{}; dx < fp.w; ++dx)
            {
                cells[grid_index(ux + dx, uy + dy, w)] = tag;
            }
        }
    }
};

inline std::size_t grid_w_from(bounds2d const& b, double step_x)
{
    auto const span = b.max_x - b.min_x;
    if (!(span >= 0.0) || step_x <= 0.0 || !std::isfinite(step_x))
    {
        throw std::invalid_argument("auto_layout: invalid step_x");
    }
    return static_cast<std::size_t>(std::floor(span / step_x + 1e-12)) + 1;
}

inline std::size_t grid_h_from(bounds2d const& b, double step_y)
{
    auto const span = b.max_y - b.min_y;
    if (!(span >= 0.0) || step_y <= 0.0 || !std::isfinite(step_y))
    {
        throw std::invalid_argument("auto_layout: invalid step_y");
    }
    return static_cast<std::size_t>(std::floor(span / step_y + 1e-12)) + 1;
}

inline cell snap_native_to_cell(bounds2d const& b, double step_x, double step_y, position p)
{
    auto const fx = (p.x - b.min_x) / step_x;
    auto const fy = (p.y - b.min_y) / step_y;
    if (!std::isfinite(fx) || !std::isfinite(fy))
    {
        return cell{-1, -1};
    }
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
    if (e.is_element_xyz())
    {
        auto const origin = ex.element_xyz_origin();
        position el_pos{
            (native_pos.x - origin.x) / element_xyz::x_unit,
            (native_pos.y - origin.y) / element_xyz::y_unit,
            (native_pos.z - origin.z) / element_xyz::z_unit,
        };
        if (e.is_big_element())
        {
            // Invert `element_xyz::to_native` exactly so big-element Y amendments round-trip.
            el_pos.y -= (element_xyz::y_amend_big_element / element_xyz::y_unit);
        }
        e.set_element_position(el_pos, true);
        return;
    }
    e.set_element_position(native_pos, false);
}

inline std::vector<std::vector<std::size_t>> build_adjacency(experiment const& ex,
                                                             std::unordered_map<std::string, std::size_t> const& id_to_index)
{
    std::vector<std::vector<std::size_t>> adj(ex.elements().size());
    for (auto const& w : ex.wires())
    {
        auto it_s = id_to_index.find(w.source.element_identifier);
        auto it_t = id_to_index.find(w.target.element_identifier);
        if (it_s == id_to_index.end() || it_t == id_to_index.end()) continue;
        auto const s = it_s->second;
        auto const t = it_t->second;
        if (s == t) continue;
        adj[s].push_back(t);
        adj[t].push_back(s);
    }
    for (auto& v : adj)
    {
        std::sort(v.begin(), v.end());
        v.erase(std::unique(v.begin(), v.end()), v.end());
    }
    return adj;
}

inline double placement_cost(cell candidate,
                             cell ideal,
                             std::vector<std::size_t> const& neighbors,
                             std::vector<std::optional<cell>> const& placed)
{
    double cost = 0.0;
    for (auto ni : neighbors)
    {
        if (!placed[ni]) continue;
        auto const p = *placed[ni];
        cost += static_cast<double>(std::abs(candidate.x - p.x) + std::abs(candidate.y - p.y));
    }
    cost += 0.1 * static_cast<double>(std::abs(candidate.x - ideal.x) + std::abs(candidate.y - ideal.y));
    return cost;
}

inline std::optional<cell> choose_cell(detail::occupancy const& occ,
                                      cell ideal,
                                      footprint fp,
                                      std::vector<std::size_t> const& neighbors,
                                      std::vector<std::optional<cell>> const& placed,
                                      std::size_t max_candidates,
                                      std::optional<std::size_t> max_radius_opt)
{
    auto const max_radius = [&]() -> std::size_t {
        if (max_radius_opt) return *max_radius_opt;
        return static_cast<std::size_t>(std::max<int>(static_cast<int>(occ.w), static_cast<int>(occ.h)));
    }();

    std::optional<cell> best{};
    double best_cost = std::numeric_limits<double>::infinity();
    std::size_t visited{};

    auto consider = [&](cell c) {
        if (visited >= max_candidates) return;
        ++visited;
        if (!occ.can_place(c, fp)) return;
        auto const cost = placement_cost(c, ideal, neighbors, placed);
        if (!best || cost < best_cost ||
            (cost == best_cost &&
             (std::abs(c.x - ideal.x) + std::abs(c.y - ideal.y)) < (std::abs(best->x - ideal.x) + std::abs(best->y - ideal.y))) ||
            (cost == best_cost && c.x < best->x) ||
            (cost == best_cost && c.x == best->x && c.y < best->y))
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

    for (std::size_t r{}; r <= max_radius && visited < max_candidates; ++r)
    {
        auto const ir = static_cast<int>(r);
        if (r == 0)
        {
            consider(ideal);
            continue;
        }

        // Top and bottom edges of the ring.
        for (int dx = -ir; dx <= ir && visited < max_candidates; ++dx)
        {
            cell c1{ideal.x + dx, ideal.y - ir};
            cell c2{ideal.x + dx, ideal.y + ir};
            if (in_range(c1.x, lo_x, hi_x) && in_range(c1.y, lo_y, hi_y)) consider(c1);
            if (in_range(c2.x, lo_x, hi_x) && in_range(c2.y, lo_y, hi_y)) consider(c2);
        }
        // Left and right edges (excluding corners already checked).
        for (int dy = -ir + 1; dy <= ir - 1 && visited < max_candidates; ++dy)
        {
            cell c1{ideal.x - ir, ideal.y + dy};
            cell c2{ideal.x + ir, ideal.y + dy};
            if (in_range(c1.x, lo_x, hi_x) && in_range(c1.y, lo_y, hi_y)) consider(c1);
            if (in_range(c2.x, lo_x, hi_x) && in_range(c2.y, lo_y, hi_y)) consider(c2);
        }

        if (best_cost == 0.0) break;
    }
    return best;
}

inline cell default_seed_cell(std::size_t w, std::size_t h) noexcept
{
    return cell{static_cast<int>(w / 2), static_cast<int>(h / 2)};
}

inline stats layout_cpu(experiment& ex, position corner0, position corner1, double z_fixed, options const& opt)
{
    if (!std::isfinite(z_fixed))
    {
        throw std::invalid_argument("auto_layout: z_fixed must be finite");
    }

    auto const bounds = normalize_bounds(corner0, corner1, opt.margin_x, opt.margin_y);
    auto const w = grid_w_from(bounds, opt.step_x);
    auto const h = grid_h_from(bounds, opt.step_y);
    if (w == 0 || h == 0)
    {
        throw std::runtime_error("auto_layout: empty grid");
    }

    constexpr std::size_t kMaxGridCells = 2'000'000;
    if (w > 0 && h > 0 && w > (kMaxGridCells / h))
    {
        throw std::runtime_error("auto_layout: grid too large (reduce bounds or increase step)");
    }

    occupancy occ(w, h);

    std::unordered_map<std::string, std::size_t> id_to_index;
    id_to_index.reserve(ex.elements().size());
    for (std::size_t i{}; i < ex.elements().size(); ++i)
    {
        id_to_index.emplace(ex.elements()[i].identifier(), i);
    }

    auto const adj = build_adjacency(ex, id_to_index);

    std::vector<std::optional<cell>> placed(ex.elements().size(), std::nullopt);
    std::vector<std::size_t> movable{};
    movable.reserve(ex.elements().size());

    std::size_t fixed_obstacles{};
    for (std::size_t i{}; i < ex.elements().size(); ++i)
    {
        auto const& e = ex.elements()[i];
        if (e.participate_in_layout())
        {
            movable.push_back(i);
            continue;
        }
        if (!opt.respect_fixed_elements) continue;

        auto const native_pos = e.is_element_xyz() ? element_xyz::to_native(e.element_position(), ex.element_xyz_origin(), e.is_big_element())
                                                   : e.element_position();
        auto c = snap_native_to_cell(bounds, opt.step_x, opt.step_y, native_pos);
        auto const fp = element_footprint(e, opt);
        if (w < fp.w || h < fp.h)
        {
            throw std::runtime_error("auto_layout: bounds too small for element footprint");
        }
        c.x = std::clamp(c.x, 0, static_cast<int>(w - fp.w));
        c.y = std::clamp(c.y, 0, static_cast<int>(h - fp.h));
        if (!occ.can_place(c, fp)) continue;
        occ.occupy(c, fp, static_cast<int>(i));
        placed[i] = c;
        ++fixed_obstacles;
    }

    auto degree = [&](std::size_t idx) -> std::size_t { return adj[idx].size(); };
    std::stable_sort(movable.begin(), movable.end(), [&](std::size_t a, std::size_t b) {
        auto const& ea = ex.elements()[a];
        auto const& eb = ex.elements()[b];
        if (ea.is_big_element() != eb.is_big_element()) return ea.is_big_element() > eb.is_big_element();
        auto const da = degree(a);
        auto const db = degree(b);
        if (da != db) return da > db;
        return ea.identifier() < eb.identifier();
    });

    auto seed = default_seed_cell(w, h);

    std::size_t placed_count{};
    std::size_t skipped_count{};

    for (auto idx : movable)
    {
        auto const fp = element_footprint(ex.elements()[idx], opt);

        cell ideal = seed;
        {
            std::int64_t sum_x{};
            std::int64_t sum_y{};
            std::size_t cnt{};
            for (auto nb : adj[idx])
            {
                if (!placed[nb]) continue;
                sum_x += placed[nb]->x;
                sum_y += placed[nb]->y;
                ++cnt;
            }
            if (cnt != 0)
            {
                ideal.x = static_cast<int>(std::llround(static_cast<double>(sum_x) / static_cast<double>(cnt)));
                ideal.y = static_cast<int>(std::llround(static_cast<double>(sum_y) / static_cast<double>(cnt)));
            }
        }

        auto chosen = choose_cell(occ,
                                  ideal,
                                  fp,
                                  adj[idx],
                                  placed,
                                  opt.max_candidates_per_element,
                                  opt.max_search_radius);
        if (!chosen)
        {
            ++skipped_count;
            continue;
        }

        occ.occupy(*chosen, fp, static_cast<int>(idx));
        placed[idx] = *chosen;
        ++placed_count;
    }

    for (auto idx : movable)
    {
        if (!placed[idx]) continue;
        auto const native_pos = cell_to_native(bounds, opt.step_x, opt.step_y, *placed[idx], z_fixed);
        set_element_native_position(ex, ex.get_element(ex.elements()[idx].identifier()), native_pos);
    }

    return stats{
        .grid_w = w,
        .grid_h = h,
        .step_x = opt.step_x,
        .step_y = opt.step_y,
        .fixed_obstacles = fixed_obstacles,
        .placed = placed_count,
        .skipped = skipped_count,
    };
}
}  // namespace detail

inline stats layout(experiment& ex, position corner0, position corner1, double z_fixed, options const& opt = {})
{
    if (opt.backend == backend::cuda)
    {
        if (!opt.cuda.fn)
        {
            throw std::runtime_error("auto_layout: CUDA backend requested but no dispatch function is provided");
        }
        auto const bounds = normalize_bounds(corner0, corner1, opt.margin_x, opt.margin_y);
        opt.cuda.fn(ex, bounds, z_fixed, opt.cuda.opt_opaque);
        return stats{};
    }
    return detail::layout_cpu(ex, corner0, corner1, z_fixed, opt);
}
}  // namespace phy_engine::phy_lab_wrapper::auto_layout
