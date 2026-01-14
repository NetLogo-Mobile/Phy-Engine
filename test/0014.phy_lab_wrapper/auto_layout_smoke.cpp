#include <phy_engine/phy_lab_wrapper/auto_layout/auto_layout.h>

#include <cassert>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <unordered_set>
#include <vector>

namespace
{
using namespace phy_engine::phy_lab_wrapper;

position to_native(experiment const& ex, element const& e)
{
    if (e.is_element_xyz())
    {
        return element_xyz::to_native(e.element_position(), ex.element_xyz_origin(), e.is_big_element());
    }
    return e.element_position();
}
}  // namespace

int main()
{
    using namespace phy_engine::phy_lab_wrapper;
    using namespace phy_engine::phy_lab_wrapper::auto_layout;

    for (auto const layout_mode : {mode::fast, mode::cluster, mode::spectral, mode::hierarchical, mode::force})
    {
        auto ex = experiment::create(experiment_type::circuit);
        ex.set_element_xyz(true, position{0.0, 0.0, 0.0});

        // Fixed corner markers (should not be moved).
        auto lt = ex.add_circuit_element("Student Source", {-1.0, 1.0, 0.0}, false, false, false);
        auto lb = ex.add_circuit_element("Incandescent Lamp", {-1.0, -1.0, 0.0}, false, false, false);
        auto rt = ex.add_circuit_element("Simple Switch", {1.0, 1.0, 0.0}, false, false, false);
        auto rb = ex.add_circuit_element("Battery Source", {1.0, -1.0, 0.0}, false, false, false);
        (void)lt;
        (void)lb;
        (void)rt;
        (void)rb;

        // Movable elements (many start at the same position).
        std::vector<std::string> ids;
        ids.reserve(32);

        for (std::size_t i{}; i < 20; ++i)
        {
            // Uses element-xyz coords (inherits experiment default).
            ids.push_back(ex.add_circuit_element("And Gate", {0.0, 0.0, 0.0}));
        }

        // A big element should occupy more grid cells.
        auto big = ex.add_circuit_element("Counter", {0.0, 0.0, 0.0}, std::nullopt, true, true);
        ids.push_back(big);

        for (std::size_t i{}; i + 1 < ids.size(); ++i)
        {
            ex.connect(ids[i], 0, ids[i + 1], 1);
        }

        options opt;
        opt.mode = layout_mode;
        opt.step_x = 0.16;
        opt.step_y = 0.08;
        opt.small_element = {1, 1};
        opt.big_element = {2, 2};
        opt.respect_fixed_elements = true;

        auto const st = layout(ex, {-1.0, -1.0, 0.0}, {1.0, 1.0, 0.0}, 0.0, opt);
        assert(st.placed == ids.size());
        assert(st.skipped == 0);

        // Verify fixed markers stayed fixed.
        {
            auto const lt_pos = to_native(ex, ex.get_element(lt));
            auto const lb_pos = to_native(ex, ex.get_element(lb));
            auto const rt_pos = to_native(ex, ex.get_element(rt));
            auto const rb_pos = to_native(ex, ex.get_element(rb));
            assert(lt_pos.x == -1.0 && lt_pos.y == 1.0);
            assert(lb_pos.x == -1.0 && lb_pos.y == -1.0);
            assert(rt_pos.x == 1.0 && rt_pos.y == 1.0);
            assert(rb_pos.x == 1.0 && rb_pos.y == -1.0);
        }

        // Verify no grid-cell overlap in the whole rectangle.
        auto const bounds = normalize_bounds({-1.0, -1.0, 0.0}, {1.0, 1.0, 0.0}, opt.margin_x, opt.margin_y);
        auto const grid_w = static_cast<std::size_t>(std::floor((bounds.max_x - bounds.min_x) / opt.step_x + 1e-12)) + 1;
        auto const grid_h = static_cast<std::size_t>(std::floor((bounds.max_y - bounds.min_y) / opt.step_y + 1e-12)) + 1;
        assert(grid_w > 0 && grid_h > 0);

        auto key = [&](std::size_t x, std::size_t y) -> std::size_t { return y * grid_w + x; };
        std::unordered_set<std::size_t> used;

        for (auto const& e : ex.elements())
        {
            auto const native = to_native(ex, e);
            assert(std::fabs(native.z - 0.0) < 1e-9);
            assert(native.x >= bounds.min_x - 1e-9 && native.x <= bounds.max_x + 1e-9);
            assert(native.y >= bounds.min_y - 1e-9 && native.y <= bounds.max_y + 1e-9);

            auto const fp = e.is_big_element() ? opt.big_element : opt.small_element;
            auto ix_i = static_cast<int>(std::llround((native.x - bounds.min_x) / opt.step_x));
            auto iy_i = static_cast<int>(std::llround((native.y - bounds.min_y) / opt.step_y));
            ix_i = std::clamp(ix_i, 0, static_cast<int>(grid_w - fp.w));
            iy_i = std::clamp(iy_i, 0, static_cast<int>(grid_h - fp.h));
            auto const ix = static_cast<std::size_t>(ix_i);
            auto const iy = static_cast<std::size_t>(iy_i);

            for (std::size_t dy{}; dy < fp.h; ++dy)
            {
                for (std::size_t dx{}; dx < fp.w; ++dx)
                {
                    auto const k = key(ix + dx, iy + dy);
                    assert(!used.contains(k));
                    used.insert(k);
                }
            }
        }
    }

    return 0;
}
