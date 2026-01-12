#pragma once

#include "physicslab.h"

#include <algorithm>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace phy_engine::phy_lab_wrapper::layout
{
struct corner_markers
{
    std::string_view left_top_model_id;
    std::string_view left_bottom_model_id;
    std::string_view right_top_model_id;
    std::string_view right_bottom_model_id;
};

struct corner_locator
{
    position left_top{};
    position left_bottom{};
    position right_top{};
    position right_bottom{};

    // Vectors from left_bottom in native coordinates.
    position x_axis{};  // left->right direction (u axis)
    position y_axis{};  // bottom->top direction (v axis)

    [[nodiscard]] static corner_locator from_experiment(experiment const& ex, corner_markers markers)
    {
        auto find_first_by_model_id = [&](std::string_view model_id) -> std::optional<position> {
            for(auto const& e : ex.elements())
            {
                auto const mid = e.data().value("ModelID", "");
                if(mid == model_id)
                {
                    return e.element_position();
                }
            }
            return std::nullopt;
        };

        auto lt = find_first_by_model_id(markers.left_top_model_id);
        auto lb = find_first_by_model_id(markers.left_bottom_model_id);
        auto rt = find_first_by_model_id(markers.right_top_model_id);
        auto rb = find_first_by_model_id(markers.right_bottom_model_id);
        if(!lt || !lb || !rt || !rb)
        {
            auto missing = [&](std::optional<position> const& p, std::string_view name, std::string_view mid) -> std::string {
                return p ? std::string{} : (std::string(name) + " (" + std::string(mid) + ") ");
            };
            throw std::runtime_error("corner_locator: missing marker(s): " + missing(lt, "left_top", markers.left_top_model_id) +
                                     missing(lb, "left_bottom", markers.left_bottom_model_id) +
                                     missing(rt, "right_top", markers.right_top_model_id) +
                                     missing(rb, "right_bottom", markers.right_bottom_model_id));
        }

        corner_locator out{};
        out.left_top = *lt;
        out.left_bottom = *lb;
        out.right_top = *rt;
        out.right_bottom = *rb;

        // Best-effort: average top/bottom edges for x axis and left/right edges for y axis.
        auto sub = [](position a, position b) -> position { return {a.x - b.x, a.y - b.y, a.z - b.z}; };
        auto add = [](position a, position b) -> position { return {a.x + b.x, a.y + b.y, a.z + b.z}; };
        auto mul = [](position a, double k) -> position { return {a.x * k, a.y * k, a.z * k}; };

        auto x_top = sub(out.right_top, out.left_top);
        auto x_bottom = sub(out.right_bottom, out.left_bottom);
        out.x_axis = mul(add(x_top, x_bottom), 0.5);

        auto y_left = sub(out.left_top, out.left_bottom);
        auto y_right = sub(out.right_top, out.right_bottom);
        out.y_axis = mul(add(y_left, y_right), 0.5);

        return out;
    }

    [[nodiscard]] static corner_locator from_sav(std::filesystem::path const& sav_path, corner_markers markers)
    {
        auto ex = experiment::load(sav_path);
        return from_experiment(ex, markers);
    }

    // Map a point in the unit square (u,v in [0,1]) into native coordinates.
    // u: left->right, v: bottom->top.
    [[nodiscard]] position map_uv(double u, double v, double z_offset = 0.0) const noexcept
    {
        return {
            left_bottom.x + x_axis.x * u + y_axis.x * v,
            left_bottom.y + x_axis.y * u + y_axis.y * v,
            left_bottom.z + x_axis.z * u + y_axis.z * v + z_offset,
        };
    }

    // Map an integer grid (w x h) into native coordinates.
    // If y0_is_top is true, (0,0) maps to left_top; otherwise maps to left_bottom.
    [[nodiscard]] position map_grid(std::size_t x, std::size_t y, std::size_t w, std::size_t h, bool y0_is_top = true) const noexcept
    {
        double const denom_x = (w <= 1) ? 1.0 : static_cast<double>(w - 1);
        double const denom_y = (h <= 1) ? 1.0 : static_cast<double>(h - 1);
        double const u = static_cast<double>(x) / denom_x;
        double v = static_cast<double>(y) / denom_y;
        if(y0_is_top) { v = 1.0 - v; }
        return map_uv(u, v);
    }
};
}  // namespace phy_engine::phy_lab_wrapper::layout
