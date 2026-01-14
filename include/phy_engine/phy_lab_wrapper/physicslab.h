#pragma once

#include "utility/json.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace phy_engine::phy_lab_wrapper
{
using json = nlohmann::json;

struct position
{
    double x{};
    double y{};
    double z{};
};

enum class experiment_type : int
{
    circuit = 0,
    celestial = 3,
    electromagnetism = 4,
};

enum class open_mode : int
{
    load_by_sav_name = 0,
    load_by_filepath = 1,
    load_by_plar_app = 2,
    crt = 3,
};

enum class wire_color : int
{
    black = 0,
    blue = 1,
    red = 2,
    green = 3,
    yellow = 4,
};

inline std::string wire_color_name(wire_color c)
{
    switch (c)
    {
        case wire_color::black: return "黑色导线";
        case wire_color::blue: return "蓝色导线";
        case wire_color::red: return "红色导线";
        case wire_color::green: return "绿色导线";
        case wire_color::yellow: return "黄色导线";
        default: return "蓝色导线";
    }
}

inline wire_color parse_wire_color_name(std::string_view s)
{
    if (s.empty())
    {
        return wire_color::blue;
    }

    // Matches physicsLab naming: "{黑|蓝|红|绿|黄}色导线"
    if (s.starts_with("黑")) return wire_color::black;
    if (s.starts_with("蓝")) return wire_color::blue;
    if (s.starts_with("红")) return wire_color::red;
    if (s.starts_with("绿")) return wire_color::green;
    if (s.starts_with("黄")) return wire_color::yellow;
    return wire_color::blue;
}

struct element_xyz
{
    static constexpr double x_unit{0.16};
    static constexpr double y_unit{0.08};
    static constexpr double z_unit{0.1};
    static constexpr double y_amend_big_element{0.045};

    static position to_native(position element_pos, position origin, bool is_big_element = false)
    {
        position p{
            element_pos.x * x_unit + origin.x,
            element_pos.y * y_unit + origin.y,
            element_pos.z * z_unit + origin.z,
        };
        if (is_big_element)
        {
            p.y += y_amend_big_element;
        }
        return p;
    }

    static position to_element_xyz(position native_pos, position origin, bool is_big_element = false)
    {
        position p{
            (native_pos.x - origin.x) / x_unit,
            (native_pos.y - origin.y) / y_unit,
            (native_pos.z - origin.z) / z_unit,
        };
        if (is_big_element)
        {
            p.y -= y_amend_big_element;
        }
        return p;
    }
};

namespace detail
{
inline int64_t now_ms()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

inline double round6(double v)
{
    if (!std::isfinite(v))
    {
        throw std::invalid_argument("position/rotation must be finite");
    }
    return std::round(v * 1'000'000.0) / 1'000'000.0;
}

inline std::string python_float(double v)
{
    v = round6(v);
    if (std::fabs(v - std::round(v)) < 1e-9)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << v;
        return oss.str();
    }

    std::ostringstream oss;
    oss << std::setprecision(15) << std::defaultfloat << v;
    auto s = oss.str();
    if (auto e = s.find('e'); e != std::string::npos)
    {
        std::ostringstream oss2;
        oss2 << std::fixed << std::setprecision(6) << v;
        s = oss2.str();
    }
    return s;
}

inline std::string pack_xyz(position p)
{
    return python_float(p.x) + "," + python_float(p.z) + "," + python_float(p.y);
}

inline position parse_xyz(std::string_view s)
{
    auto c1 = s.find(',');
    if (c1 == std::string_view::npos) throw std::runtime_error("invalid xyz string");
    auto c2 = s.find(',', c1 + 1);
    if (c2 == std::string_view::npos) throw std::runtime_error("invalid xyz string");

    auto to_double = [](std::string_view sv) -> double {
        std::string tmp(sv);
        std::size_t idx{};
        double v = std::stod(tmp, &idx);
        if (idx != tmp.size()) throw std::runtime_error("invalid number in xyz string");
        return v;
    };

    double x = to_double(s.substr(0, c1));
    double z = to_double(s.substr(c1 + 1, c2 - (c1 + 1)));
    double y = to_double(s.substr(c2 + 1));
    return {x, y, z};
}

inline std::string rand_string(std::size_t len)
{
    static constexpr std::string_view alphabet =
        "abcdefghijklmnopqrstuvwxyz"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "0123456789";

    thread_local std::mt19937_64 rng{std::random_device{}()};
    std::uniform_int_distribution<std::size_t> dist(0, alphabet.size() - 1);

    std::string out;
    out.reserve(len);
    for (std::size_t i{}; i < len; ++i)
    {
        out.push_back(alphabet[dist(rng)]);
    }
    return out;
}

inline std::filesystem::path default_sav_dir()
{
    if (auto* env = std::getenv("PHYSICSLAB_HOME_PATH"); env != nullptr && *env != '\0')
    {
        return std::filesystem::path(env);
    }

#if defined(_WIN32)
    if (auto* home = std::getenv("USERPROFILE"); home != nullptr && *home != '\0')
    {
        return std::filesystem::path(home) / "AppData" / "LocalLow" / "CIVITAS" / "Quantum Physics" / "Circuit";
    }
#endif

    return std::filesystem::current_path() / "physicsLabSav";
}

inline void ensure_directory(std::filesystem::path const& p)
{
    std::error_code ec;
    std::filesystem::create_directories(p, ec);
    if (ec)
    {
        throw std::runtime_error("failed to create directory: " + p.string());
    }
}

inline json default_plsav_template(experiment_type type)
{
    // Mirrors `physicsLab/physicsLab/savTemplate.py` (Generate placeholders are replaced with null / empty strings).
    json tpl;
    switch (type)
    {
        case experiment_type::circuit:
            tpl = json{
                {"Type", 0},
                {"Experiment",
                 {{"ID", nullptr},
                  {"Type", 0},
                  {"Components", 7},
                  {"Subject", nullptr},
                  {"StatusSave", ""},
                  {"CameraSave", ""},
                  {"Version", 2404},
                  {"CreationDate", nullptr},
                  {"Paused", false},
                  {"Summary", nullptr},
                  {"Plots", nullptr}}},
                {"ID", nullptr},
                {"Summary",
                 {{"Type", 0},
                  {"ParentID", nullptr},
                  {"ParentName", nullptr},
                  {"ParentCategory", nullptr},
                  {"ContentID", nullptr},
                  {"Editor", nullptr},
                  {"Coauthors", json::array()},
                  {"Description", nullptr},
                  {"LocalizedDescription", nullptr},
                  {"Tags", json::array({"Type-0"})},
                  {"ModelID", nullptr},
                  {"ModelName", nullptr},
                  {"ModelTags", json::array()},
                  {"Version", 0},
                  {"Language", "Chinese"},
                  {"Visits", 0},
                  {"Stars", 0},
                  {"Supports", 0},
                  {"Remixes", 0},
                  {"Comments", 0},
                  {"Price", 0},
                  {"Popularity", 0},
                  {"CreationDate", nullptr},
                  {"UpdateDate", 0},
                  {"SortingDate", 0},
                  {"ID", nullptr},
                  {"Category", nullptr},
                  {"Subject", ""},
                  {"LocalizedSubject", nullptr},
                  {"Image", 0},
                  {"ImageRegion", 0},
                  {"User",
                   {{"ID", nullptr},
                    {"Nickname", nullptr},
                    {"Signature", nullptr},
                    {"Avatar", 0},
                    {"AvatarRegion", 0},
                    {"Decoration", 0},
                    {"Verification", nullptr}}},
                  {"Visibility", 0},
                  {"Settings", json::object()},
                  {"Anonymous", false},
                  {"Multilingual", false}}},
                {"CreationDate", 0},
                {"Speed", 1.0},
                {"SpeedMinimum", 0.0002},
                {"SpeedMaximum", 2.0},
                {"SpeedReal", 0.0},
                {"Paused", false},
                {"Version", 0},
                {"CameraSnapshot", nullptr},
                {"Plots", json::array()},
                {"Widgets", json::array()},
                {"WidgetGroups", json::array()},
                {"Bookmarks", json::object()},
                {"Interfaces", {{"Play-Expanded", false}, {"Chart-Expanded", false}}}};
            break;
        case experiment_type::celestial:
            tpl = json{
                {"Type", 3},
                {"Experiment",
                 {{"ID", nullptr},
                  {"Type", 3},
                  {"Components", 0},
                  {"Subject", nullptr},
                  {"StatusSave", ""},
                  {"CameraSave", ""},
                  {"Version", 2407},
                  {"CreationDate", nullptr},
                  {"Paused", false},
                  {"Summary", nullptr},
                  {"Plots", nullptr}}},
                {"ID", nullptr},
                {"Summary",
                 {{"Type", 3},
                  {"ParentID", nullptr},
                  {"ParentName", nullptr},
                  {"ParentCategory", nullptr},
                  {"ContentID", nullptr},
                  {"Editor", nullptr},
                  {"Coauthors", json::array()},
                  {"Description", nullptr},
                  {"LocalizedDescription", nullptr},
                  {"Tags", json::array({"Type-3"})},
                  {"ModelID", nullptr},
                  {"ModelName", nullptr},
                  {"ModelTags", json::array()},
                  {"Version", 0},
                  {"Language", nullptr},
                  {"Visits", 0},
                  {"Stars", 0},
                  {"Supports", 0},
                  {"Remixes", 0},
                  {"Comments", 0},
                  {"Price", 0},
                  {"Popularity", 0},
                  {"CreationDate", nullptr},
                  {"UpdateDate", 0},
                  {"SortingDate", 0},
                  {"ID", nullptr},
                  {"Category", nullptr},
                  {"Subject", nullptr},
                  {"LocalizedSubject", nullptr},
                  {"Image", 0},
                  {"ImageRegion", 0},
                  {"User",
                   {{"ID", nullptr},
                    {"Nickname", nullptr},
                    {"Signature", nullptr},
                    {"Avatar", 0},
                    {"AvatarRegion", 0},
                    {"Decoration", 0},
                    {"Verification", nullptr}}},
                  {"Visibility", 0},
                  {"Settings", json::object()},
                  {"Multilingual", false}}},
                {"CreationDate", 0},
                {"Speed", 1.0},
                {"SpeedMinimum", 0.1},
                {"SpeedMaximum", 10.0},
                {"SpeedReal", 0.0},
                {"Paused", false},
                {"Version", 0},
                {"CameraSnapshot", nullptr},
                {"Plots", json::array()},
                {"Widgets", json::array()},
                {"WidgetGroups", json::array()},
                {"Bookmarks", json::object()},
                {"Interfaces", {{"Play-Expanded", false}, {"Chart-Expanded", false}}}};
            break;
        case experiment_type::electromagnetism:
            tpl = json{
                {"Type", 4},
                {"Experiment",
                 {{"ID", nullptr},
                  {"Type", 4},
                  {"Components", 1},
                  {"Subject", nullptr},
                  {"StatusSave", ""},
                  {"CameraSave", ""},
                  {"Version", 2405},
                  {"CreationDate", nullptr},
                  {"Paused", false},
                  {"Summary", nullptr},
                  {"Plots", nullptr}}},
                {"ID", nullptr},
                {"Summary",
                 {{"Type", 4},
                  {"ParentID", nullptr},
                  {"ParentName", nullptr},
                  {"ParentCategory", nullptr},
                  {"ContentID", nullptr},
                  {"Editor", nullptr},
                  {"Coauthors", json::array()},
                  {"Description", nullptr},
                  {"LocalizedDescription", nullptr},
                  {"Tags", json::array({"Type-4"})},
                  {"ModelID", nullptr},
                  {"ModelName", nullptr},
                  {"ModelTags", json::array()},
                  {"Version", 0},
                  {"Language", nullptr},
                  {"Visits", 0},
                  {"Stars", 0},
                  {"Supports", 0},
                  {"Remixes", 0},
                  {"Comments", 0},
                  {"Price", 0},
                  {"Popularity", 0},
                  {"CreationDate", nullptr},
                  {"UpdateDate", 0},
                  {"SortingDate", 0},
                  {"ID", nullptr},
                  {"Category", nullptr},
                  {"Subject", nullptr},
                  {"LocalizedSubject", nullptr},
                  {"Image", 0},
                  {"ImageRegion", 0},
                  {"User",
                   {{"ID", nullptr},
                    {"Nickname", nullptr},
                    {"Signature", nullptr},
                    {"Avatar", 0},
                    {"AvatarRegion", 0},
                    {"Decoration", 0},
                    {"Verification", nullptr}}},
                  {"Visibility", 0},
                  {"Settings", json::object()},
                  {"Multilingual", false}}},
                {"CreationDate", 0},
                {"Speed", 1.0},
                {"SpeedMinimum", 0.1},
                {"SpeedMaximum", 2.0},
                {"SpeedReal", 0.0},
                {"Paused", false},
                {"Version", 0},
                {"CameraSnapshot", nullptr},
                {"Plots", json::array()},
                {"Widgets", json::array()},
                {"WidgetGroups", json::array()},
                {"Bookmarks", json::object()},
                {"Interfaces", {{"Play-Expanded", false}, {"Chart-Expanded", false}}}};
            break;
        default: throw std::runtime_error("unsupported experiment type");
    }
    return tpl;
}
}  // namespace detail

class element
{
public:
    element() = default;

    static element circuit(std::string model_id,
                           position pos,
                           bool element_xyz_coords = false,
                           bool is_big_element = false,
                           bool participate_in_layout = true)
    {
        element e;

        std::string_view const mid = model_id;
        json props = json::object();
        json stats = json::object();

        // Fill minimal defaults for common digital elements so exported .sav matches physicsLab expectations.
        if (mid == "Logic Input")
        {
            props = json{{"高电平", 3.0}, {"低电平", 0.0}, {"锁定", 1.0}, {"开关", 0.0}};
            stats = json{{"电流", 0.0}, {"电压", 0.0}, {"功率", 0.0}};
        }
        else if (mid == "Logic Output")
        {
            props = json{{"状态", 0.0}, {"高电平", 3.0}, {"低电平", 0.0}, {"锁定", 1.0}};
        }
        else if (mid == "Yes Gate" || mid == "No Gate" || mid == "And Gate" || mid == "Or Gate" || mid == "Xor Gate" ||
                 mid == "Xnor Gate" || mid == "Nand Gate" || mid == "Nor Gate" || mid == "Imp Gate" || mid == "Nimp Gate")
        {
            props = json{{"高电平", 3.0}, {"低电平", 0.0}, {"最大电流", 0.1}, {"锁定", 1.0}};
        }
        else if (mid == "Half Adder" || mid == "Full Adder" || mid == "Half Subtractor" || mid == "Full Subtractor" ||
                 mid == "Multiplier" || mid == "D Flipflop" || mid == "T Flipflop" || mid == "Real-T Flipflop" ||
                 mid == "JK Flipflop" || mid == "Counter" || mid == "Random Generator")
        {
            props = json{{"高电平", 3.0}, {"低电平", 0.0}, {"锁定", 1.0}};
        }
        else if (mid == "8bit Input")
        {
            props = json{{"高电平", 3.0}, {"低电平", 0.0}, {"十进制", 0.0}, {"锁定", 1.0}};
        }
        else if (mid == "8bit Display")
        {
            props = json{{"高电平", 3.0}, {"低电平", 0.0}, {"状态", 0.0}, {"锁定", 1.0}};
            stats = json{{"7", 0.0}, {"6", 0.0}, {"5", 0.0}, {"4", 0.0}, {"3", 0.0}, {"2", 0.0}, {"1", 0.0}, {"0", 0.0}, {"十进制", 0.0}};
        }

        e.data_ = json{
            {"ModelID", std::move(model_id)},
            {"Identifier", detail::rand_string(33)},
            {"Label", nullptr},
            {"IsBroken", false},
            {"IsLocked", false},
            {"Properties", std::move(props)},
            {"Statistics", std::move(stats)},
            {"Position", ""},
            {"Rotation", "0,180,0"},
            {"DiagramCached", false},
            {"DiagramPosition", {{"X", 0}, {"Y", 0}, {"Magnitude", 0.0}}},
            {"DiagramRotation", 0},
        };
        e.pos_ = pos;
        e.is_element_xyz_ = element_xyz_coords;
        e.is_big_element_ = is_big_element;
        e.participate_in_layout_ = participate_in_layout;
        return e;
    }

    static element generic(json data,
                           position pos,
                           bool element_xyz_coords = false,
                           bool is_big_element = false,
                           bool participate_in_layout = true)
    {
        element e;
        e.data_ = std::move(data);
        if (!e.data_.contains("Identifier"))
        {
            e.data_["Identifier"] = detail::rand_string(33);
        }
        e.pos_ = pos;
        e.is_element_xyz_ = element_xyz_coords;
        e.is_big_element_ = is_big_element;
        e.participate_in_layout_ = participate_in_layout;
        return e;
    }

    [[nodiscard]] std::string identifier() const
    {
        auto it = data_.find("Identifier");
        if (it == data_.end() || !it->is_string())
        {
            throw std::runtime_error("element missing string Identifier");
        }
        return it->get<std::string>();
    }

    [[nodiscard]] bool is_element_xyz() const noexcept { return is_element_xyz_; }
    [[nodiscard]] bool is_big_element() const noexcept { return is_big_element_; }
    [[nodiscard]] bool participate_in_layout() const noexcept { return participate_in_layout_; }

    [[nodiscard]] position element_position() const noexcept { return pos_; }

    void set_element_position(position pos, bool element_xyz_coords)
    {
        pos_ = pos;
        is_element_xyz_ = element_xyz_coords;
    }

    void set_big_element(bool v) noexcept { is_big_element_ = v; }
    void set_participate_in_layout(bool v) noexcept { participate_in_layout_ = v; }

    void set_rotation(position rot_native_xyz = {0.0, 0.0, 180.0})
    {
        data_["Rotation"] = detail::pack_xyz(rot_native_xyz);
    }

    void write_native_position(position native_pos) { data_["Position"] = detail::pack_xyz(native_pos); }

    [[nodiscard]] std::optional<position> try_read_native_position() const
    {
        auto it = data_.find("Position");
        if (it == data_.end() || !it->is_string())
        {
            return std::nullopt;
        }
        return detail::parse_xyz(it->get_ref<const std::string&>());
    }

    [[nodiscard]] json& data() noexcept { return data_; }
    [[nodiscard]] json const& data() const noexcept { return data_; }

private:
    json data_ = json::object();
    position pos_{};
    bool is_element_xyz_{false};
    bool is_big_element_{false};
    bool participate_in_layout_{true};
};

struct pin_ref
{
    std::string element_identifier;
    int pin{};
};

struct wire
{
    pin_ref source;
    pin_ref target;
    wire_color color{wire_color::blue};

    [[nodiscard]] json to_json() const
    {
        return json{
            {"Source", source.element_identifier},
            {"SourcePin", source.pin},
            {"Target", target.element_identifier},
            {"TargetPin", target.pin},
            {"ColorName", wire_color_name(color)},
        };
    }
};

class experiment
{
public:
    static experiment create(experiment_type type)
    {
        experiment ex;
        ex.type_ = type;
        ex.plsav_ = detail::default_plsav_template(type);
        ex.camera_save_ = json::object();

        // Match physicsLab defaults so generated .sav opens in the official client.
        switch (type)
        {
            case experiment_type::circuit:
                ex.camera_save_["Mode"] = 0;
                ex.camera_save_["Distance"] = 2.7;
                ex.vision_center_ = {0.0, -0.45, 1.08};
                ex.target_rotation_ = {50.0, 0.0, 0.0};
                break;
            case experiment_type::celestial:
                ex.camera_save_["Mode"] = 2;
                ex.camera_save_["Distance"] = 2.75;
                ex.vision_center_ = {0.0, 0.0, 1.08};
                ex.target_rotation_ = {90.0, 0.0, 0.0};
                break;
            case experiment_type::electromagnetism:
                ex.camera_save_["Mode"] = 0;
                ex.camera_save_["Distance"] = 3.25;
                ex.vision_center_ = {0.0, 0.0, 0.88};
                ex.target_rotation_ = {90.0, 0.0, 0.0};
                break;
            default:
                ex.vision_center_ = {0.0, 0.0, 0.0};
                ex.target_rotation_ = {0.0, 0.0, 0.0};
                break;
        }

        ex.camera_save_["VisionCenter"] = detail::pack_xyz(ex.vision_center_);
        ex.camera_save_["TargetRotation"] = detail::pack_xyz(ex.target_rotation_);
        ex.element_xyz_enabled_ = false;
        ex.element_xyz_origin_ = {0.0, 0.0, 0.0};
        return ex;
    }

    static experiment load(std::filesystem::path const& path)
    {
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs.is_open())
        {
            throw std::runtime_error("failed to open sav: " + path.string());
        }

        std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
        return load_from_string(content);
    }

    static experiment load(std::string const& path) { return load(std::filesystem::path(path)); }

    static experiment load_from_string(std::string_view content)
    {
        json root = json::parse(content.begin(), content.end(), nullptr, true, true);
        return load_from_json(std::move(root));
    }

    static experiment load_from_json(json root)
    {
        experiment ex;

        // Full .sav contains {Type, Experiment, Summary, ...}; exported .sav may contain only the "Experiment" object.
        if (root.is_object() && root.contains("Experiment"))
        {
            ex.plsav_ = std::move(root);
            ex.type_ = static_cast<experiment_type>(ex.plsav_.at("Type").get<int>());
        }
        else if (root.is_object() && root.contains("Type"))
        {
            ex.type_ = static_cast<experiment_type>(root.at("Type").get<int>());
            ex.plsav_ = detail::default_plsav_template(ex.type_);
            ex.plsav_["Experiment"] = std::move(root);
        }
        else
        {
            throw std::runtime_error("invalid sav json: expected object");
        }

        auto const& exp = ex.plsav_.at("Experiment");
        if (exp.contains("CameraSave") && exp["CameraSave"].is_string())
        {
            auto const& cs = exp["CameraSave"].get_ref<const std::string&>();
            ex.camera_save_ = cs.empty() ? json::object() : json::parse(cs, nullptr, true, true);
        }
        else
        {
            ex.camera_save_ = json::object();
        }
        if (ex.camera_save_.contains("VisionCenter") && ex.camera_save_["VisionCenter"].is_string())
        {
            ex.vision_center_ = detail::parse_xyz(ex.camera_save_["VisionCenter"].get_ref<const std::string&>());
        }
        if (ex.camera_save_.contains("TargetRotation") && ex.camera_save_["TargetRotation"].is_string())
        {
            ex.target_rotation_ = detail::parse_xyz(ex.camera_save_["TargetRotation"].get_ref<const std::string&>());
        }

        json status_save = json::object();
        if (exp.contains("StatusSave") && exp["StatusSave"].is_string())
        {
            auto const& ss = exp["StatusSave"].get_ref<const std::string&>();
            status_save = ss.empty() ? json::object() : json::parse(ss, nullptr, true, true);
        }

        if (ex.type_ == experiment_type::circuit || ex.type_ == experiment_type::electromagnetism)
        {
            if (status_save.contains("Elements") && status_save["Elements"].is_array())
            {
                for (auto const& el : status_save["Elements"])
                {
                    if (!el.is_object()) continue;
                    element e = element::generic(el, {0, 0, 0}, false, false);
                    if (auto p = e.try_read_native_position(); p)
                    {
                        e.set_element_position(*p, false);
                    }
                    static_cast<void>(ex.add_element(std::move(e)));
                }
            }
            if (ex.type_ == experiment_type::circuit && status_save.contains("Wires") && status_save["Wires"].is_array())
            {
                for (auto const& wj : status_save["Wires"])
                {
                    if (!wj.is_object()) continue;
                    wire w;
                    w.source.element_identifier = wj.value("Source", "");
                    w.source.pin = wj.value("SourcePin", 0);
                    w.target.element_identifier = wj.value("Target", "");
                    w.target.pin = wj.value("TargetPin", 0);
                    if (auto it = wj.find("ColorName"); it != wj.end() && it->is_string())
                    {
                        w.color = parse_wire_color_name(it->get_ref<const std::string&>());
                    }
                    ex.wires_.push_back(std::move(w));
                }
            }
        }
        else if (ex.type_ == experiment_type::celestial)
        {
            if (status_save.contains("Elements") && status_save["Elements"].is_object())
            {
                for (auto const& [id, el] : status_save["Elements"].items())
                {
                    if (!el.is_object()) continue;
                    element e = element::generic(el, {0, 0, 0}, false, false);
                    static_cast<void>(ex.add_element(std::move(e)));
                    (void)id;
                }
            }
        }

        return ex;
    }

    [[nodiscard]] experiment_type type() const noexcept { return type_; }

    void entitle(std::string const& sav_name)
    {
        plsav_["Summary"]["Subject"] = sav_name;
        plsav_["InternalName"] = sav_name;
    }

    [[nodiscard]] std::optional<std::filesystem::path> sav_path() const { return sav_path_; }

    void set_sav_path(std::filesystem::path path) { sav_path_ = std::move(path); }

    void set_element_xyz(bool enabled, position origin = {0.0, 0.0, 0.0})
    {
        element_xyz_enabled_ = enabled;
        element_xyz_origin_ = origin;
    }

    [[nodiscard]] bool element_xyz_enabled() const noexcept { return element_xyz_enabled_; }
    [[nodiscard]] position element_xyz_origin() const noexcept { return element_xyz_origin_; }

    void set_camera(position vision_center_native, position target_rotation_native)
    {
        vision_center_ = vision_center_native;
        target_rotation_ = target_rotation_native;
    }

    [[nodiscard]] std::string add_element(element e)
    {
        auto id = e.identifier();
        if (id_to_index_.contains(id))
        {
            throw std::runtime_error("duplicate element identifier: " + id);
        }
        id_to_index_.emplace(id, elements_.size());
        elements_.push_back(std::move(e));
        return id;
    }

    [[nodiscard]] std::string add_circuit_element(std::string model_id,
                                                  position pos,
                                                  std::optional<bool> element_xyz_coords = std::nullopt,
                                                  bool is_big_element = false,
                                                  bool participate_in_layout = true)
    {
        if (type_ != experiment_type::circuit)
        {
            throw std::runtime_error("add_circuit_element only valid for circuit experiments");
        }
        bool is_element_xyz_coords = element_xyz_coords.value_or(element_xyz_enabled_);
        element e = element::circuit(std::move(model_id), pos, is_element_xyz_coords, is_big_element, participate_in_layout);
        return add_element(std::move(e));
    }

    [[nodiscard]] element& get_element(std::string const& identifier)
    {
        auto it = id_to_index_.find(identifier);
        if (it == id_to_index_.end())
        {
            throw std::runtime_error("unknown element identifier: " + identifier);
        }
        return elements_.at(it->second);
    }

    [[nodiscard]] element const& get_element(std::string const& identifier) const
    {
        auto it = id_to_index_.find(identifier);
        if (it == id_to_index_.end())
        {
            throw std::runtime_error("unknown element identifier: " + identifier);
        }
        return elements_.at(it->second);
    }

    void connect(std::string const& src_id, int src_pin, std::string const& dst_id, int dst_pin, wire_color color = wire_color::blue)
    {
        if (type_ != experiment_type::circuit)
        {
            throw std::runtime_error("wires only supported for circuit experiments");
        }
        if (!id_to_index_.contains(src_id)) throw std::runtime_error("unknown source element: " + src_id);
        if (!id_to_index_.contains(dst_id)) throw std::runtime_error("unknown target element: " + dst_id);
        wires_.push_back(wire{{src_id, src_pin}, {dst_id, dst_pin}, color});
    }

    void clear_wires()
    {
        if (type_ != experiment_type::circuit)
        {
            throw std::runtime_error("wires only supported for circuit experiments");
        }
        wires_.clear();
    }

    [[nodiscard]] std::vector<element> const& elements() const noexcept { return elements_; }
    [[nodiscard]] std::vector<wire> const& wires() const noexcept { return wires_; }

    [[nodiscard]] json to_plsav_json() const
    {
        json out = plsav_;

        json status_save;
        if (type_ == experiment_type::circuit)
        {
            status_save = json{
                {"SimulationSpeed", 1.0},
                {"Elements", json::array()},
                {"Wires", json::array()},
            };
            for (auto const& e : elements_)
            {
                json el = e.data();
                auto element_pos = e.element_position();
                position native_pos = e.is_element_xyz() ? element_xyz::to_native(element_pos, element_xyz_origin_, e.is_big_element())
                                                         : element_pos;
                el["Position"] = detail::pack_xyz(native_pos);
                status_save["Elements"].push_back(std::move(el));
            }
            for (auto const& w : wires_)
            {
                status_save["Wires"].push_back(w.to_json());
            }
        }
        else if (type_ == experiment_type::electromagnetism)
        {
            status_save = json{
                {"SimulationSpeed", 1.0},
                {"Elements", json::array()},
            };
            for (auto const& e : elements_)
            {
                json el = e.data();
                auto element_pos = e.element_position();
                el["Position"] = detail::pack_xyz(element_pos);
                status_save["Elements"].push_back(std::move(el));
            }
        }
        else if (type_ == experiment_type::celestial)
        {
            status_save = json{
                {"MainIdentifier", nullptr},
                {"Elements", json::object()},
                {"WorldTime", 0.0},
                {"ScalingName", "内太阳系"},
                {"LengthScale", 1.0},
                {"SizeLinear", 0.0001},
                {"SizeNonlinear", 0.5},
                {"StarPresent", false},
                {"Setting", nullptr},
            };
            for (auto const& e : elements_)
            {
                status_save["Elements"][e.identifier()] = e.data();
            }
        }
        else
        {
            throw std::runtime_error("unsupported experiment type");
        }

        auto creation_ms = detail::now_ms();
        out["Experiment"]["CreationDate"] = creation_ms;
        out["Summary"]["CreationDate"] = creation_ms;

        json camera_save = camera_save_;
        camera_save["VisionCenter"] = detail::pack_xyz(vision_center_);
        camera_save["TargetRotation"] = detail::pack_xyz(target_rotation_);

        out["Experiment"]["CameraSave"] = camera_save.dump(-1, ' ', true);
        out["Experiment"]["StatusSave"] = status_save.dump(-1, ' ', true);

        return out;
    }

    [[nodiscard]] std::string dump(int indent = 2) const
    {
        return to_plsav_json().dump(indent, ' ', false);
    }

    void save(std::filesystem::path const& path, int indent = 2) const
    {
        std::ofstream ofs(path, std::ios::binary);
        if (!ofs.is_open())
        {
            throw std::runtime_error("failed to open output: " + path.string());
        }
        auto txt = dump(indent);
        ofs.write(txt.data(), static_cast<std::streamsize>(txt.size()));
    }

    void save(std::string const& path, int indent = 2) const { save(std::filesystem::path(path), indent); }

    void save(int indent = 2) const
    {
        if (!sav_path_)
        {
            throw std::runtime_error("experiment has no bound sav_path");
        }
        if (auto dir = sav_path_->parent_path(); !dir.empty())
        {
            detail::ensure_directory(dir);
        }
        save(*sav_path_, indent);
    }

    static experiment open(open_mode mode,
                           std::string const& arg,
                           experiment_type type_for_create = experiment_type::circuit,
                           bool force_create = false,
                           std::optional<std::filesystem::path> sav_dir = std::nullopt)
    {
        if (mode == open_mode::load_by_filepath)
        {
            auto ex = load(std::filesystem::path(arg));
            ex.set_sav_path(std::filesystem::path(arg));
            return ex;
        }
        if (mode == open_mode::load_by_sav_name)
        {
            auto dir = sav_dir.value_or(detail::default_sav_dir());
            auto found = find_sav_by_name(dir, arg);
            if (!found)
            {
                throw std::runtime_error("no such experiment: " + arg);
            }
            auto ex = load(*found);
            ex.set_sav_path(*found);
            return ex;
        }
        if (mode == open_mode::crt)
        {
            auto dir = sav_dir.value_or(detail::default_sav_dir());
            detail::ensure_directory(dir);
            auto out_path = dir / (arg + ".sav");
            if (!force_create && std::filesystem::exists(out_path))
            {
                throw std::runtime_error("experiment already exists: " + out_path.string());
            }
            auto ex = create(type_for_create);
            ex.entitle(arg);
            ex.set_sav_path(std::move(out_path));
            return ex;
        }

        throw std::runtime_error("open_mode.load_by_plar_app is not implemented in C++ wrapper yet");
    }

    [[nodiscard]] experiment& merge(experiment const& other, position offset = {0.0, 0.0, 0.0})
    {
        if (type_ != other.type_)
        {
            throw std::runtime_error("cannot merge experiments of different types");
        }

        std::unordered_map<std::string, std::string> id_map;
        id_map.reserve(other.elements_.size());

        for (auto const& e : other.elements_)
        {
            json d = e.data();
            auto old_id = d.value("Identifier", "");
            if (old_id.empty())
            {
                throw std::runtime_error("merge: element missing Identifier");
            }

            std::string new_id;
            do
            {
                new_id = detail::rand_string(33);
            } while (id_to_index_.contains(new_id) || id_map.contains(new_id));

            d["Identifier"] = new_id;

            auto pos = e.element_position();
            pos.x += offset.x;
            pos.y += offset.y;
            pos.z += offset.z;

            element ne = element::generic(std::move(d), pos, e.is_element_xyz(), e.is_big_element());
            ne.set_participate_in_layout(e.participate_in_layout());
            static_cast<void>(add_element(std::move(ne)));
            id_map.emplace(old_id, new_id);
        }

        if (type_ == experiment_type::circuit)
        {
            for (auto const& w : other.wires_)
            {
                auto it_s = id_map.find(w.source.element_identifier);
                auto it_t = id_map.find(w.target.element_identifier);
                if (it_s == id_map.end() || it_t == id_map.end())
                {
                    continue;
                }
                wires_.push_back(wire{{it_s->second, w.source.pin}, {it_t->second, w.target.pin}, w.color});
            }
        }

        return *this;
    }

private:
    static std::optional<std::filesystem::path> find_sav_by_name(std::filesystem::path const& dir, std::string const& sav_name)
    {
        std::error_code ec;
        if (!std::filesystem::exists(dir, ec))
        {
            return std::nullopt;
        }

        // 1) fast path: filename match
        auto direct = dir / sav_name;
        if (direct.extension() != ".sav")
        {
            auto with_ext = direct;
            with_ext += ".sav";
            if (std::filesystem::exists(with_ext, ec))
            {
                return with_ext;
            }
        }
        if (std::filesystem::exists(direct, ec))
        {
            return direct;
        }

        // 2) scan: InternalName match
        for (auto const& entry : std::filesystem::directory_iterator(dir, ec))
        {
            if (ec)
            {
                break;
            }
            if (!entry.is_regular_file())
            {
                continue;
            }
            if (entry.path().extension() != ".sav")
            {
                continue;
            }

            try
            {
                auto ex = load(entry.path());
                auto it = ex.plsav_.find("InternalName");
                if (it != ex.plsav_.end() && it->is_string() && it->get_ref<const std::string&>() == sav_name)
                {
                    return entry.path();
                }
            }
            catch (...)
            {
                continue;
            }
        }

        return std::nullopt;
    }

    experiment_type type_{experiment_type::circuit};
    json plsav_{json::object()};
    json camera_save_{json::object()};
    std::optional<std::filesystem::path> sav_path_;

    position vision_center_{0.0, 0.0, 0.0};
    position target_rotation_{0.0, 0.0, 0.0};

    bool element_xyz_enabled_{false};
    position element_xyz_origin_{0.0, 0.0, 0.0};

    std::vector<element> elements_;
    std::vector<wire> wires_;
    std::unordered_map<std::string, std::size_t> id_to_index_;
};

}  // namespace phy_engine::phy_lab_wrapper
