// RocketInputLoader.cpp
// JSON / CSV 파일에서 로켓 설정 로드
#include "RocketInputLoader.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

// ─────────────────────────────────────────────────────────────────────────────
//  내부 헬퍼
// ─────────────────────────────────────────────────────────────────────────────

std::string FRocketInputLoader::ReadFile(const std::string& filename, bool& ok)
{
    std::ifstream ifs(filename);
    if (!ifs.is_open()) { ok = false; return {}; }
    ok = true;
    return std::string(std::istreambuf_iterator<char>(ifs),
                       std::istreambuf_iterator<char>());
}

std::string FRocketInputLoader::Trim(const std::string& s)
{
    const auto notSpace = [](unsigned char c){ return !std::isspace(c); };
    auto b = std::find_if(s.begin(), s.end(), notSpace);
    auto e = std::find_if(s.rbegin(), s.rend(), notSpace).base();
    return (b < e) ? std::string(b, e) : std::string{};
}

std::string FRocketInputLoader::GetExt(const std::string& filename)
{
    auto pos = filename.rfind('.');
    if (pos == std::string::npos) return {};
    std::string ext = filename.substr(pos + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    return ext;
}

bool FRocketInputLoader::ParseDouble(const std::string& s, double& out)
{
    std::string t = Trim(s);
    if (t.empty()) return false;
    try { out = std::stod(t); return true; }
    catch (...) { return false; }
}

// ─────────────────────────────────────────────────────────────────────────────
//  FindJsonValue
//  JSON 문자열에서 "key" : <value> 를 찾아 value 부분 문자열을 반환
//  value 는 숫자, 문자열, 배열, 객체 중 하나
// ─────────────────────────────────────────────────────────────────────────────
bool FRocketInputLoader::FindJsonValue(const std::string& json,
                                        const std::string& key,
                                        std::string& outValue)
{
    std::string searchKey = "\"" + key + "\"";
    size_t pos = json.find(searchKey);
    if (pos == std::string::npos) return false;

    // ':' 까지 건너뜀
    pos += searchKey.size();
    while (pos < json.size() && json[pos] != ':') ++pos;
    if (pos >= json.size()) return false;
    ++pos; // ':' 건너뜀

    // 공백 건너뜀
    while (pos < json.size() && std::isspace((unsigned char)json[pos])) ++pos;
    if (pos >= json.size()) return false;

    char start = json[pos];
    if (start == '[' || start == '{') {
        // 매칭 괄호 찾기 (중첩 + 문자열 내 괄호 무시)
        char close = (start == '[') ? ']' : '}';
        int depth = 1;
        size_t end = pos + 1;
        while (end < json.size() && depth > 0) {
            char c = json[end];
            if (c == '"') {
                // 문자열 내부 건너뜀
                ++end;
                while (end < json.size() && json[end] != '"') {
                    if (json[end] == '\\') ++end; // escape
                    ++end;
                }
            } else if (c == start)  ++depth;
            else if (c == close)    --depth;
            ++end;
        }
        outValue = json.substr(pos, end - pos);
    } else if (start == '"') {
        // 문자열 값
        size_t end = pos + 1;
        while (end < json.size() && json[end] != '"') {
            if (json[end] == '\\') ++end;
            ++end;
        }
        outValue = json.substr(pos + 1, end - pos - 1); // 따옴표 제외
    } else {
        // 숫자, true, false, null
        size_t end = pos;
        while (end < json.size() &&
               json[end] != ',' && json[end] != '}' &&
               json[end] != ']' && json[end] != '\n' && json[end] != '\r')
            ++end;
        outValue = Trim(json.substr(pos, end - pos));
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  ParseArray2D
//  "[[t0, v0], [t1, v1], ...]" 형태 → vector<pair<double,double>>
// ─────────────────────────────────────────────────────────────────────────────
bool FRocketInputLoader::ParseArray2D(const std::string& arrayStr,
                                       std::vector<std::pair<double,double>>& out)
{
    out.clear();
    // 외부 '[' ']' 안의 내용 추출
    size_t outer_open = arrayStr.find('[');
    if (outer_open == std::string::npos) return false;
    size_t outer_close = arrayStr.rfind(']');
    if (outer_close == std::string::npos || outer_close <= outer_open) return false;

    std::string inner = arrayStr.substr(outer_open + 1, outer_close - outer_open - 1);

    // 각 [t, v] 쌍 파싱
    size_t pos = 0;
    while (pos < inner.size()) {
        // '[' 찾기
        size_t bOpen = inner.find('[', pos);
        if (bOpen == std::string::npos) break;

        size_t bClose = inner.find(']', bOpen);
        if (bClose == std::string::npos) return false;

        std::string pair = inner.substr(bOpen + 1, bClose - bOpen - 1);

        // 쉼표로 분리
        size_t comma = pair.find(',');
        if (comma == std::string::npos) return false;

        double t, v;
        if (!ParseDouble(pair.substr(0, comma), t)) return false;
        if (!ParseDouble(pair.substr(comma + 1),  v)) return false;

        out.push_back({t, v});
        pos = bClose + 1;
    }
    return !out.empty();
}

// ─────────────────────────────────────────────────────────────────────────────
//  ParseVector3
//  "[x, y, z]" 형태 → x, y, z
// ─────────────────────────────────────────────────────────────────────────────
bool FRocketInputLoader::ParseVector3(const std::string& arrayStr,
                                       double& x, double& y, double& z)
{
    size_t op = arrayStr.find('[');
    size_t cl = arrayStr.rfind(']');
    if (op == std::string::npos || cl == std::string::npos) return false;

    std::string inner = arrayStr.substr(op + 1, cl - op - 1);
    std::istringstream ss(inner);
    std::string tok;
    double vals[3] = {};
    int i = 0;
    while (std::getline(ss, tok, ',') && i < 3) {
        if (!ParseDouble(tok, vals[i])) return false;
        ++i;
    }
    if (i != 3) return false;
    x = vals[0]; y = vals[1]; z = vals[2];
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  FinalizeThrustCurve
//  ThrustCurve 로드 후 BurnTime / MaxThrust 자동 설정
// ─────────────────────────────────────────────────────────────────────────────
void FRocketInputLoader::FinalizeThrustCurve(FRocketConfig& cfg)
{
    if (cfg.ThrustCurve.empty()) return;

    // 시간 순 정렬
    std::sort(cfg.ThrustCurve.begin(), cfg.ThrustCurve.end(),
              [](const auto& a, const auto& b){ return a.first < b.first; });

    cfg.BurnTime  = cfg.ThrustCurve.back().first;
    cfg.MaxThrust = 0;
    for (auto& [t, f] : cfg.ThrustCurve)
        cfg.MaxThrust = std::max(cfg.MaxThrust, f);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Load  ─  확장자 자동 감지
// ─────────────────────────────────────────────────────────────────────────────
bool FRocketInputLoader::Load(const std::string& filename,
                               FRocketConfig& cfg, std::string& errMsg)
{
    std::string ext = GetExt(filename);
    if (ext == "json") return LoadFromJSON(filename, cfg, errMsg);
    if (ext == "csv")  return LoadThrustFromCSV(filename, cfg, errMsg);
    if (ext == "xlsx") {
        errMsg = "[RocketInputLoader] .xlsx 파일은 직접 읽을 수 없습니다.\n"
                 "  convert_thrust_xlsx.py 로 먼저 JSON 또는 CSV 로 변환하세요.\n"
                 "  예: python convert_thrust_xlsx.py input.xlsx output.json";
        return false;
    }
    errMsg = "[RocketInputLoader] 지원하지 않는 확장자: " + ext
           + " (.json 또는 .csv 사용)";
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  LoadFromJSON
// ─────────────────────────────────────────────────────────────────────────────
bool FRocketInputLoader::LoadFromJSON(const std::string& filename,
                                       FRocketConfig& cfg, std::string& errMsg)
{
    bool fileOk;
    std::string raw = ReadFile(filename, fileOk);
    if (!fileOk) { errMsg = "[JSON] cannot open: " + filename; return false; }

    // remove // comments (non-standard JSON but tolerant)
    std::string json;
    {
        std::istringstream ss(raw);
        std::string line;
        while (std::getline(ss, line)) {
            auto cpos = line.find("//");
            if (cpos != std::string::npos) line = line.substr(0, cpos);
            json += line + "\n";
        }
    }

    std::string sec, val;

    // rocket section
    if (FindJsonValue(json, "rocket", sec)) {
        double v;
        if (FindJsonValue(sec, "dry_mass_kg",   val) && ParseDouble(val, v)) cfg.DryMass    = v;
        if (FindJsonValue(sec, "prop_mass_kg",  val) && ParseDouble(val, v)) cfg.PropMass   = v;
        if (FindJsonValue(sec, "ref_area_m2",   val) && ParseDouble(val, v)) cfg.RefArea    = v;
        if (FindJsonValue(sec, "ref_length_m",  val) && ParseDouble(val, v)) cfg.RefLength  = v;
        if (FindJsonValue(sec, "Ixx",           val) && ParseDouble(val, v)) cfg.Ixx        = v;
        if (FindJsonValue(sec, "Iyy",           val) && ParseDouble(val, v)) cfg.Iyy        = v;
        if (FindJsonValue(sec, "Izz",           val) && ParseDouble(val, v)) cfg.Izz        = v;
        if (FindJsonValue(sec, "XCG_initial_m", val) && ParseDouble(val, v)) cfg.XCG_initial= v;
        if (FindJsonValue(sec, "XCG_final_m",   val) && ParseDouble(val, v)) cfg.XCG_final  = v;
        if (FindJsonValue(sec, "BurnTime",      val) && ParseDouble(val, v)) cfg.BurnTime   = v;
        if (FindJsonValue(sec, "MaxThrust",     val) && ParseDouble(val, v)) cfg.MaxThrust  = v;
        if (FindJsonValue(sec, "burn_time_s",   val) && ParseDouble(val, v)) cfg.BurnTime   = v;
        if (FindJsonValue(sec, "max_thrust_N",  val) && ParseDouble(val, v)) cfg.MaxThrust  = v;
        if (FindJsonValue(sec, "isp_s",         val) && ParseDouble(val, v)) cfg.IspSeconds = v;
    }
    // _source.isp_s as fallback (auto-generated SRM JSON)
    {
        std::string srcSec;
        if (FindJsonValue(json, "_source", srcSec)) {
            double v;
            if (FindJsonValue(srcSec, "isp_s", val) && ParseDouble(val, v))
                if (cfg.IspSeconds <= 0.0) cfg.IspSeconds = v;
        }
    }

    // geometry section
    if (FindJsonValue(json, "geometry", sec)) {
        double v;
        auto& G = cfg.Geometry;
        G.RefLength  = cfg.RefLength;
        if (FindJsonValue(sec, "body_length_m",    val) && ParseDouble(val, v)) G.BodyLength   = v;
        if (FindJsonValue(sec, "nose_length_m",    val) && ParseDouble(val, v)) G.NoseLength   = v;
        if (FindJsonValue(sec, "fin_count",        val) && ParseDouble(val, v)) G.FinCount     = v;
        if (FindJsonValue(sec, "fin_span_m",       val) && ParseDouble(val, v)) G.FinSpan      = v;
        if (FindJsonValue(sec, "fin_root_chord_m", val) && ParseDouble(val, v)) G.FinRootChord = v;
        if (FindJsonValue(sec, "fin_tip_chord_m",  val) && ParseDouble(val, v)) G.FinTipChord  = v;
        if (FindJsonValue(sec, "fin_sweep_m",      val) && ParseDouble(val, v)) G.FinSweep     = v;
        if (FindJsonValue(sec, "XCP_m",            val) && ParseDouble(val, v)) G.XCP          = v;
        if (FindJsonValue(sec, "XCG_initial_m",    val) && ParseDouble(val, v)) G.XCG_initial  = v;
        G.RefLength = cfg.RefLength;
    }

    // inertia_table (optional)
    if (FindJsonValue(json, "inertia_table", sec)) {
        std::string dataSec;
        if (FindJsonValue(sec, "data", dataSec)) {
            size_t cursor = 0;
            cfg.InertiaTable.clear();
            while (cursor < dataSec.size()) {
                size_t obOpen = dataSec.find('{', cursor);
                if (obOpen == std::string::npos) break;
                int depth = 1; size_t end = obOpen + 1;
                while (end < dataSec.size() && depth > 0) {
                    if (dataSec[end] == '{') ++depth;
                    else if (dataSec[end] == '}') --depth;
                    ++end;
                }
                std::string obj = dataSec.substr(obOpen, end - obOpen);
                double t_s = 0, Ixx_v = 0, Iyy_v = 0;
                if (FindJsonValue(obj, "t_s",        val) && ParseDouble(val, t_s)
                 && FindJsonValue(obj, "I_xx_kgm2",  val) && ParseDouble(val, Ixx_v)
                 && FindJsonValue(obj, "I_yy_kgm2",  val) && ParseDouble(val, Iyy_v))
                {
                    cfg.InertiaTable.emplace_back(t_s, Ixx_v, Iyy_v);
                }
                cursor = end;
            }
        }
        if (!cfg.InertiaTable.empty()) {
            if (cfg.Ixx <= 0.0) cfg.Ixx = std::get<1>(cfg.InertiaTable.front());
            if (cfg.Iyy <= 0.0) cfg.Iyy = std::get<2>(cfg.InertiaTable.front());
            if (cfg.Izz <= 0.0) cfg.Izz = cfg.Iyy;
        }
    }

    // thrust_curve
    if (FindJsonValue(json, "thrust_curve", val)) {
        std::vector<std::pair<double,double>> tc;
        if (!ParseArray2D(val, tc)) {
            errMsg = "[JSON] thrust_curve parse failed: " + filename;
            return false;
        }
        cfg.ThrustCurve = std::move(tc);
    }

    // pressure_curve (optional, diagnostic)
    if (FindJsonValue(json, "pressure_curve", val)) {
        std::vector<std::pair<double,double>> pc;
        if (ParseArray2D(val, pc) && pc.size() >= 2)
            cfg.PressureCurve = std::move(pc);
    }

    // environment section
    if (FindJsonValue(json, "environment", sec)) {
        std::string windDirStr;
        double windSpeed = 0.0;
        double wx = 0, wy = 0, wz = 0;
        bool hasDir = FindJsonValue(sec, "wind_direction", windDirStr)
                   && ParseVector3(windDirStr, wx, wy, wz);
        bool hasSpeed = FindJsonValue(sec, "wind_speed_mps", val)
                     && ParseDouble(val, windSpeed);
        if (hasDir && hasSpeed) {
            double mag = std::sqrt(wx*wx + wy*wy + wz*wz);
            if (mag > 1e-8) {
                cfg.WindVelocity = {wx/mag * windSpeed,
                                    wy/mag * windSpeed,
                                    wz/mag * windSpeed};
            }
        } else if (hasSpeed && !hasDir) {
            cfg.WindVelocity = {windSpeed, 0, 0};
        }
    }

    // simulation section
    if (FindJsonValue(json, "simulation", sec)) {
        double v;
        if (FindJsonValue(sec, "thrust_aligned_prob",     val) && ParseDouble(val, v)) cfg.ThrustAlignedProb    = v;
        if (FindJsonValue(sec, "thrust_max_misalign_deg", val) && ParseDouble(val, v)) cfg.ThrustMaxMisalignDeg = v;
        if (FindJsonValue(sec, "launch_rail_length_m",    val) && ParseDouble(val, v)) cfg.LaunchRailLength     = v;
        if (FindJsonValue(sec, "rk45_abs_tol",  val) && ParseDouble(val, v)) cfg.RK45_AbsTol   = v;
        if (FindJsonValue(sec, "rk45_rel_tol",  val) && ParseDouble(val, v)) cfg.RK45_RelTol   = v;
        if (FindJsonValue(sec, "rk45_init_step",val) && ParseDouble(val, v)) cfg.RK45_InitStep = v;
        if (FindJsonValue(sec, "rk45_max_step", val) && ParseDouble(val, v)) cfg.RK45_MaxStep  = v;
    }

    FinalizeThrustCurve(cfg);

    // ── Auto-fix zero/invalid geometry (SRM-only output, launch builder skipped) ─
    //   ref_area_m2 = 0  -> drag = 0  -> vacuum-like flight (2x altitude!)
    //   ref_length_m = 0 -> aero moment = 0 -> attitude dynamics disabled
    const double PI_ = 3.14159265358979323846;
    if (cfg.RefLength <= 0.0 || cfg.RefLength > 5.0)
    {
        cfg.RefLength = 0.054;
        errMsg += "[Loader][WARN] ref_length_m=0/invalid -> 0.054m fallback. "
                  "Run launch input builder (Step 02) for accurate values.\n";
    }
    if (cfg.RefArea <= 0.0)
    {
        cfg.RefArea = PI_ / 4.0 * cfg.RefLength * cfg.RefLength;
        errMsg += "[Loader][WARN] ref_area_m2=0 -> auto-computed from ref_length.\n";
    }
    if (cfg.Geometry.RefLength <= 0.0) cfg.Geometry.RefLength = cfg.RefLength;
    if (cfg.Geometry.BodyLength <= 0.0) cfg.Geometry.BodyLength = 0.6;
    if (cfg.Geometry.NoseLength <= 0.0) cfg.Geometry.NoseLength = 0.2;
    if (cfg.Geometry.XCP <= 0.0)        cfg.Geometry.XCP = 0.5;
    if (cfg.XCG_initial <= 0.0)         cfg.XCG_initial   = 0.4;
    if (cfg.XCG_final <= 0.0)           cfg.XCG_final     = cfg.XCG_initial;
    if (cfg.Geometry.XCG_initial <= 0.0)cfg.Geometry.XCG_initial = cfg.XCG_initial;
    if (cfg.Geometry.FinCount <= 0)     cfg.Geometry.FinCount = 4;
    if (cfg.Geometry.FinSpan <= 0.0)    cfg.Geometry.FinSpan = 0.06;
    if (cfg.Geometry.FinRootChord <= 0.0) cfg.Geometry.FinRootChord = 0.08;
    if (cfg.Geometry.FinTipChord <= 0.0)  cfg.Geometry.FinTipChord  = 0.04;
    double m_total_est = cfg.DryMass + cfg.PropMass;
    if (cfg.Ixx <= 0.0 && m_total_est > 0.0)
        cfg.Ixx = 0.5 * m_total_est * (cfg.RefLength*0.5) * (cfg.RefLength*0.5);
    if (cfg.Iyy <= 0.0 && m_total_est > 0.0)
        cfg.Iyy = m_total_est * (3.0*(cfg.RefLength*0.5)*(cfg.RefLength*0.5)
                                  + cfg.Geometry.BodyLength*cfg.Geometry.BodyLength) / 12.0;
    if (cfg.Izz <= 0.0) cfg.Izz = cfg.Iyy;

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  LoadThrustFromCSV
// ─────────────────────────────────────────────────────────────────────────────
bool FRocketInputLoader::LoadThrustFromCSV(const std::string& filename,
                                            FRocketConfig& cfg, std::string& errMsg)
{
    std::ifstream ifs(filename);
    if (!ifs.is_open()) { errMsg = "[CSV] cannot open: " + filename; return false; }

    cfg.ThrustCurve.clear();
    double windDirX = 1.0, windDirY = 0.0, windDirZ = 0.0, windSpeed = 0.0;
    bool hasWindDir = false, hasWindSpeed = false;

    std::string line;
    while (std::getline(ifs, line)) {
        line = Trim(line);
        if (line.empty()) continue;

        if (line[0] == '#') {
            std::string comment = Trim(line.substr(1));
            auto eq = comment.find('=');
            if (eq != std::string::npos) {
                std::string key = Trim(comment.substr(0, eq));
                std::string val = Trim(comment.substr(eq + 1));
                double v;
                if (ParseDouble(val, v)) {
                    if      (key == "dry_mass_kg")             cfg.DryMass            = v;
                    else if (key == "prop_mass_kg")            cfg.PropMass           = v;
                    else if (key == "wind_speed_mps")        { windSpeed = v; hasWindSpeed = true; }
                    else if (key == "wind_dir_x")            { windDirX = v; hasWindDir = true; }
                    else if (key == "wind_dir_y")              windDirY = v;
                    else if (key == "wind_dir_z")              windDirZ = v;
                    else if (key == "thrust_aligned_prob")     cfg.ThrustAlignedProb    = v;
                    else if (key == "thrust_max_misalign_deg") cfg.ThrustMaxMisalignDeg = v;
                }
            }
            continue;
        }

        if (!std::isdigit((unsigned char)line[0]) && line[0] != '-' && line[0] != '+') continue;

        auto comma = line.find(',');
        if (comma == std::string::npos) {
            comma = line.find('\t');
            if (comma == std::string::npos) continue;
        }

        double t, f;
        if (!ParseDouble(line.substr(0, comma), t)) continue;
        if (!ParseDouble(line.substr(comma + 1), f)) continue;
        cfg.ThrustCurve.push_back({t, f});
    }

    if (cfg.ThrustCurve.empty()) {
        errMsg = "[CSV] no valid data rows: " + filename;
        return false;
    }

    if (hasWindDir && hasWindSpeed) {
        double mag = std::sqrt(windDirX*windDirX + windDirY*windDirY + windDirZ*windDirZ);
        if (mag > 1e-8) {
            cfg.WindVelocity = {windDirX/mag * windSpeed,
                                windDirY/mag * windSpeed,
                                windDirZ/mag * windSpeed};
        }
    } else if (hasWindSpeed) {
        cfg.WindVelocity = {windSpeed, 0, 0};
    }

    FinalizeThrustCurve(cfg);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  SaveToJSON
// ─────────────────────────────────────────────────────────────────────────────
bool FRocketInputLoader::SaveToJSON(const std::string& filename,
                                     const FRocketConfig& cfg, std::string& errMsg)
{
    std::ofstream ofs(filename);
    if (!ofs.is_open()) { errMsg = "[SaveJSON] cannot write: " + filename; return false; }

    auto d = [](double v) -> std::string {
        std::ostringstream ss;
        ss << std::setprecision(10) << v;
        return ss.str();
    };

    ofs << "{\n";
    ofs << "  \"rocket\": {\n";
    ofs << "    \"dry_mass_kg\":   " << d(cfg.DryMass)     << ",\n";
    ofs << "    \"prop_mass_kg\":  " << d(cfg.PropMass)    << ",\n";
    ofs << "    \"ref_area_m2\":   " << d(cfg.RefArea)     << ",\n";
    ofs << "    \"ref_length_m\":  " << d(cfg.RefLength)   << ",\n";
    ofs << "    \"Ixx\":           " << d(cfg.Ixx)         << ",\n";
    ofs << "    \"Iyy\":           " << d(cfg.Iyy)         << ",\n";
    ofs << "    \"Izz\":           " << d(cfg.Izz)         << ",\n";
    ofs << "    \"XCG_initial_m\": " << d(cfg.XCG_initial) << ",\n";
    ofs << "    \"XCG_final_m\":   " << d(cfg.XCG_final)   << "\n";
    ofs << "  },\n";

    const auto& G = cfg.Geometry;
    ofs << "  \"geometry\": {\n";
    ofs << "    \"body_length_m\":    " << d(G.BodyLength)   << ",\n";
    ofs << "    \"nose_length_m\":    " << d(G.NoseLength)   << ",\n";
    ofs << "    \"fin_count\":        " << d(G.FinCount)     << ",\n";
    ofs << "    \"fin_span_m\":       " << d(G.FinSpan)      << ",\n";
    ofs << "    \"fin_root_chord_m\": " << d(G.FinRootChord) << ",\n";
    ofs << "    \"fin_tip_chord_m\":  " << d(G.FinTipChord)  << ",\n";
    ofs << "    \"XCP_m\":            " << d(G.XCP)          << "\n";
    ofs << "  },\n";

    ofs << "  \"thrust_curve\": [\n";
    for (size_t i = 0; i < cfg.ThrustCurve.size(); ++i) {
        ofs << "    [" << d(cfg.ThrustCurve[i].first) << ", "
                       << d(cfg.ThrustCurve[i].second) << "]";
        if (i + 1 < cfg.ThrustCurve.size()) ofs << ",";
        ofs << "\n";
    }
    ofs << "  ],\n";

    double wMag = cfg.WindVelocity.Norm();
    double wDx = 1.0, wDy = 0.0, wDz = 0.0;
    if (wMag > 1e-8) {
        wDx = cfg.WindVelocity.x / wMag;
        wDy = cfg.WindVelocity.y / wMag;
        wDz = cfg.WindVelocity.z / wMag;
    }
    ofs << "  \"environment\": {\n";
    ofs << "    \"wind_direction\": [" << d(wDx) << ", " << d(wDy) << ", " << d(wDz) << "],\n";
    ofs << "    \"wind_speed_mps\": " << d(wMag) << "\n";
    ofs << "  },\n";

    ofs << "  \"simulation\": {\n";
    ofs << "    \"thrust_aligned_prob\":     " << d(cfg.ThrustAlignedProb)    << ",\n";
    ofs << "    \"thrust_max_misalign_deg\": " << d(cfg.ThrustMaxMisalignDeg) << ",\n";
    ofs << "    \"rk45_abs_tol\":  " << d(cfg.RK45_AbsTol)   << ",\n";
    ofs << "    \"rk45_rel_tol\":  " << d(cfg.RK45_RelTol)   << ",\n";
    ofs << "    \"rk45_init_step\":" << d(cfg.RK45_InitStep) << ",\n";
    ofs << "    \"rk45_max_step\": " << d(cfg.RK45_MaxStep)  << "\n";
    ofs << "  }\n";
    ofs << "}\n";

    return true;
}
