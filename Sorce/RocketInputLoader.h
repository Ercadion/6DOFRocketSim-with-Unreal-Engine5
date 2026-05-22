// RocketInputLoader.h
// 파일에서 로켓 설정(질량, 추력 곡선 등)을 불러오는 유틸리티
//
// 지원 형식
//   .json  ── 전체 설정 (질량 + 추력 + 환경 + 시뮬레이션 파라미터)
//   .csv   ── 추력 곡선만 (2열: time_s, thrust_N)
//             헤더 주석(# key=value)으로 dry_mass / prop_mass 지정 가능
//
// Excel(.xlsx) 변환
//   convert_thrust_xlsx.py 로 → JSON 또는 CSV 로 변환 후 위 형식으로 로드
//
// JSON 예시 → example_thrust_config.json 참고
//
// 사용 예:
//   FRocketConfig cfg = MakeDefaultConfig();   // 기본값 세팅
//   std::string err;
//   if (!FRocketInputLoader::Load("my_rocket.json", cfg, err))
//       std::cerr << "Load error: " << err << "\n";
//   // cfg.WindVelocity 등 추가 필드는 파일 로드 후 코드에서 덮어쓸 수 있음
#pragma once
#include "RocketSim6DOF.h"
#include <string>

class FRocketInputLoader {
public:
    // 파일 확장자에 따라 자동 디스패치
    //   .json → LoadFromJSON
    //   .csv  → LoadThrustFromCSV
    //   .xlsx → 오류 메시지 반환 (convert_thrust_xlsx.py 사용 안내)
    // 반환: 성공이면 true, 실패면 false + errMsg 채움
    static bool Load(const std::string& filename, FRocketConfig& cfg, std::string& errMsg);

    // JSON 파일 전체 로드
    static bool LoadFromJSON(const std::string& filename, FRocketConfig& cfg, std::string& errMsg);

    // CSV 파일에서 추력 곡선 로드
    // 첫 번째 열: 시간[s], 두 번째 열: 추력[N]
    // 헤더 주석:  # dry_mass_kg=0.420
    //             # prop_mass_kg=0.095
    // 해당 주석이 있으면 cfg.DryMass, cfg.PropMass 도 설정
    static bool LoadThrustFromCSV(const std::string& filename, FRocketConfig& cfg, std::string& errMsg);

    // 현재 cfg를 JSON으로 저장 (검수/공유용)
    static bool SaveToJSON(const std::string& filename, const FRocketConfig& cfg, std::string& errMsg);

private:
    // ── JSON 파싱 헬퍼 ───────────────────────────────────────────────────
    static std::string  ReadFile(const std::string& filename, bool& ok);
    static std::string  Trim    (const std::string& s);
    static bool         FindJsonValue(const std::string& json, const std::string& key,
                                      std::string& outValue);
    static bool         ParseDouble  (const std::string& s, double& out);
    static bool         ParseArray2D (const std::string& arrayStr,
                                      std::vector<std::pair<double,double>>& out);
    static bool         ParseVector3 (const std::string& arrayStr,
                                      double& x, double& y, double& z);

    // ── 공통 후처리 ─────────────────────────────────────────────────────
    static void FinalizeThrustCurve(FRocketConfig& cfg);

    // 파일 확장자 소문자 반환
    static std::string GetExt(const std::string& filename);
};
