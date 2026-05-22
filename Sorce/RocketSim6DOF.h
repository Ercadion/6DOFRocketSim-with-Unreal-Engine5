// RocketSim6DOF.h
// 6DOF 로켓 시뮬레이터
//   적분기     : RK45 Dormand-Prince (적응형 내부 서브스텝)
//   추력 보간  : 자연 3차 스플라인 (Natural Cubic Spline)
//   추력 편향  : 확률 기반 랜덤 노즐 미스얼라인먼트 (cone sampling)
//   풍향/풍속  : 겉보기 속도(V_rocket - V_wind) 기반 공력 계산
//   입력 방식  : JSON / CSV 파일 또는 코드 직접 설정
//   출력      : Position / Velocity / Acceleration + 자세 쿼터니언 → FSimSnapshot
#pragma once
#include "RocketVectorDefinition.h"
#include "AtmosphereModel.h"
#include "AdvancedAeroCoeffModel.h"
#include <vector>
#include <random>
#include <cmath>
#include <stdexcept>
#include <string>
#include <tuple>
#include <initializer_list>
#include <utility>

// ═════════════════════════════════════════════════════════════════════════════
//  자연 3차 스플라인 보간기 (변경 없음)
// ═════════════════════════════════════════════════════════════════════════════
class FCubicSpline {
public:
    FCubicSpline() = default;
    void   Build(const std::vector<std::pair<double,double>>& pts);
    double Eval (double x) const;
    bool   IsBuilt() const { return !xs_.empty(); }
private:
    std::vector<double> xs_, a_, b_, c_, d_;
};

// ═════════════════════════════════════════════════════════════════════════════
//  로켓 설정 구조체
//  ─ 코드 직접 설정 또는 RocketInputLoader::Load() 로 파일에서 채울 수 있음
// ═════════════════════════════════════════════════════════════════════════════
struct FRocketConfig {
    // ── 기체 물리 ──────────────────────────────────────────────────────────
    double RefArea   = 0.0;   // 기준 면적 [m²]
    double RefLength = 0.0;   // 기준 길이 = 직경 [m]
    double Ixx = 0.0, Iyy = 0.0, Izz = 0.0; // 관성 모멘트 [kg·m²] (초기, fallback)
    double BurnTime   = 0.0;  // 연소 시간 [s]  (ThrustCurve 마지막 시점으로 자동 설정)
    double MaxThrust  = 0.0;  // 최대 추력 (참고) [N]
    double DryMass    = 0.0;  // 건조 질량 [kg]
    double PropMass   = 0.0;  // 추진제 초기 질량 [kg]

    // ── 시변 관성 테이블 (선택적) ─────────────────────────────────────────
    // inertia_table.json 의 (t, Ixx, Iyy) 시계열 → 추진제 소모에 따른 관성 변화
    // 비어 있으면 위 Ixx/Iyy/Izz 고정값 사용
    // Iyy = Izz (축대칭 가정)
    std::vector<std::tuple<double,double,double>> InertiaTable; // (t, Ixx, Iyy)

    // ── 추력 곡선 ──────────────────────────────────────────────────────────
    // (t[s], F[N]) 쌍; JSON / CSV 로드 또는 직접 채움
    // BurnTime 은 ThrustCurve.back().first 로 자동 설정됨
    std::vector<std::pair<double,double>> ThrustCurve;

    // ── 압력 곡선 (TMS 측정 연소실 압력) ──────────────────────────────────
    // (t[s], P[bar]) 쌍; parse_tms_data.py 로 생성된 JSON 의 "pressure_curve"
    // 추력 계산에 직접 사용되지 않고 진단/시각화용으로 저장됨.
    // 파일에 pressure_curve 필드가 없으면 비어있음
    std::vector<std::pair<double,double>> PressureCurve;

    // ── 공력 형상 ──────────────────────────────────────────────────────────
    FAeroModel::FRocketGeometry Geometry;

    // ── 무게중심 이동 ──────────────────────────────────────────────────────
    double XCG_initial = 0.0; // 발사 직후 CG (노즈 기준) [m]
    double XCG_final   = 0.0; // 연소 완료 시 CG (노즈 기준) [m]

    // ── 추진제 비추력 Isp [s] ─────────────────────────────────────────────
    // mass flow = -Thrust(t) / (Isp · g0)  (Tsiolkovsky 정확 모델)
    // 0 또는 음수 → fallback: dM/dt = -PropMass/BurnTime (평균 추력 가정)
    double IspSeconds = 0.0;

    // ── 풍향 / 풍속 ────────────────────────────────────────────────────────
    // WindVelocity = WindDirection(단위벡터) × WindSpeed
    // 예) 동쪽 5m/s 바람: WindVelocity = {5, 0, 0}
    // 겉보기 속도 V_app = V_rocket - WindVelocity 로 공력 계산
    FVector3 WindVelocity = {0, 0, 0}; // World 좌표 [m/s]

    // ── 추력 방향 오차 (TVC 불완전성 모델) ────────────────────────────────
    // 실제 노즐은 항상 완벽히 축 방향으로 분사되지 않음.
    // 매 시뮬레이션 프레임(Step 호출)마다 다음 규칙으로 추력 방향 결정:
    //   - ThrustAlignedProb  확률로 → Body +X 방향 (정상)
    //   - (1-ThrustAlignedProb) 확률로 → 최대 ThrustMaxMisalignDeg° 이내 랜덤 편향
    // 편향 분포: Body +X 축 중심 구면 콘(Cone) 내 균일 분포
    double ThrustAlignedProb    = 1.0;  // 정상 추력 확률 (예: 0.95 = 5% 편향)
    double ThrustMaxMisalignDeg = 45.0; // 최대 편향각 [deg]

    // ── 발사 레일 구속 ─────────────────────────────────────────────────────
    // 로켓이 발사대(레일)를 이탈하기 전까지 자세·각속도를 수직으로 고정.
    // 실제 발사대는 레일 끝까지 로켓 방향을 잡아주며 그 전에는 옆으로 기울지 않음.
    // LaunchRailLength [m]: 레일 길이 (로켓 기저부가 이 고도를 넘으면 자유 비행 전환)
    //   0.0 → 레일 없음 (기존 동작 유지; 바람이 세면 즉시 기움)
    //   5.0 → 지상 5m 상승 후 자유 비행 (권장 기본값)
    double LaunchRailLength = 5.0;   // [m]

    // ── RK45 적응형 스텝 제어 ──────────────────────────────────────────────
    double RK45_AbsTol   = 1.0e-6;
    double RK45_RelTol   = 1.0e-6;
    double RK45_InitStep = 0.005;
    double RK45_MaxStep  = 0.05;
    double RK45_MinStep  = 1.0e-8;
};

// ═════════════════════════════════════════════════════════════════════════════
//  FRocketSim6DOF  ─  6자유도 로켓 시뮬레이터
//
//  [사용 예시 - 파일 입력]
//    FRocketConfig cfg;
//    std::string err;
//    FRocketInputLoader::Load("rocket.json", cfg, err);   // JSON / CSV
//    cfg.WindVelocity = {5.0, 0, 0};          // 동풍 5 m/s 추가 설정
//    FRocketSim6DOF sim(cfg);
//    sim.SetSeed(42);                          // 재현 가능 시뮬레이션
//    sim.Reset(initState);
//    while (!done) {
//        sim.Step(deltaTime);
//        FSimSnapshot snap = sim.GetSnapshot();
//        // snap.PosX/Y/Z, VelX/Y/Z, AccX/Y/Z → Unreal SetActorLocation
//    }
// ═════════════════════════════════════════════════════════════════════════════
class FRocketSim6DOF {
public:
    explicit FRocketSim6DOF(const FRocketConfig& InCfg);

    void Reset(const FRocketState& InitState);

    // dt [s] 동안 RK45 적응형 서브스텝 적분
    // 매 호출 시 추력 편향 방향이 확률적으로 결정됨
    void Step(double dt);

    // 시드 설정 (재현 가능한 시뮬레이션용)
    void SetSeed(uint32_t seed) { Rng.seed(seed); }

    const FRocketState& GetState()  const { return State; }
    const FVector3&     GetAccel()  const { return LastAccel; }
    const FVector3&     GetCurrentThrustDir() const { return CurrentThrustDir; }
    FSimSnapshot        GetSnapshot() const;

    // 현재 스텝의 추력 편향 여부 (디버그용)
    bool IsCurrentStepMisaligned() const { return bLastStepMisaligned; }

private:
    FRocketConfig Cfg;
    FRocketState  State;
    FCubicSpline  ThrustSpline;
    FCubicSpline  PressureSpline; // TMS 측정 압력 (있을 때만 Built 상태)

    // 발사 레일: Reset() 시 초기 자세 저장 → 레일 구간 동안 강제 고정
    // bHasLeftRail: 한 번이라도 LaunchRailLength 를 넘어선 적이 있으면 true.
    //   → 이후 하강 중 z<레일 길이 가 되어도 자세 구속 재발동을 막는다.
    FQuaternion   InitialAttitude{ 1, 0, 0, 0 }; // Reset()에서 복사됨
    bool          bHasLeftRail = false;

    // 추력 방향 오차 관련
    std::mt19937 Rng;                   // 난수 생성기
    FVector3     CurrentThrustDir{1, 0, 0}; // 현재 스텝 추력 방향 (Body 좌표)
    bool         bLastStepMisaligned = false;

    // Unreal 출력 캐시
    FVector3 LastAccel  {};
    double   LastThrust = 0.0;
    double   LastDrag   = 0.0;
    double   LastMach   = 0.0;
    double   LastDynQ   = 0.0;

    // ── 상태 미분 ────────────────────────────────────────────────────────
    struct FDerivative {
        FVector3    dPos;
        FVector3    dVel;
        FQuaternion dAtt;
        FVector3    dOmega;
        double      dMass = 0;
    };

    FDerivative  ComputeDerivative(const FRocketState& S, double t);
    FRocketState ApplyDerivative(const FRocketState& S, const FDerivative& D, double h) const;
    FRocketState BlendState(const FRocketState& S, double h,
        std::initializer_list<std::pair<double,const FDerivative*>> terms) const;

    // ── RK45 ─────────────────────────────────────────────────────────────
    struct FRK45Result { FRocketState y5, y4; };
    FRK45Result RK45Step(const FRocketState& S, double t, double h);
    double ComputeErrorNorm(const FRocketState& y5, const FRocketState& y4,
                             const FRocketState& yOld) const;

    // ── 보조 ─────────────────────────────────────────────────────────────
    double   GetThrust(double t) const;
    double   GetCurrentXCG(const FRocketState& S) const;
    FVector3 ComputeThrustDir();  // 매 Step 시작 시 호출; 랜덤 편향 적용

    // 시변 관성 조회 — InertiaTable 이 비었으면 Cfg.Ixx/Iyy 반환
    // 선형 보간; 범위 밖이면 클램프
    void GetCurrentInertia(double t, double& outIxx,
                           double& outIyy, double& outIzz) const;

public:
    // TMS 측정 연소실 압력 조회 [bar]
    // PressureCurve 가 없으면 0 반환; t 범위 밖이면 클램프
    double GetChamberPressure(double t) const;

private:

    static FVector3 RotateVectorByQuat   (const FVector3& V, const FQuaternion& Q);
    static FVector3 RotateVectorByQuatInv(const FVector3& V, const FQuaternion& Q);

    // ── Dormand-Prince 계수 ───────────────────────────────────────────────
    static constexpr double DP_c2 = 1.0/5.0, DP_c3 = 3.0/10.0, DP_c4 = 4.0/5.0, DP_c5 = 8.0/9.0;
    static constexpr double DP_a21 = 1.0/5.0;
    static constexpr double DP_a31 = 3.0/40.0,       DP_a32 = 9.0/40.0;
    static constexpr double DP_a41 = 44.0/45.0,      DP_a42 = -56.0/15.0,      DP_a43 = 32.0/9.0;
    static constexpr double DP_a51 = 19372.0/6561.0, DP_a52 =-25360.0/2187.0, DP_a53 = 64448.0/6561.0, DP_a54 = -212.0/729.0;
    static constexpr double DP_a61 = 9017.0/3168.0,  DP_a62 = -355.0/33.0,    DP_a63 = 46732.0/5247.0,  DP_a64 = 49.0/176.0,  DP_a65 =-5103.0/18656.0;
    static constexpr double DP_b1 = 35.0/384.0,      DP_b3 = 500.0/1113.0, DP_b4 = 125.0/192.0, DP_b5 =-2187.0/6784.0, DP_b6 = 11.0/84.0;
    static constexpr double DP_b4star1 = 5179.0/57600.0, DP_b4star3 = 7571.0/16695.0, DP_b4star4 = 393.0/640.0,
                            DP_b4star5 =-92097.0/339200.0, DP_b4star6 = 187.0/2100.0,  DP_b4star7 = 1.0/40.0;
};
