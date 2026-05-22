// RocketSim6DOF.cpp
// 6DOF 로켓 시뮬레이터 구현
//   적분기  : RK45 Dormand-Prince (Hairer et al. "DOPRI5")
//   추력보간 : 자연 3차 스플라인 (Thomas 알고리즘으로 3대각 행렬 O(n) 풀이)
#include "RocketSim6DOF.h"
#include <cmath>
#include <algorithm>
#include <cassert>
#include <stdexcept>

// ─────────────────────────────────────────────────────────────────────────────
//  상수
// ─────────────────────────────────────────────────────────────────────────────
static constexpr double GRAVITY     = 9.80665;   // [m/s²]
static constexpr double RK45_SAFETY = 0.9;       // 스텝 제어 안전 계수
static constexpr double RK45_MAXFAC = 5.0;       // 스텝 최대 확대 비율
static constexpr double RK45_MINFAC = 0.2;       // 스텝 최소 축소 비율
static constexpr double QUAT_RENORM_THRESH = 1e-8; // 쿼터니언 재정규화 임계치
static constexpr double MY_PI       = 3.14159265358979323846; // MY_PI 대체 (MSVC 호환)

// ═════════════════════════════════════════════════════════════════════════════
//  FCubicSpline  ─  자연 3차 스플라인
// ═════════════════════════════════════════════════════════════════════════════

/*
 * 3차 스플라인 이론 요약
 * ─────────────────────────────────────────────────────────────────────────
 * n개 구간(n+1 개 노트)에서 각 구간 i의 스플라인:
 *   S_i(x) = a_i + b_i·dx + c_i·dx² + d_i·dx³   (dx = x - x_i)
 *
 * 두 번째 미분 m_i = S''(x_i) 를 구하면 나머지 계수가 결정됨.
 * m_i는 3대각 선형계:
 *   h[i-1]·m[i-1] + 2(h[i-1]+h[i])·m[i] + h[i]·m[i+1] = r[i]
 *   r[i] = 6·[(y[i+1]-y[i])/h[i] - (y[i]-y[i-1])/h[i-1]]
 * 자연 경계 조건: m[0] = m[n] = 0
 * → Thomas 알고리즘(전진소거+후진대입)으로 O(n) 풀이
 *
 * 계수 공식:
 *   a_i = y_i
 *   b_i = (y[i+1]-y[i])/h[i] - h[i]·(2m[i]+m[i+1])/6
 *   c_i = m[i]/2
 *   d_i = (m[i+1]-m[i]) / (6·h[i])
 */
void FCubicSpline::Build(const std::vector<std::pair<double, double>>& pts)
{
    int N = static_cast<int>(pts.size());
    if (N < 2)
        throw std::invalid_argument("FCubicSpline::Build: at least 2 data points required");

    int n = N - 1; // 구간 수

    xs_.resize(N);
    std::vector<double> y(N);
    for (int i = 0; i < N; ++i) {
        xs_[i] = pts[i].first;
        y[i]   = pts[i].second;
    }

    // 구간 폭 h[i] = x[i+1] - x[i]
    std::vector<double> h(n);
    for (int i = 0; i < n; ++i) {
        h[i] = xs_[i + 1] - xs_[i];
        if (h[i] <= 0.0)
            throw std::invalid_argument("FCubicSpline::Build: x values must be strictly increasing");
    }

    // 두 번째 미분 m[0..n], 자연 경계조건으로 m[0]=m[n]=0
    std::vector<double> m(N, 0.0);

    int M = n - 1; // 내부 미지수 개수 (m[1]..m[n-1])
    if (M >= 1) {
        // 3대각 행렬 설정 (내부 노트 j=0..M-1 ↔ 노트 k=1..n-1)
        //   diag[j] = 2*(h[j]+h[j+1])
        //   band[j] = h[j+1]   (상·하 대각 동일: 대칭 행렬)
        //   rhs[j]  = 6*[(y[j+2]-y[j+1])/h[j+1] - (y[j+1]-y[j])/h[j]]
        std::vector<double> diag(M), band(M - 1), rhs(M);
        for (int j = 0; j < M; ++j) {
            diag[j] = 2.0 * (h[j] + h[j + 1]);
            rhs[j]  = 6.0 * ((y[j + 2] - y[j + 1]) / h[j + 1]
                            - (y[j + 1] - y[j])     / h[j]);
        }
        for (int j = 0; j < M - 1; ++j)
            band[j] = h[j + 1]; // 상·하 대각 공통값

        // Thomas 알고리즘 전진 소거
        //   cp[j] = 수정된 상대각 / 현재 대각
        //   dp[j] = 수정된 RHS / 현재 대각
        std::vector<double> cp(M, 0.0), dp(M);
        cp[0] = (M > 1) ? band[0] / diag[0] : 0.0;
        dp[0] = rhs[0] / diag[0];
        for (int j = 1; j < M; ++j) {
            double lo    = band[j - 1];            // 현재 행의 하대각 = h[j]
            double denom = diag[j] - lo * cp[j - 1];
            cp[j] = (j < M - 1) ? band[j] / denom : 0.0;
            dp[j] = (rhs[j] - lo * dp[j - 1]) / denom;
        }

        // 후진 대입: u[j] = m[j+1]
        m[n - 1] = dp[M - 1]; // u[M-1] = m[n-1]
        for (int j = M - 2; j >= 0; --j)
            m[j + 1] = dp[j] - cp[j] * m[j + 2];
        // m[0] = m[n] = 0 유지
    }

    // 스플라인 계수 계산
    a_.resize(N); b_.resize(n); c_.resize(n); d_.resize(n);
    for (int i = 0; i < N; ++i) a_[i] = y[i];
    for (int i = 0; i < n; ++i) {
        c_[i] = m[i] / 2.0;
        d_[i] = (m[i + 1] - m[i]) / (6.0 * h[i]);
        b_[i] = (y[i + 1] - y[i]) / h[i]
                - h[i] * (2.0 * m[i] + m[i + 1]) / 6.0;
    }
}

double FCubicSpline::Eval(double x) const
{
    if (xs_.empty()) return 0.0;

    const int N = static_cast<int>(xs_.size());
    const int n = N - 1;

    // 범위 밖 클램프
    if (x <= xs_.front()) return a_.front();
    if (x >= xs_.back())  return a_.back();

    // 이진 탐색으로 구간 인덱스 결정
    int lo = 0, hi = n - 1;
    while (lo < hi) {
        int mid = (lo + hi + 1) / 2;
        if (xs_[mid] <= x) lo = mid;
        else                hi = mid - 1;
    }
    double dx = x - xs_[lo];
    // Horner 방법: a + dx*(b + dx*(c + dx*d))
    return a_[lo] + dx * (b_[lo] + dx * (c_[lo] + dx * d_[lo]));
}

// ═════════════════════════════════════════════════════════════════════════════
//  FRocketSim6DOF 생성자 & Reset
// ═════════════════════════════════════════════════════════════════════════════
FRocketSim6DOF::FRocketSim6DOF(const FRocketConfig& InCfg) : Cfg(InCfg)
{
    // 추력 곡선으로 스플라인 구축
    if (!Cfg.ThrustCurve.empty())
        ThrustSpline.Build(Cfg.ThrustCurve);

    // 압력 곡선 스플라인 구축 (데이터가 2포인트 이상일 때만)
    if (Cfg.PressureCurve.size() >= 2)
        PressureSpline.Build(Cfg.PressureCurve);

    FRocketState initState;
    initState.Position        = { 0, 0, 0 };
    initState.Velocity        = { 0, 0, 0 };
    initState.Attitude        = FQuaternion::Identity();
    initState.AngularVelocity = { 0, 0, 0 };
    initState.Mass            = Cfg.DryMass + Cfg.PropMass;
    initState.Time            = 0.0;
    Reset(initState);
}

void FRocketSim6DOF::Reset(const FRocketState& InitState)
{
    State           = InitState;
    InitialAttitude = InitState.Attitude;   // 레일 구속에서 이 자세로 복원
    bHasLeftRail    = false;                // 재발사 시 레일 단계 다시 시작
    LastAccel   = { 0, 0, 0 };
    LastThrust  = 0.0;
    LastDrag    = 0.0;
    LastMach    = 0.0;
    LastDynQ    = 0.0;
}

// ═════════════════════════════════════════════════════════════════════════════
//  추력 보간 (3차 스플라인)
// ═════════════════════════════════════════════════════════════════════════════
double FRocketSim6DOF::GetThrust(double t) const
{
    if (t < 0.0 || t > Cfg.BurnTime) return 0.0;
    if (!ThrustSpline.IsBuilt())      return 0.0;

    double T = ThrustSpline.Eval(t);
    return std::max(0.0, T); // 물리적으로 음수 추력 방지
}

// ═════════════════════════════════════════════════════════════════════════════
//  압력 보간 (TMS 측정 연소실 압력)
// ═════════════════════════════════════════════════════════════════════════════
double FRocketSim6DOF::GetChamberPressure(double t) const
{
    if (!PressureSpline.IsBuilt()) return 0.0;
    if (t < 0.0 || t > Cfg.BurnTime) return 0.0;
    return PressureSpline.Eval(t); // [bar]
}

// ═════════════════════════════════════════════════════════════════════════════
//  CG 위치 계산 (추진제 소모에 따른 선형 이동)
// ═════════════════════════════════════════════════════════════════════════════
double FRocketSim6DOF::GetCurrentXCG(const FRocketState& S) const
{
    // 추진제 잔여 비율 (0 ~ 1)
    double propFrac = (Cfg.PropMass > 0.0)
                    ? std::clamp((S.Mass - Cfg.DryMass) / Cfg.PropMass, 0.0, 1.0)
                    : 0.0;
    // 연소 중: XCG_initial → XCG_final 선형 이동
    return Cfg.XCG_initial * propFrac + Cfg.XCG_final * (1.0 - propFrac);
}

// ═════════════════════════════════════════════════════════════════════════════
//  시변 관성 모멘트 조회
//  InertiaTable 이 비었으면 Cfg.Ixx/Iyy/Izz 고정값 반환.
//  채워져 있으면 시간 t 에 대해 선형 보간 (Iyy = Izz 가정).
// ═════════════════════════════════════════════════════════════════════════════
void FRocketSim6DOF::GetCurrentInertia(double t, double& outIxx,
                                        double& outIyy, double& outIzz) const
{
    if (Cfg.InertiaTable.empty())
    {
        outIxx = Cfg.Ixx;
        outIyy = Cfg.Iyy;
        outIzz = Cfg.Izz;
        return;
    }

    const auto& tbl = Cfg.InertiaTable;
    // 클램프
    if (t <= std::get<0>(tbl.front()))
    {
        outIxx = std::get<1>(tbl.front());
        outIyy = std::get<2>(tbl.front());
        outIzz = outIyy;
        return;
    }
    if (t >= std::get<0>(tbl.back()))
    {
        outIxx = std::get<1>(tbl.back());
        outIyy = std::get<2>(tbl.back());
        outIzz = outIyy;
        return;
    }

    // 이진 탐색
    int lo = 0, hi = (int)tbl.size() - 1;
    while (lo + 1 < hi)
    {
        int mid = (lo + hi) / 2;
        if (std::get<0>(tbl[mid]) <= t) lo = mid;
        else                              hi = mid;
    }
    double t0 = std::get<0>(tbl[lo]), t1 = std::get<0>(tbl[lo + 1]);
    double a  = (t - t0) / std::max(t1 - t0, 1e-12);
    outIxx = std::get<1>(tbl[lo]) * (1.0 - a) + std::get<1>(tbl[lo + 1]) * a;
    outIyy = std::get<2>(tbl[lo]) * (1.0 - a) + std::get<2>(tbl[lo + 1]) * a;
    outIzz = outIyy;  // 축대칭 가정
}

// ═════════════════════════════════════════════════════════════════════════════
//  ComputeThrustDir  ─  추력 방향 결정 (확률 기반 노즐 미스얼라인먼트)
//
//  매 Step() 시작 시 한 번 호출하여 CurrentThrustDir을 설정.
//  Body 좌표계 기준:
//    - P(정상) = ThrustAlignedProb  → Body +X 방향 반환
//    - P(편향) = 1 - ThrustAlignedProb → 구면 콘(Spherical Cap) 내 균일 분포
//
//  콘 샘플링:
//    cos(θ) = 1 - u*(1-cos(θ_max))   (u ~ Uniform[0,1])
//    φ      = 2π * v               (v ~ Uniform[0,1])
//    → (sinθ·cosφ, sinθ·sinφ, cosθ) 를 Body X→ 정렬로 회전
//    Body +X 중심 콘: Rx(-90°) 좌표계 → x=cosθ, y=sinθ·cosφ, z=sinθ·sinφ
// ═════════════════════════════════════════════════════════════════════════════
FVector3 FRocketSim6DOF::ComputeThrustDir()
{
    // 정상 추력 확률 판정
    std::uniform_real_distribution<double> dist01(0.0, 1.0);
    double r = dist01(Rng);

    if (r < Cfg.ThrustAlignedProb) {
        // 정상: Body +X 방향 (노즐 완전 정렬)
        bLastStepMisaligned = false;
        return { 1.0, 0.0, 0.0 };
    }

    // 편향: 콘 내 균일 구면 분포
    bLastStepMisaligned = true;

    double thetaMax = Cfg.ThrustMaxMisalignDeg * (MY_PI / 180.0);
    double cosMax   = std::cos(thetaMax);

    // cos(θ)를 [cosMax, 1] 구간에서 균일 샘플 → 구면 균일 분포 보장
    double cosTheta = cosMax + (1.0 - cosMax) * dist01(Rng);
    double sinTheta = std::sqrt(std::max(0.0, 1.0 - cosTheta * cosTheta));
    double phi      = 2.0 * MY_PI * dist01(Rng);

    // Body 좌표계: +X = 추력 기준 축
    //   방위각 φ는 Y-Z 평면에서 회전
    return {
        cosTheta,
        sinTheta * std::cos(phi),
        sinTheta * std::sin(phi)
    };
}

// ═════════════════════════════════════════════════════════════════════════════
//  ComputeDerivative  ─  6DOF 물리 모델
// ═════════════════════════════════════════════════════════════════════════════
FRocketSim6DOF::FDerivative
FRocketSim6DOF::ComputeDerivative(const FRocketState& S, double t)
{
    FDerivative D{};

    // ── 1. 대기 데이터 (겉보기 속도 기반: V_app = V_rocket - V_wind) ────────
    // 풍속이 있을 때 공력은 로켓 절대속도가 아닌 겉보기 속도로 계산
    auto Atmo  = FAtmosphereISA::GetAtmo(S.Position.z);
    FVector3 VelApparent = S.Velocity - Cfg.WindVelocity; // 겉보기 속도 [World]
    double Vmag = VelApparent.Norm();
    double Mach = (Atmo.SpeedOfSound > 0.0) ? Vmag / Atmo.SpeedOfSound : 0.0;
    double DynQ = 0.5 * Atmo.Density * Vmag * Vmag;

    // ── 2. Body 좌표 겉보기 속도 & Total AoA ─────────────────────────────
    // Body 프레임으로 변환해 받음각(AoA) 계산
    FVector3 VelApparentBody = RotateVectorByQuatInv(VelApparent, S.Attitude);
    //
    // 받음각 정의:
    //   α_total = atan2(√(Vy²+Vz²), Vx)        ∈ [0, π]    — 총 받음각
    //   α_pitch = atan2(Vz, Vx)                 ∈ (-π, π]  — 피치 평면 (X-Z)
    //   α_yaw   = atan2(Vy, Vx)                 ∈ (-π, π]  — 요 평면 (X-Y)
    //
    // 클램핑 / 가드 없음 (이전 버전의 |Vx|>0.05 가드, ±π/2 클램프 모두 제거):
    //   • atan2 는 Vx=0 도 안전하게 처리 (값이 ±π/2 로 정의됨)
    //   • 가로 자세(Body+X ⊥ velocity) 에서 sin(α)=1 이 자연스럽게 최대 모멘트 발휘
    //     → 하강 시 노즈-다운으로 회전을 멈추지 않고 끝까지 진행
    //   • 후방 비행(α≈π) 에서는 sin(α)≈0 으로 자연스러운 불안정 평형 표현
    //     → 작은 perturbation 으로 안정 자세(노즈-다운)로 자연 회복
    //
    // 속도가 매우 작을 때 (정점 근처) 만 α=0 으로 비활성화 — 수치 잡음 방지
    double AlphaTotal = 0.0;
    double AlphaPitch = 0.0;
    double AlphaYaw   = 0.0;
    if (Vmag > 0.5)
    {
        double Vyz = std::sqrt(VelApparentBody.y * VelApparentBody.y +
                                VelApparentBody.z * VelApparentBody.z);
        AlphaTotal = std::atan2(Vyz,                   VelApparentBody.x);
        AlphaPitch = std::atan2(VelApparentBody.z,     VelApparentBody.x);
        AlphaYaw   = std::atan2(VelApparentBody.y,     VelApparentBody.x);
    }

    // ── 3. 항력 (겉보기 속도 반대 방향, World 좌표) ───────────────────────
    // 항력은 항상 겉보기 바람 방향의 반대 (V_app 방향의 -1)
    // Total AoA 사용 → 옆바람에서도 항력 증가 효과 정확히 반영
    double Cd   = FAeroModel::GetCd(Mach, AlphaTotal);
    double Drag = DynQ * Cfg.RefArea * Cd;
    FVector3 DragForce = { 0, 0, 0 };
    if (Vmag > 0.01) {
        double invV = 1.0 / Vmag;
        DragForce = VelApparent * (-Drag * invV);
    }

    // ── 4. 추력 (CurrentThrustDir Body 방향 → World 변환) ────────────────
    // CurrentThrustDir: Step() 진입 시 ComputeThrustDir()로 확률적으로 결정됨
    // 정상(확률 ThrustAlignedProb): Body +X 방향
    // 편향(1-ThrustAlignedProb): 콘 각도 내 균일 랜덤 방향
    double  Thrust       = GetThrust(t);
    FVector3 ThrustBody  = CurrentThrustDir * Thrust;
    FVector3 ThrustWorld = RotateVectorByQuat(ThrustBody, S.Attitude);

    // ── 5. 중력 (World -Z 방향) ───────────────────────────────────────────
    FVector3 GravForce = { 0.0, 0.0, -S.Mass * GRAVITY };

    // ── 6. 합력 → 선가속도 (World 좌표) ──────────────────────────────────
    double InvM = 1.0 / S.Mass;
    FVector3 TotalForce = ThrustWorld + DragForce + GravForce;

    // ── 발사대 구속 (z ≤ 0 에서 순하향력이면 지면 반력 적용) ──────────────
    // 로켓이 지면에 정지 중(z≤0, vz≤0)이고, 합력이 아래 방향이면
    // 지면이 반력을 제공해 수직 운동 억제 (이륙 대기 상태 모델)
    bool onGround = (S.Position.z <= 0.0 && S.Velocity.z <= 0.0);
    if (onGround) {
        if (TotalForce.z * InvM <= 0.0) {
            // 아직 이륙 불가 → 모든 운동 미분 = 0
            D.dPos   = { 0, 0, 0 };
            D.dVel   = { 0, 0, 0 };
            D.dOmega = { 0, 0, 0 };
            D.dAtt   = { 0, 0, 0, 0 };
            // 연소는 진행 (추진제는 소모) — Isp 모델 사용
            if (t < Cfg.BurnTime && GetThrust(t) > 0.0 && S.Mass > Cfg.DryMass)
            {
                double Thr = GetThrust(t);
                if (Cfg.IspSeconds > 0.0)
                    D.dMass = -Thr / (Cfg.IspSeconds * GRAVITY);
                else
                    D.dMass = (Cfg.BurnTime > 0.0) ? -Cfg.PropMass / Cfg.BurnTime : 0.0;
            }
            else
                D.dMass = 0.0;
            // 발사대 가속도 = 0
            LastAccel  = { 0, 0, 0 };
            LastThrust = Thrust;
            LastDrag   = Drag;
            LastMach   = Mach;
            LastDynQ   = DynQ;
            return D;
        }
    }

    D.dVel = TotalForce * InvM;
    D.dPos = S.Velocity;

    // ── 7. 피칭/요잉 모멘트 (AdvancedAeroCoeffModel 사용) ────────────────
    double PitchRate = S.AngularVelocity.y; // Body Y축 피치 각속도
    double YawRate   = S.AngularVelocity.z; // Body Z축 요 각속도
    double CurrentXCG = GetCurrentXCG(S);

    // 피치 평면: α_pitch 와 q (피치 각속도)
    double Cm = FAeroModel::GetCm(AlphaPitch, PitchRate, Vmag,
                                   Cfg.Geometry, CurrentXCG);
    double PitchMoment = DynQ * Cfg.RefArea * Cfg.RefLength * Cm;

    // 요 평면: α_yaw 와 r (요 각속도) — 축대칭 가정으로 동일 함수 사용
    //
    //   부호 규약 (직접 좌표 변환으로 검증됨):
    //     Body 회전축에서 +Y(피치) 와 +Z(요) 의 right-hand rule 방향이
    //     X→Z plane 과 X→Y plane 에서 서로 다르므로, 동일 GetCm() 식을
    //     쓰려면 α_yaw 의 부호를 뒤집어 전달해야 한다.
    //   양의 α_yaw (속도가 Body+Y 쪽에서 들어옴) → 노즈가 -Y 방향(=바람 source)로
    //   회전해야 함 → 음의 ωz → 음의 YawMoment 필요.
    //   GetCm(-α) = +CNα·arm/L·sin(α)  (정적 항이 부호 반전)
    //   damping 항은 +Cmq·r·L/(2V) 그대로 유지 — ωz 가 같은 부호로 damping 됨
    double Cn = FAeroModel::GetCm(-AlphaYaw, YawRate, Vmag,
                                   Cfg.Geometry, CurrentXCG);
    double YawMoment = DynQ * Cfg.RefArea * Cfg.RefLength * Cn;

    // ── 8. Euler 방정식 (Body 좌표 각가속도) — 시변 관성 적용 ────────────
    // dω/dt = (M - ω×(I·ω)) / I  (대각 관성 텐서 가정)
    double Ixx_t, Iyy_t, Izz_t;
    GetCurrentInertia(t, Ixx_t, Iyy_t, Izz_t);
    Ixx_t = std::max(Ixx_t, 1e-9);
    Iyy_t = std::max(Iyy_t, 1e-9);
    Izz_t = std::max(Izz_t, 1e-9);

    const auto& w = S.AngularVelocity;
    D.dOmega = {
        // Roll: 단순화 (추력 편향만이 롤 모멘트 생성 - 본 모델에서는 무시)
        // 자이로 항: (Iyy - Izz) * wy * wz = 0  (대칭)
        0.0,
        // Pitch: 피칭 모멘트 + 자이로 효과
        (PitchMoment - (Ixx_t - Izz_t) * w.x * w.z) / Iyy_t,
        // Yaw: 요잉 모멘트 + 자이로 효과
        (YawMoment   - (Iyy_t - Ixx_t) * w.x * w.y) / Izz_t
    };

    // ── 9. 쿼터니언 미분: dq/dt = 0.5 * q ⊗ [0, ω] ──────────────────────
    const auto& q = S.Attitude;
    D.dAtt = {
        0.5 * (-q.x * w.x - q.y * w.y - q.z * w.z),   // dw
        0.5 * ( q.w * w.x + q.y * w.z - q.z * w.y),   // dx
        0.5 * ( q.w * w.y - q.x * w.z + q.z * w.x),   // dy
        0.5 * ( q.w * w.z + q.x * w.y - q.y * w.x)    // dz
    };

    // ── 10. 질량 감소 (Tsiolkovsky: dm/dt = -ṁ_prop) ─────────────────────
    // Isp 가 주어지면 정확 모델: dm/dt = -F(t) / (Isp · g0)
    // 아니면 fallback: dm/dt = -PropMass/BurnTime (평균 추력 가정)
    // 연소 중에만 질량 감소 (추진제 소진 시 DryMass 이하로 내려가지 않음)
    if (t < Cfg.BurnTime && Thrust > 0.0 && S.Mass > Cfg.DryMass)
    {
        if (Cfg.IspSeconds > 0.0)
            D.dMass = -Thrust / (Cfg.IspSeconds * GRAVITY);
        else
            D.dMass = (Cfg.BurnTime > 0.0) ? -Cfg.PropMass / Cfg.BurnTime : 0.0;
    }
    else
        D.dMass = 0.0;

    // ── 캐시 업데이트 (GetSnapshot용) ────────────────────────────────────
    // const 메서드가 아니므로 직접 갱신
    LastAccel  = D.dVel;
    LastThrust = Thrust;
    LastDrag   = Drag;
    LastMach   = Mach;
    LastDynQ   = DynQ;

    return D;
}

// ═════════════════════════════════════════════════════════════════════════════
//  ApplyDerivative  ─  S + h*D
// ═════════════════════════════════════════════════════════════════════════════
FRocketState FRocketSim6DOF::ApplyDerivative(const FRocketState& S,
                                              const FDerivative& D,
                                              double h) const
{
    FRocketState N = S;
    N.Position        = S.Position        + D.dPos   * h;
    N.Velocity        = S.Velocity        + D.dVel   * h;
    N.Attitude        = S.Attitude        + D.dAtt   * h;
    N.AngularVelocity = S.AngularVelocity + D.dOmega * h;
    N.Mass            = std::max(S.Mass   + D.dMass  * h, Cfg.DryMass);
    return N;
}

// ═════════════════════════════════════════════════════════════════════════════
//  BlendState  ─  S + h * Σ(c_i * D_i)
// ═════════════════════════════════════════════════════════════════════════════
FRocketState FRocketSim6DOF::BlendState(
    const FRocketState& S, double h,
    std::initializer_list<std::pair<double, const FDerivative*>> terms) const
{
    FDerivative sum{};
    for (auto& [c, Dptr] : terms) {
        sum.dPos   = sum.dPos   + Dptr->dPos   * c;
        sum.dVel   = sum.dVel   + Dptr->dVel   * c;
        sum.dAtt   = sum.dAtt   + Dptr->dAtt   * c;
        sum.dOmega = sum.dOmega + Dptr->dOmega * c;
        sum.dMass  = sum.dMass  + Dptr->dMass  * c;
    }
    return ApplyDerivative(S, sum, h);
}

// ═════════════════════════════════════════════════════════════════════════════
//  RK45Step  ─  Dormand-Prince DOPRI5 단일 스텝
//
//  7단계 스테이지 계산 (FSAL: First Same As Last):
//    k1 ~ k6 → 5차 해 y5
//    k7 = f(t+h, y5)  ← 다음 스텝의 k1 재활용 가능 (FSAL)
//    4차 해 y4 = y + h*(b*1·k1 + b*3·k3 + b*4·k4 + b*5·k5 + b*6·k6 + b*7·k7)
//    오차 = y5 - y4
// ═════════════════════════════════════════════════════════════════════════════
FRocketSim6DOF::FRK45Result
FRocketSim6DOF::RK45Step(const FRocketState& S, double t, double h)
{
    // Stage evaluations
    FDerivative k1 = ComputeDerivative(S, t);

    FRocketState s2 = BlendState(S, h, {{DP_a21, &k1}});
    FDerivative k2  = ComputeDerivative(s2, t + DP_c2 * h);

    FRocketState s3 = BlendState(S, h, {{DP_a31, &k1}, {DP_a32, &k2}});
    FDerivative k3  = ComputeDerivative(s3, t + DP_c3 * h);

    FRocketState s4 = BlendState(S, h, {{DP_a41, &k1}, {DP_a42, &k2}, {DP_a43, &k3}});
    FDerivative k4  = ComputeDerivative(s4, t + DP_c4 * h);

    FRocketState s5 = BlendState(S, h, {{DP_a51, &k1}, {DP_a52, &k2},
                                         {DP_a53, &k3}, {DP_a54, &k4}});
    FDerivative k5  = ComputeDerivative(s5, t + DP_c5 * h);

    FRocketState s6 = BlendState(S, h, {{DP_a61, &k1}, {DP_a62, &k2},
                                         {DP_a63, &k3}, {DP_a64, &k4},
                                         {DP_a65, &k5}});
    FDerivative k6  = ComputeDerivative(s6, t + h);

    // 5차 해 (주 해)
    FRocketState y5 = BlendState(S, h, {{DP_b1, &k1},
                                         {DP_b3, &k3},
                                         {DP_b4, &k4},
                                         {DP_b5, &k5},
                                         {DP_b6, &k6}});
    y5.Attitude.Normalize(); // 수치 드리프트 방지

    // FSAL: k7 = f(t+h, y5) → 오차 추정에 사용
    FDerivative k7 = ComputeDerivative(y5, t + h);

    // 4차 해 (오차 추정용)
    FRocketState y4 = BlendState(S, h, {{DP_b4star1, &k1},
                                         {DP_b4star3, &k3},
                                         {DP_b4star4, &k4},
                                         {DP_b4star5, &k5},
                                         {DP_b4star6, &k6},
                                         {DP_b4star7, &k7}});
    y4.Attitude.Normalize();

    return { y5, y4 };
}

// ═════════════════════════════════════════════════════════════════════════════
//  ComputeErrorNorm  ─  혼합 atol/rtol 기반 RMS 오차 노름
//
//  err_norm = sqrt( 1/N * Σ [ (y5_i - y4_i) / sc_i ]² )
//    sc_i   = atol + rtol * max(|yOld_i|, |y5_i|)
//    N      = 상태 성분 수 (Position 3 + Velocity 3 + Quat 4 + ω 3 + Mass 1 = 14)
//  err_norm ≤ 1.0 → 스텝 허용
// ═════════════════════════════════════════════════════════════════════════════
double FRocketSim6DOF::ComputeErrorNorm(const FRocketState& y5,
                                         const FRocketState& y4,
                                         const FRocketState& yOld) const
{
    double atol = Cfg.RK45_AbsTol;
    double rtol = Cfg.RK45_RelTol;
    double sum  = 0.0;
    int    N    = 0;

    auto addComp = [&](double v5, double v4, double vOld) {
        double sc  = atol + rtol * std::max(std::abs(vOld), std::abs(v5));
        double err = (v5 - v4) / sc;
        sum += err * err;
        ++N;
    };

    // Position
    addComp(y5.Position.x, y4.Position.x, yOld.Position.x);
    addComp(y5.Position.y, y4.Position.y, yOld.Position.y);
    addComp(y5.Position.z, y4.Position.z, yOld.Position.z);
    // Velocity
    addComp(y5.Velocity.x, y4.Velocity.x, yOld.Velocity.x);
    addComp(y5.Velocity.y, y4.Velocity.y, yOld.Velocity.y);
    addComp(y5.Velocity.z, y4.Velocity.z, yOld.Velocity.z);
    // Quaternion
    addComp(y5.Attitude.w, y4.Attitude.w, yOld.Attitude.w);
    addComp(y5.Attitude.x, y4.Attitude.x, yOld.Attitude.x);
    addComp(y5.Attitude.y, y4.Attitude.y, yOld.Attitude.y);
    addComp(y5.Attitude.z, y4.Attitude.z, yOld.Attitude.z);
    // Angular velocity
    addComp(y5.AngularVelocity.x, y4.AngularVelocity.x, yOld.AngularVelocity.x);
    addComp(y5.AngularVelocity.y, y4.AngularVelocity.y, yOld.AngularVelocity.y);
    addComp(y5.AngularVelocity.z, y4.AngularVelocity.z, yOld.AngularVelocity.z);
    // Mass
    addComp(y5.Mass, y4.Mass, yOld.Mass);

    return std::sqrt(sum / static_cast<double>(N));
}

// ═════════════════════════════════════════════════════════════════════════════
//  Step  ─  외부 호출 인터페이스 (Unreal Tick에서 호출)
//
//  dt 동안 RK45 적응형 서브스텝을 반복하여 State를 갱신.
//  내부 스텝 크기는 오차 노름에 따라 자동 조절됨.
// ═════════════════════════════════════════════════════════════════════════════
void FRocketSim6DOF::Step(double dt)
{
    if (dt <= 0.0) return;

    // 착지 완료: z < 0 이고 하강 중 → 완전 정지
    if (State.Position.z < 0.0 && State.Velocity.z < 0.0) {
        State.Position.z = 0.0;
        State.Velocity   = { 0, 0, 0 };
        LastAccel        = { 0, 0, 0 };
        return;
    }

    // ── 추력 방향 결정 (이 Step 내 모든 서브스텝에서 동일한 방향 사용) ─────
    // 연소 중일 때만 미스얼라인먼트 적용; 연소 종료 후에는 +X 고정
    if (State.Time < Cfg.BurnTime)
        CurrentThrustDir = ComputeThrustDir();
    else {
        CurrentThrustDir    = { 1.0, 0.0, 0.0 };
        bLastStepMisaligned = false;
    }

    double tRemain = dt;
    double h = std::min({ Cfg.RK45_InitStep, Cfg.RK45_MaxStep, dt });

    // 착지 클램프 람다 (내부 서브스텝마다 체크)
    auto clampGroundContact = [&]() {
        if (State.Position.z < 0.0 && State.Velocity.z < 0.0) {
            State.Position.z = 0.0;
            State.Velocity   = { 0, 0, 0 };
            LastAccel        = { 0, 0, 0 };
            tRemain          = 0.0; // 루프 조기 종료
        }
    };

    while (tRemain > 1.0e-12) {
        h = std::min(h, tRemain);

        FRocketState yOld = State;
        double       tNow = State.Time;

        bool accepted = false;
        int  nReject  = 0;

        while (!accepted) {
            auto [y5, y4] = RK45Step(State, tNow, h);

            double errNorm = ComputeErrorNorm(y5, y4, yOld);

            if (errNorm <= 1.0 || h <= Cfg.RK45_MinStep) {
                // ── 스텝 허용 ─────────────────────────────────────────────
                State      = y5;
                State.Time = tNow + h;
                // 수치 드리프트 방지: 쿼터니언 재정규화
                if (std::abs(State.Attitude.NormSq() - 1.0) > QUAT_RENORM_THRESH)
                    State.Attitude.Normalize();
                tRemain -= h;
                accepted = true;
                clampGroundContact(); // 착지 즉시 클램프

                // ── 발사 레일 구속 ─────────────────────────────────────────
                // 로켓 기저부가 레일 길이(LaunchRailLength)를 벗어나기 전까지만
                // 자세와 각속도를 수직(초기값)으로 고정.
                // bHasLeftRail 로 한 번 레일을 벗어났으면 더 이상 재구속 안 함
                // (하강 시 z 가 다시 레일 길이 아래로 내려와도 영향 없음)
                if (Cfg.LaunchRailLength > 0.0 && !bHasLeftRail)
                {
                    if (State.Position.z < Cfg.LaunchRailLength &&
                        State.Position.z > 0.0)
                    {
                        State.Attitude        = InitialAttitude;
                        State.AngularVelocity = { 0.0, 0.0, 0.0 };
                    }
                    else if (State.Position.z >= Cfg.LaunchRailLength)
                    {
                        bHasLeftRail = true;   // 레일 이탈 — 이후 자유 비행
                    }
                }

                // ── 스텝 크기 제어 (PI 없음, 단순 비례) ──────────────────
                // h_new = h * S * (1/err)^(1/5)   [5차 방법]
                double factor = RK45_SAFETY * std::pow(
                    (errNorm > 1.0e-15) ? 1.0 / errNorm : RK45_MAXFAC / RK45_SAFETY,
                    0.2);
                factor = std::clamp(factor, RK45_MINFAC, RK45_MAXFAC);
                h = std::clamp(h * factor, Cfg.RK45_MinStep, Cfg.RK45_MaxStep);

            } else {
                // ── 스텝 거부 ─────────────────────────────────────────────
                double factor = RK45_SAFETY * std::pow(1.0 / errNorm, 0.2);
                factor = std::clamp(factor, RK45_MINFAC, 1.0);
                h = std::max(h * factor, Cfg.RK45_MinStep);
                ++nReject;

                if (nReject > 50) {
                    // 스텝 무한 거부 방지: 최소 스텝으로 강제 진행
                    h = Cfg.RK45_MinStep;
                }
            }
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  GetSnapshot  ─  Unreal Engine 5와 공유하는 FSimSnapshot 생성
// ═════════════════════════════════════════════════════════════════════════════
FSimSnapshot FRocketSim6DOF::GetSnapshot() const
{
    FSimSnapshot snap{};
    const auto& S = State;

    // 위치 [m]
    snap.PosX = static_cast<float>(S.Position.x);
    snap.PosY = static_cast<float>(S.Position.y);
    snap.PosZ = static_cast<float>(S.Position.z);

    // 속도 [m/s]
    snap.VelX = static_cast<float>(S.Velocity.x);
    snap.VelY = static_cast<float>(S.Velocity.y);
    snap.VelZ = static_cast<float>(S.Velocity.z);

    // 가속도 [m/s²] ← 언리얼 로켓 캐드 물리 제어에 활용
    snap.AccX = static_cast<float>(LastAccel.x);
    snap.AccY = static_cast<float>(LastAccel.y);
    snap.AccZ = static_cast<float>(LastAccel.z);

    // 자세 쿼터니언 (Body→World)
    snap.QuatW = static_cast<float>(S.Attitude.w);
    snap.QuatX = static_cast<float>(S.Attitude.x);
    snap.QuatY = static_cast<float>(S.Attitude.y);
    snap.QuatZ = static_cast<float>(S.Attitude.z);

    // 각속도 [rad/s] (Body 좌표)
    snap.AngVelX = static_cast<float>(S.AngularVelocity.x);
    snap.AngVelY = static_cast<float>(S.AngularVelocity.y);
    snap.AngVelZ = static_cast<float>(S.AngularVelocity.z);

    // 공력 / 추진 진단
    snap.Mach        = static_cast<float>(LastMach);
    snap.DynPressure = static_cast<float>(LastDynQ);
    snap.AltAGL      = static_cast<float>(S.Position.z); // 발사장 고도 = 0 가정
    snap.ThrustN     = static_cast<float>(LastThrust);
    snap.DragN       = static_cast<float>(LastDrag);

    snap.SimTime = S.Time;

    return snap;
}

// ═════════════════════════════════════════════════════════════════════════════
//  쿼터니언 회전 헬퍼
// ═════════════════════════════════════════════════════════════════════════════

// v' = q ⊗ [0,v] ⊗ q*  (World ← Body)
FVector3 FRocketSim6DOF::RotateVectorByQuat(const FVector3& V, const FQuaternion& Q)
{
    // 최적화된 공식 (2배 반복 곱 대신 직접 전개)
    double tx = 2.0 * (Q.y * V.z - Q.z * V.y);
    double ty = 2.0 * (Q.z * V.x - Q.x * V.z);
    double tz = 2.0 * (Q.x * V.y - Q.y * V.x);
    return {
        V.x + Q.w * tx + Q.y * tz - Q.z * ty,
        V.y + Q.w * ty + Q.z * tx - Q.x * tz,
        V.z + Q.w * tz + Q.x * ty - Q.y * tx
    };
}

// v' = q* ⊗ [0,v] ⊗ q  (Body ← World)  →  켤레 쿼터니언으로 역변환
FVector3 FRocketSim6DOF::RotateVectorByQuatInv(const FVector3& V, const FQuaternion& Q)
{
    FQuaternion Qc = { Q.w, -Q.x, -Q.y, -Q.z }; // 켤레
    return RotateVectorByQuat(V, Qc);
}
