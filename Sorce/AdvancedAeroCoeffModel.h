// AdvancedAeroCoeffModel.h
// 로켓 공력 계수 모델
//   - GetCd  : 항력계수 (Mach, 받음각 함수)
//   - GetCm  : 피칭 모멘트 계수 (CP-CG 오프셋, 핀 효과, 감쇠 포함)
//   - FRocketGeometry : 공력 형상 파라미터 구조체
#pragma once
#include <cmath>

namespace FAeroModel
{
    // ─────────────────────────────────────────────────────────────────────────
    //  로켓 형상 파라미터 (RocketInputLoader 에서 JSON으로 채워짐)
    // ─────────────────────────────────────────────────────────────────────────
    struct FRocketGeometry
    {
        double BodyLength   = 0.58;   // 기체 전장 [m]
        double NoseLength   = 0.12;   // 노즈콘 길이 [m]
        double RefLength    = 0.064;  // 기준 길이 = 직경 [m]
        double FinCount     = 4.0;    // 핀 개수
        double FinSpan      = 0.040;  // 핀 스팬 [m] (루트에서 팁까지)
        double FinRootChord = 0.070;  // 핀 루트 코드 [m]
        double FinTipChord  = 0.030;  // 핀 팁 코드 [m]
        double FinSweep     = 0.030;  // LE → 팁 LE 후퇴 [m] (Barrowman lm 계산용)
        double XCP          = 0.370;  // 공기 압력 중심 위치 (노즈 기준) [m]
        double XCG_initial  = 0.295;  // 초기 무게 중심 (노즈 기준) [m]
    };

    // ─────────────────────────────────────────────────────────────────────────
    //  항력계수 Cd  (Mach, 받음각 α [rad])
    //
    //  모델:
    //    Cd_base : 기체 마찰/압력 항력 (마하 함수)
    //    Cd_wave : 천음속/초음속 파항력 (Ma 0.8~1.2 근처 spike)
    //    Cd_alpha: 받음각에 의한 항력 증가 (∝ sin²α)
    // ─────────────────────────────────────────────────────────────────────────
    inline double GetCd(double Mach, double Alpha)
    {
        // ── 기저 항력 (아음속 / 천음속 / 초음속) ─────────────────────────────
        double Cd_base;
        if (Mach < 0.8)
        {
            // 아음속: Prandtl-Glauert 보정
            double beta = std::sqrt(std::max(1.0 - Mach * Mach, 0.01));
            Cd_base = 0.35 / beta;
        }
        else if (Mach < 1.2)
        {
            // 천음속: 비선형 spike (Ma=1.0 근방 최대)
            double t = (Mach - 0.8) / 0.4;          // 0~1
            double spike = 4.0 * t * (1.0 - t);     // 포물선 0→1→0
            Cd_base = 0.35 + 0.45 * spike;
        }
        else
        {
            // 초음속: 1/Ma 감소 경향
            Cd_base = 0.35 + 0.45 / (Mach * Mach);
        }

        // ── 받음각에 의한 항력 증가 ───────────────────────────────────────────
        double sinA    = std::sin(Alpha);
        double Cd_aoa  = 1.2 * sinA * sinA;   // 단면 항력 기여

        return Cd_base + Cd_aoa;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  피칭 모멘트 계수 Cm  (Body Y축 기준, 노즈 기준 모멘트)
    //
    //  Cm = CNα * (XCP - XCG) / RefLength * sin(α) + Cmq * q_hat
    //
    //    CNα : 법선력 곡선 기울기 [1/rad]
    //    XCP : 압력 중심 [m] (노즈 기준)
    //    XCG : 무게 중심 [m] (노즈 기준, 현재 추진제 소모 반영)
    //    Cmq : 피치 감쇠 계수 (q-damping)
    //    q_hat = q * RefLength / (2 * Vmag)  (무차원 피치율)
    // ─────────────────────────────────────────────────────────────────────────
    inline double GetCm(double Alpha,
                        double PitchRate,      // [rad/s], Body Y축
                        double Vmag,           // 겉보기 속도 크기 [m/s]
                        const FRocketGeometry& G,
                        double XCG)            // 현재 CG (노즈 기준) [m]
    {
        // 법선력 기울기 CNα — 정확한 Barrowman 공식 사용
        //   노즈콘:     CNα_nose = 2.0  (모든 형상 공통)
        //   핀 세트:    CNα_fin  = K_body * (4 N (s/d)²) /
        //                          (1 + sqrt(1 + (2 lm/(cr+ct))²))
        //     K_body = 1 + R_b / (s + R_b)   (body interference)
        //     lm     = sqrt(s² + (sweep + ct/2 - cr/2)²)  (mid-chord line)
        // ─────────────────────────────────────────────────────────────────
        double CNalpha_nose = 2.0;

        double d  = G.RefLength;
        double Rb = 0.5 * d;
        double s  = G.FinSpan;
        double cr = G.FinRootChord;
        double ct = G.FinTipChord;
        double sweep = G.FinSweep;
        double lm = std::sqrt(s*s + (sweep + ct*0.5 - cr*0.5) * (sweep + ct*0.5 - cr*0.5));
        double sd = s / (d > 1e-9 ? d : 1e-9);
        double crct = (cr + ct) > 1e-9 ? (cr + ct) : 1e-9;
        double denom = 1.0 + std::sqrt(1.0 + (2.0 * lm / crct) * (2.0 * lm / crct));
        double CN_fin_alone = (4.0 * G.FinCount * sd * sd) / (denom > 1e-9 ? denom : 1e-9);
        double K_body       = 1.0 + Rb / ((s + Rb) > 1e-9 ? (s + Rb) : 1e-9);
        double CNalpha_fin  = K_body * CN_fin_alone;

        double CNalpha = CNalpha_nose + CNalpha_fin;

        // 정적 안정 모멘트
        double staticArm = G.XCP - XCG;          // 양수 = 안정, 음수 = 불안정
        //
        // 부호 규약: α > 0 → Cm < 0 (복원 모멘트, 노즈를 속도 방향으로 정렬)
        //   CP가 CG 뒤에 있을 때(staticArm > 0) 안정 → 부호 음수
        //
        // 클램핑 없음 (이전 ±π/2 클램프 제거):
        //   α 가 ±π 까지 자연스럽게 변할 수 있도록 sin(α) 를 그대로 사용
        //   • 작은 α (forward) → sin(α)≈α, 표준 선형 복원
        //   • α = ±π/2 (가로 자세, perpendicular flight) → sin=±1, 최대 모멘트
        //   • α ≈ ±π (tail-first) → sin≈0, 불안정 평형 자연 표현
        //     (작은 perturbation 으로 안정 자세로 자동 회복)
        double Cm_static = -CNalpha * (staticArm / std::max(G.RefLength, 1e-6))
                         * std::sin(Alpha);

        // 피치 감쇠 (q-damping)  ─ 강화된 형상 기반 식
        //
        //   1) 선형 (Cmq * q):  Barrowman 핀 댐핑
        //        Cmq = -64 * CN_α * (arm/L)²
        //        Apogee 후 가로 자세 → 노즈-다운 전환 시 진자 운동을 방지하기 위해
        //        coefficient 를 -16 → -64 로 4배 강화 (ζ ≈ 0.4 → 0.7)
        //
        //   2) 비선형 (quadratic, ∝ |ω|·ω): 대진폭 진동 빠른 감쇠
        //        실제 로켓의 vortex shedding / 와류 손실 모델링
        //        선형 댐핑은 작은 ω 에서만 효과적; 큰 ω 에서는 quadratic 이 지배적
        //
        //   결과: ζ ≈ 0.6~0.8 (약간 over-damped), 1~2 cycle 안에 노즈-다운 안정화
        double Cmq_geom = -64.0 * CNalpha * (staticArm * staticArm)
                                           / std::max(G.RefLength * G.RefLength, 1e-12);
        double Cmq = (Cmq_geom < -800.0) ? Cmq_geom : -800.0;  // 최소 -800

        double q_hat = 0.0;
        if (Vmag > 0.1)
            q_hat = PitchRate * G.RefLength / (2.0 * Vmag);
        double Cm_damp_linear = Cmq * q_hat;

        // Quadratic damping (vortex shedding / 대진폭 감쇠)
        //   M_quad = -k_q * |ω| * ω    where k_q proportional to fin geometry
        //   k_q 가 정적 stiffness 와 같은 단위가 되도록 normalize:
        //     Cm_quad = -C_q * |q_hat| * q_hat,  C_q ≈ 8 * (arm/L)²
        double C_quad = 8.0 * (staticArm * staticArm)
                              / std::max(G.RefLength * G.RefLength, 1e-12);
        double Cm_damp_quad = -C_quad * std::abs(q_hat) * q_hat;

        return Cm_static + Cm_damp_linear + Cm_damp_quad;
    }

} // namespace FAeroModel
