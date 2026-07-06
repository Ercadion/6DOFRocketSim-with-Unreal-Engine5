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

        // ── [A5] 핀 두께 [m] ────────────────────────────────────────────
        // 0.0 → 두께비 6% (MAC 기준) fallback
        double FinThickness = 0.0;

        // ── [A2] 노즐 출구 직경 [m] ─────────────────────────────────────
        // 연소 중 베이스 항력 감소 계산용: Cd_base_burn = Cd_base_coast × (1 − (Dn/d)²)
        // 0.0 → legacy 상수 0.04 사용
        double NozzleExitDiameter = 0.0;

        // ── [A6] launch lug / rail button ───────────────────────────────
        double LugCount        = 2.0;     // 개수
        double LugFrontalArea  = 4.0e-5;  // 1개 정면적 [m²] (기본 40mm²)
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
    //  Cd 성분 분해 진단 구조체 (모두 RefArea 기준)
    //  GetCdAdvanced 의 마지막 인자로 전달하면 성분별 값이 채워짐 (로깅용)
    // ─────────────────────────────────────────────────────────────────────────
    struct FCdBreakdown
    {
        double Skin  = 0.0;   // 본체+노즈 마찰 (form factor 포함)
        double Fin   = 0.0;   // 핀 마찰 + 두께 profile + 간섭
        double Base  = 0.0;   // 베이스 항력
        double Lug   = 0.0;   // launch lug / rail button
        double Par   = 0.0;   // parasitic (외부 하드웨어)
        double Wave  = 0.0;   // 천음속/초음속 wave drag
        double AoA   = 0.0;   // 받음각 cross-flow
        double Total = 0.0;
    };

    // ─────────────────────────────────────────────────────────────────────────
    //  GetCdAdvanced — 형상·Reynolds 기반 항력 모델 (16주차 개정판)
    //
    //  성분 분해 (모두 RefArea 기준):
    //    Cd_skin : 본체+노즈 마찰. Cf(Re) × body form factor × wetted/Aref
    //              [A3] 거칠기: 임의 배율 → 물리적 거칠기 높이 Rs[μm] 모델
    //                   Cf_rough = 0.032·(Rs/L)^0.2, Cf = max(Cf_smooth, Cf_rough)
    //              [A4] body form factor (1 + 1/(2·f_B)) 추가
    //    Cd_fin  : 핀 마찰 + Hoerner 두께 profile + 간섭 (K=1.30)
    //              [A5] 두께비 = FinThickness/MAC (미지정 시 6%)
    //    Cd_base : [A2] Hoerner: 0.12 + 0.13·M² (M<1), 0.25/M (M≥1)
    //              연소 중엔 노즐 출구 면적비만큼 감소
    //    Cd_lug  : [A6] 개수·정면적 파라미터화
    //    Cd_par  : [A6] 기본 fudge 0.05 제거 → ExtraParasiticCd 만 사용
    //    Cd_wave : [A1+A7] 노즈 압력항력을 아음속(≈0)에서 제거하고,
    //              등가 원뿔각 기반 peak (0.8·sin²θ)로 천음속 이상에서만 적용
    //    Cd_aoa  : [A8] planform 면적 기준 cross-flow (Jorgensen/Galejs)
    //
    //  주의: 아음속 유선형 노즈(오자이브)의 압력항력은 ≈0 (OpenRocket 동일).
    //  구모델의 상시 Cd_nose(+0.05)와 과대 베이스 항력(+0.06)이 제거되어
    //  아음속 코스트 Cd 가 약 0.10~0.15 감소함.
    // ─────────────────────────────────────────────────────────────────────────
    inline double GetCdAdvanced(double Mach,
                                double Alpha,
                                double Re,            // Reynolds (본체 전장 기준)
                                bool   bCoasting,
                                const  FRocketGeometry& G,
                                double RefArea_m2,    // [m²]
                                double RoughnessMicrons = 60.0,
                                                      // 표면 거칠기 높이 Rs [μm]
                                                      // 20=광택, 60=페인트,
                                                      // 100=미도장 FRP, 250=거친
                                double ExtraParasiticCd = 0.0,
                                                      // 컴포넌트 기반 외부 하드웨어 Cd
                                                      // 0 → parasitic 없음
                                FCdBreakdown* OutBreakdown = nullptr)
    {
        const double d      = G.RefLength;
        const double LBody  = G.BodyLength;
        const double LNose  = G.NoseLength;
        const double LTotal = std::max(LBody + LNose, 1e-6);
        const double Aref   = std::max(RefArea_m2, 1e-9);
        constexpr double PI_CONST = 3.14159265358979323846;

        // ── 1. 마찰계수 Cf (Re + 거칠기 높이 + 압축성) ────────────────────
        //   smooth: laminar 1.328/√Re, turbulent 1/(3.46·log10(Re)−5.6)²
        //   rough : 0.032·(Rs/L)^0.2  (roughness-limited, turbulent 시 하한)
        //   압축성: M<1 → ×(1−0.1M²),  M≥1 → ×(1+0.15M²)^−0.58
        auto skin_friction = [&](double ReX, double Lchar) -> double {
            ReX = std::max(ReX, 1.0);
            double Cf;
            if (ReX < 1.0e4)
                Cf = 1.48e-2;                                   // 저 Re 상한
            else if (ReX < 5.0e5)
                Cf = 1.328 / std::sqrt(ReX);                    // laminar
            else
            {
                Cf = 1.0 / std::pow(3.46 * std::log10(ReX) - 5.6, 2.0); // turbulent
                if (RoughnessMicrons > 0.0)                     // 거칠기 하한
                {
                    const double Rs = RoughnessMicrons * 1.0e-6;
                    const double Cf_rough =
                        0.032 * std::pow(Rs / std::max(Lchar, 1e-6), 0.2);
                    Cf = std::max(Cf, Cf_rough);
                }
            }
            if (Mach < 1.0)
                Cf *= (1.0 - 0.1 * Mach * Mach);
            else
                Cf *= std::pow(1.0 + 0.15 * Mach * Mach, -0.58);
            return Cf;
        };

        // ── 2. 본체+노즈 마찰 (body form factor 포함) ─────────────────────
        const double Cf_body  = skin_friction(Re, LTotal);
        const double fB       = LTotal / std::max(d, 1e-6);    // 전장 fineness
        const double FF_body  = 1.0 + 1.0 / (2.0 * std::max(fB, 1.0)); // [A4]
        const double Aw_body  = PI_CONST * d * LBody;
        const double Aw_nose  = PI_CONST * d * LNose * 0.85;   // 오자이브 보정
        const double Cd_skin  = Cf_body * FF_body * (Aw_body + Aw_nose) / Aref;

        // ── 3. 핀 항력 (마찰 + 두께 profile + 간섭) ───────────────────────
        const double cr   = G.FinRootChord;
        const double ct   = G.FinTipChord;
        const double s    = G.FinSpan;
        const double crct = std::max(cr + ct, 1e-9);
        const double mac  = (2.0 / 3.0) * (cr + ct - cr * ct / crct);
        const double Re_fin  = Re * mac / LTotal;              // MAC 기준 Re
        const double Cf_fin  = skin_friction(Re_fin, std::max(mac, 1e-6));

        // [A5] 두께비: FinThickness 지정 시 실제값, 아니면 6% fallback
        const double tc = (G.FinThickness > 0.0 && mac > 1e-6)
                        ? G.FinThickness / mac
                        : 0.06;

        const double A_fin_one = 0.5 * (cr + ct) * s;          // 사다리꼴
        const double Aw_fin    = 2.0 * G.FinCount * A_fin_one; // 양면
        const double Cd_fin_alone =
            Cf_fin * (1.0 + 2.0 * tc + 60.0 * tc * tc * tc * tc)
            * Aw_fin / Aref;
        const double K_interference = 1.30;                    // 핀-본체 간섭
        const double Cd_fin = Cd_fin_alone * K_interference;

        // ── 4. 베이스 항력 [A2] (Hoerner) ─────────────────────────────────
        //   코스트 : Cd_base = 0.12 + 0.13·M²  (M<1),  0.25/M  (M≥1)
        //   연소 중: 노즐 배기가 베이스 wake 일부를 채움
        //            → 노즐 출구 면적 제외한 환형 면적만 기여
        //            Cd_base_burn = Cd_base_coast × max(0, 1 − (Dn/d)²)
        //            (Dn 미지정 시 legacy 상수 0.04)
        double Cd_base_coast;
        if (Mach < 1.0) Cd_base_coast = 0.12 + 0.13 * Mach * Mach;
        else            Cd_base_coast = 0.25 / Mach;

        double Cd_base;
        if (bCoasting)
            Cd_base = Cd_base_coast;
        else if (G.NozzleExitDiameter > 0.0)
        {
            const double ratio = G.NozzleExitDiameter / std::max(d, 1e-6);
            Cd_base = Cd_base_coast * std::max(0.0, 1.0 - ratio * ratio);
        }
        else
            Cd_base = 0.04;                                    // legacy fallback

        // ── 5. Launch lug [A6] (파라미터화) ───────────────────────────────
        const double Cd_lug = 0.5 * G.LugCount * G.LugFrontalArea / Aref;

        // ── 6. Parasitic [A6] ─────────────────────────────────────────────
        //   기본 fudge(0.05) 제거. 컴포넌트 기반 산출값만 사용.
        const double Cd_par = std::max(ExtraParasiticCd, 0.0);

        // ── 7. Wave drag [A1+A7] (노즈 형상 기반) ─────────────────────────
        //   아음속(M<0.8): 유선형 노즈 압력항력 ≈ 0 → wave 성분 없음
        //   peak: 등가 원뿔 반각 θ = atan(1/(2f)) 의 초음속 원뿔 압력항력
        //         Cd_peak = 0.8·sin²θ  (구모델이 상시 더하던 Cd_nose 를
        //         천음속 이상 구간으로 이동시킨 것)
        //   M 0.8→1.05 : t² ramp,  M>1.05 : (1.05/M)² 감쇠
        const double f_nose  = LNose / std::max(d, 1e-6);
        const double sinTh   = std::sin(std::atan(1.0 / (2.0 * std::max(f_nose, 0.1))));
        const double Cd_peak = 0.8 * sinTh * sinTh;
        double Cd_wave;
        if (Mach < 0.8)
            Cd_wave = 0.0;
        else if (Mach < 1.05)
        {
            const double t = (Mach - 0.8) / 0.25;
            Cd_wave = Cd_peak * t * t;
        }
        else
            Cd_wave = Cd_peak * (1.05 / Mach) * (1.05 / Mach);

        // ── 8. 받음각 cross-flow [A8] (planform 기준, Jorgensen/Galejs) ──
        //   ΔCd = Cd_c × (A_plan/A_ref) × |sin³α|,  Cd_c ≈ 1.1 (원통 cross-flow)
        //   구모델의 1.2·sin²α (RefArea 기준)는 동체 측면적을 무시해 과소평가.
        const double A_plan = d * (LBody + 0.7 * LNose);       // 측면 투영 면적
        const double sinA   = std::sin(Alpha);
        const double Cd_aoa = 1.1 * (A_plan / Aref)
                            * sinA * sinA * std::abs(sinA);

        const double Cd_total = Cd_skin + Cd_fin + Cd_base
                              + Cd_lug + Cd_par + Cd_wave + Cd_aoa;

        if (OutBreakdown)
        {
            OutBreakdown->Skin  = Cd_skin;
            OutBreakdown->Fin   = Cd_fin;
            OutBreakdown->Base  = Cd_base;
            OutBreakdown->Lug   = Cd_lug;
            OutBreakdown->Par   = Cd_par;
            OutBreakdown->Wave  = Cd_wave;
            OutBreakdown->AoA   = Cd_aoa;
            OutBreakdown->Total = Cd_total;
        }
        return Cd_total;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  [Phase 1 A-2] 법선력 곡선 기울기 CN_α  [1/rad]
    //
    //  Barrowman 정확 공식:
    //    CN_α_total = CN_α_nose + K_body × CN_α_fin
    //      CN_α_nose = 2.0  (모든 형상 공통, 노즈콘 base 기준)
    //      CN_α_fin  = (4 N (s/d)²) / (1 + √(1 + (2 lm/(cr+ct))²))
    //          lm = √(s² + (sweep + ct/2 − cr/2)²)   (mid-chord line)
    //      K_body    = 1 + R_b / (s + R_b)            (body-on-fin interference)
    //
    //  주의: 이 값은 RefArea 기준 (RocketSim6DOF에서 q × A_ref × CN_α × sin α 로
    //  법선력을 계산). 기존 GetCm 함수와 동일한 계산을 그대로 공유.
    // ─────────────────────────────────────────────────────────────────────────
    inline double GetCNalpha(const FRocketGeometry& G)
    {
        const double CNalpha_nose = 2.0;

        const double d  = G.RefLength;
        const double Rb = 0.5 * d;
        const double s  = G.FinSpan;
        const double cr = G.FinRootChord;
        const double ct = G.FinTipChord;
        const double sweep = G.FinSweep;

        const double lm   = std::sqrt(s*s + (sweep + ct*0.5 - cr*0.5)
                                            * (sweep + ct*0.5 - cr*0.5));
        const double sd   = s / (d > 1e-9 ? d : 1e-9);
        const double crct = (cr + ct) > 1e-9 ? (cr + ct) : 1e-9;
        const double denom = 1.0 + std::sqrt(1.0 + (2.0 * lm / crct)
                                                    * (2.0 * lm / crct));
        const double CN_fin_alone = (4.0 * G.FinCount * sd * sd)
                                    / (denom > 1e-9 ? denom : 1e-9);
        const double K_body       = 1.0 + Rb
                                    / ((s + Rb) > 1e-9 ? (s + Rb) : 1e-9);
        const double CNalpha_fin  = K_body * CN_fin_alone;

        return CNalpha_nose + CNalpha_fin;
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
        // [Phase 1 A-2] CN_α 는 별도 GetCNalpha() 로 분리, 여기선 호출만.
        double CNalpha = GetCNalpha(G);

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
