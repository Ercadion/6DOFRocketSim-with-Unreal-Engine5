#include "RocketSim6DOF.h"
#include <cmath>
#include <algorithm>

FRocketSim6DOF::FRocketSim6DOF(const FRocketConfig& InCfg) : Cfg(InCfg) 
{
    Reset({ {0,0,0}, {0,0,0}, FQuaternion::Identity(), {0,0,0}, Cfg.DryMass + Cfg.PropMass, 0.0 });
	// = position, Velocity, Attitude, AngularVelocity, Mass, Time
}

void FRocketSim6DOF::Reset(const FRocketState& InitState) {
    State = InitState;
}

FRocketState FRocketSim6DOF::Integrate(
    const FRocketState& S,
    const FDerivative& D,
    double dt
) const
{
    FRocketState N = S;
    N.Position = S.Position + D.dPos * dt;
    N.Velocity = S.Velocity + D.dVel * dt;
    N.Attitude = S.Attitude + D.dAtt * dt;
    N.AngularVelocity = S.AngularVelocity + D.dOmega * dt;
    N.Mass = std::max(S.Mass + D.dMass * dt, Cfg.DryMass);
    return N;
}

// RK4 Integral
void FRocketSim6DOF::Step(double dt) {
    double t = State.Time;
    FDerivative k1 = ComputeDerivative(State, t);
    FDerivative k2 = ComputeDerivative(Integrate(State, k1, dt * 0.5), t + dt * 0.5);
    FDerivative k3 = ComputeDerivative(Integrate(State, k2, dt * 0.5), t + dt * 0.5);
    FDerivative k4 = ComputeDerivative(Integrate(State, k3, dt), t + dt);

    // 가중합
    FDerivative Final;
    Final.dPos = (k1.dPos + k2.dPos * 2 + k3.dPos * 2 + k4.dPos) * (1.0 / 6.0);
    Final.dVel = (k1.dVel + k2.dVel * 2 + k3.dVel * 2 + k4.dVel) * (1.0 / 6.0);
    Final.dAtt = (k1.dAtt + k2.dAtt * 2 + k3.dAtt * 2 + k4.dAtt) * (1.0 / 6.0);
    Final.dOmega = (k1.dOmega + k2.dOmega * 2 + k3.dOmega * 2 + k4.dOmega) * (1.0 / 6.0);
    Final.dMass = (k1.dMass + k2.dMass * 2 + k3.dMass * 2 + k4.dMass) / 6.0;

    State = Integrate(State, Final, dt);
    State.Attitude.Normalize();   // 수치 드리프트 방지
    State.Time += dt;
}

FRocketSim6DOF::FDerivative
FRocketSim6DOF::ComputeDerivative(const FRocketState& S, double t) const {
    FDerivative D{};

    // 1. 대기 데이터
    auto Atmo = FAtmosphereISA::GetAtmo(S.Position.z);
    double Vmag = std::sqrt(S.Velocity.x * S.Velocity.x +
        S.Velocity.y * S.Velocity.y +
        S.Velocity.z * S.Velocity.z);
    double Mach = (Atmo.SpeedOfSound > 0) ? Vmag / Atmo.SpeedOfSound : 0.0;
    double DynQ = 0.5 * Atmo.Density * Vmag * Vmag;

    // 2. Body 속도 (공격각 계산용)
    FVector3 VelBody = RotateVectorByQuatInv(S.Velocity, S.Attitude);
    double Alpha = (std::abs(VelBody.x) > 0.1) ?
        std::atan2(VelBody.z, VelBody.x) : 0.0;

    // 3. 항력 (World 좌표, 속도 반대 방향)
    double Cd = FAeroModel::GetCd(Mach, Alpha);
    double Drag = DynQ * Cfg.RefArea * Cd;
    FVector3 DragForce = { 0,0,0 };
    if (Vmag > 0.01) {
        DragForce = { -S.Velocity.x / Vmag * Drag,
                      -S.Velocity.y / Vmag * Drag,
                      -S.Velocity.z / Vmag * Drag };
    }

    // 4. 추력 (Body +X 방향 → World 변환)
    double Thrust = GetThrust(t);
    FVector3 ThrustBody = { Thrust, 0, 0 };
    FVector3 ThrustWorld = RotateVectorByQuat(ThrustBody, S.Attitude);

    // 5. 중력
    FVector3 GravForce = { 0, 0, -S.Mass * 9.80665 };

    // 6. 합력 → 선가속도
    double InvM = 1.0 / S.Mass;
    D.dVel = { (ThrustWorld.x + DragForce.x + GravForce.x) * InvM,
               (ThrustWorld.y + DragForce.y + GravForce.y) * InvM,
               (ThrustWorld.z + DragForce.z + GravForce.z) * InvM };
    D.dPos = S.Velocity;

    // 7. 피칭 모멘트 → 각가속도 (Body 좌표)
    double Cm = FAeroModel::GetCm(Alpha);
    double PitchMoment = DynQ * Cfg.RefArea * Cfg.RefLength * Cm;
    // Euler 방정식 (단순화: Ixx,Iyy,Izz만 사용)
    D.dOmega = { 0.0,
                 (PitchMoment - (Cfg.Izz - Cfg.Ixx) * S.AngularVelocity.x * S.AngularVelocity.z) / Cfg.Iyy,
                 -(Cfg.Iyy - Cfg.Ixx) * S.AngularVelocity.x * S.AngularVelocity.y / Cfg.Izz };

    // 8. 쿼터니언 도함수  dq/dt = 0.5 * q ⊗ [0, ω]
    const auto& q = S.Attitude;
    const auto& w = S.AngularVelocity;
    D.dAtt = {
        0.5 * (-q.x * w.x - q.y * w.y - q.z * w.z),
        0.5 * (q.w * w.x + q.y * w.z - q.z * w.y),
        0.5 * (q.w * w.y - q.x * w.z + q.z * w.x),
        0.5 * (q.w * w.z + q.x * w.y - q.y * w.x)
    };

    // 9. 질량 감소 (연소 중)
    D.dMass = (t < Cfg.BurnTime && Thrust > 0) ?
        -Cfg.PropMass / Cfg.BurnTime : 0.0;

    return D;
}

double FRocketSim6DOF::GetThrust(double t) const {
    if (t < 0 || t > Cfg.BurnTime || Cfg.ThrustCurve.empty())
        return 0.0;
    // 선형 보간
    for (size_t i = 1; i < Cfg.ThrustCurve.size(); ++i) {
        if (t <= Cfg.ThrustCurve[i].first) {
            double dt2 = Cfg.ThrustCurve[i].first - Cfg.ThrustCurve[i - 1].first;
            double frac = (dt2 > 0) ? (t - Cfg.ThrustCurve[i - 1].first) / dt2 : 0.0;
            return Cfg.ThrustCurve[i - 1].second +
                frac * (Cfg.ThrustCurve[i].second - Cfg.ThrustCurve[i - 1].second);
        }
    }
    return 0.0;
}

FVector3 FRocketSim6DOF::RotateVectorByQuat(const FVector3& V, const FQuaternion& Q) {
    // v' = q * [0,v] * q^(-1)
    double tx = 2 * (Q.y * V.z - Q.z * V.y);
    double ty = 2 * (Q.z * V.x - Q.x * V.z);
    double tz = 2 * (Q.x * V.y - Q.y * V.x);
    return { V.x + Q.w * tx + Q.y * tz - Q.z * ty,
             V.y + Q.w * ty + Q.z * tx - Q.x * tz,
             V.z + Q.w * tz + Q.x * ty - Q.y * tx };
}

FVector3 FRocketSim6DOF::RotateVectorByQuatInv(const FVector3& V, const FQuaternion& Q) {
    FQuaternion Qc = { Q.w, -Q.x, -Q.y, -Q.z };   // 켤레
    return RotateVectorByQuat(V, Qc);
}
