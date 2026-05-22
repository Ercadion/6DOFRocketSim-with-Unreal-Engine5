// RocketVectorDefinition.h
// 기본 벡터/쿼터니언 타입 및 상태 구조체 정의
// Unreal Engine과 공유하는 FSimSnapshot에 가속도(AccX/Y/Z) 추가
#pragma once
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
//  3D 벡터
// ─────────────────────────────────────────────────────────────────────────────
struct FVector3 {
    double x, y, z;

    FVector3 operator+(const FVector3& o) const { return { x + o.x, y + o.y, z + o.z }; }
    FVector3 operator-(const FVector3& o) const { return { x - o.x, y - o.y, z - o.z }; }
    FVector3 operator*(double s)          const { return { x * s,   y * s,   z * s   }; }
    FVector3& operator+=(const FVector3& o)     { x += o.x; y += o.y; z += o.z; return *this; }

    double Norm()    const { return std::sqrt(x * x + y * y + z * z); }
    double NormSq()  const { return x * x + y * y + z * z; }
    double Dot(const FVector3& o) const { return x * o.x + y * o.y + z * o.z; }
};

// ─────────────────────────────────────────────────────────────────────────────
//  쿼터니언  (Body → World 변환 규약)
// ─────────────────────────────────────────────────────────────────────────────
struct FQuaternion {
    double w, x, y, z;

    FQuaternion operator+(const FQuaternion& o) const
        { return { w + o.w, x + o.x, y + o.y, z + o.z }; }
    FQuaternion operator*(double s) const
        { return { w * s, x * s, y * s, z * s }; }

    void Normalize() {
        double n = std::sqrt(w * w + x * x + y * y + z * z);
        if (n > 1e-15) { w /= n; x /= n; y /= n; z /= n; }
    }
    double NormSq() const { return w * w + x * x + y * y + z * z; }

    static FQuaternion Identity() { return { 1.0, 0.0, 0.0, 0.0 }; }
};

// ─────────────────────────────────────────────────────────────────────────────
//  6DOF 상태 벡터
// ─────────────────────────────────────────────────────────────────────────────
struct FRocketState {
    FVector3    Position;        // 지구 고정 좌표 [m]     (+Z 위쪽)
    FVector3    Velocity;        // 지구 고정 좌표 속도 [m/s]
    FQuaternion Attitude;        // Body→World 쿼터니언
    FVector3    AngularVelocity; // Body 좌표 각속도 [rad/s]
    double      Mass;            // 현재 질량 [kg]
    double      Time;            // 경과 시간 [s]
};

// ─────────────────────────────────────────────────────────────────────────────
//  FSimSnapshot  ─  Unreal Engine 5와 공유하는 출력 구조체
//
//  ※ Tick마다 FRocketSim6DOF::GetSnapshot()으로 채워서 Unreal에 넘긴다.
//  ※ 단위계: 위치[m], 속도[m/s], 가속도[m/s²], 각속도[rad/s], 압력[Pa], 추력/항력[N]
//  ※ Unreal 좌표계는 cm 단위이므로 호출 측에서 ×100 변환 필요
// ─────────────────────────────────────────────────────────────────────────────
struct FSimSnapshot {
    // ── 위치 (World 좌표, m) ─────────────────────────────────────────────
    float PosX, PosY, PosZ;

    // ── 속도 (World 좌표, m/s) ──────────────────────────────────────────
    float VelX, VelY, VelZ;

    // ── 가속도 (World 좌표, m/s²)  ← Unreal 로켓 캐드 제어에 활용 ────────
    float AccX, AccY, AccZ;

    // ── 자세 쿼터니언 (Body→World) ───────────────────────────────────────
    float QuatW, QuatX, QuatY, QuatZ;

    // ── 각속도 (Body 좌표, rad/s) ────────────────────────────────────────
    float AngVelX, AngVelY, AngVelZ;

    // ── 공력 / 추진 진단값 ───────────────────────────────────────────────
    float Mach;          // 마하수 [-]
    float DynPressure;   // 동압 [Pa]
    float AltAGL;        // AGL 고도 [m]
    float ThrustN;       // 추력 [N]
    float DragN;         // 항력 [N]

    // ── 시뮬레이션 시각 ──────────────────────────────────────────────────
    double SimTime;      // [s]
};
