#pragma once
#include <array>
#include <cmath>

// Vector structure Definition
struct FVector3 {
    double x, y, z;
    // Vector addition
	FVector3 operator+(const FVector3& o) const 
    { 
        return { x + o.x, y + o.y, z + o.z }; 
    }
    // Scalar multiplication
	FVector3 operator*(double s) const 
    { 
        return { x * s,   y * s,   z * s }; 
    }
};

// Quaternion structure Definition
struct FQuaternion {
    double w, x, y, z;
	// Quaternion addition
    FQuaternion operator+(const FQuaternion& o) const 
    { 
        return { w + o.w, x + o.x, y + o.y, z + o.z }; 
    }
	// scalar multiplication
    FQuaternion operator*(double s) const 
    { 
        return { w * s,   x * s,   y * s,   z * s }; 
    }
	// Quaternion normalization
    void Normalize() 
    {
        double n = std::sqrt(w * w + x * x + y * y + z * z);
        w /= n; x /= n; y /= n; z /= n;
    }
	// Quaternion identity(when there is no rotation)
    static FQuaternion Identity() 
    { 
        return { 1,0,0,0 }; 
    }
};

// 6DOF State Vector Definition
struct FRocketState {
    FVector3    Position;       // 지구 고정 좌표 [m]
    FVector3    Velocity;       // 지구 고정 좌표 속도 [m/s]
    FQuaternion Attitude;       // Body→World 쿼터니언
    FVector3    AngularVelocity;// Body 좌표 각속도 [rad/s]
    double      Mass;           // 현재 질량 [kg]
    double      Time;           // 경과 시간 [s]
};

// Snapshot Structure Definition(shared with Unreal5 Engine)
struct FSimSnapshot {
    float PosX, PosY, PosZ;
    float VelX, VelY, VelZ;
    float QuatW, QuatX, QuatY, QuatZ;
    float AngVelX, AngVelY, AngVelZ;
    float Mach, DynPressure, AltAGL;
    float ThrustN, DragN;
    double SimTime;
};
