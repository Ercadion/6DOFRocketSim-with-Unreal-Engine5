// RocketSim6DOF.h
#pragma once
#include "RocketVectorDefinition.h"
#include "AtmosphereModel.h"
#include "AeroCoeffModel.h"
#include <vector>
#include <functional>

struct FRocketConfig {
    double RefArea;         // Reference Area [m²]
	double RefLength;       // Reference Length (Diameter) [m]
	double Ixx, Iyy, Izz;   // Moment of Inertia Components [kg·m²]
    double BurnTime;        // Burn Time [s]
    double MaxThrust;       // Max Thrust [N]
	double DryMass;         // Dry Mass [kg]
    double PropMass;        // Propellant Weight [kg]
	std::vector<std::pair<double, double>> ThrustCurve; // Time-Thrust Table(t, N)
};

class FRocketSim6DOF {
public:
    explicit FRocketSim6DOF(const FRocketConfig& Cfg);

    void Reset(const FRocketState& InitState);
    void Step(double dt);                          // RK4 single step

    const FRocketState& GetState() const { return State; }
    FSimSnapshot        GetSnapshot() const;

private:
    FRocketConfig   Cfg;
    FRocketState    State;

    // RK4 도함수 계산
    struct FDerivative {
        FVector3    dPos, dVel;
        FQuaternion dAtt;
        FVector3    dOmega;
        double      dMass;
    };

    FDerivative ComputeDerivative(const FRocketState& S, double t) const;
    FRocketState Integrate(const FRocketState& S, const FDerivative& D, double dt) const;

    double GetThrust(double t) const;

	// World↔Body transformation using quaternions 
    static FVector3 RotateVectorByQuat(const FVector3& V, const FQuaternion& Q);
    static FVector3 RotateVectorByQuatInv(const FVector3& V, const FQuaternion& Q);
};
