#pragma once
#include <cmath>
#include <algorithm>

// ISA Atmosphere Model Definition
class FAtmosphereISA {
public:
    // Atmosphere Structure Definition
    struct FAtmoData
    {
        double Pressure;
        double Density;
        double Temperature;
        double SpeedOfSound;
    };
	// Atmosphere Calculation Function
    static FAtmoData GetAtmo(double AltMSL_m) {
        constexpr double R = 287.058;                                        // General Gas Constant [J/kg·K]
		constexpr double g0 = 9.80665;                                       // Standard Gravity [m/s²]
		constexpr double T0 = 288.15;                                        // Sea Level Standard Temperature [K]
		constexpr double P0 = 101325.0;                                      // Sea Level Standard Pressure [Pa]
		constexpr double L = 0.0065;                                         // Temperature Lapse Rate [K/m]
		constexpr double Gamma = 1.4;                                        // Specific Heat Ratio for Air

        FAtmoData d{};
        double h = std::max(0.0, AltMSL_m);

		d.Temperature = T0 - L * h;                                     // Tempurature relative to attitude
	    d.Pressure = P0 * std::pow(d.Temperature / T0, g0 / (L * R));   // Pressure relative to attitude
		d.Density = d.Pressure / (R * d.Temperature);                       // Density relative to attitude
		d.SpeedOfSound = std::sqrt(Gamma * R * d.Temperature);              // Speed of sound relative to attitude
        
        return d;
    }
};
