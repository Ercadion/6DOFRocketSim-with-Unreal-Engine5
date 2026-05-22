// AtmosphereModel.h
// 국제 표준 대기 (ISA, International Standard Atmosphere)
// 고도 h [m] 에서 밀도, 압력, 온도, 음속 반환
// 대기 구조:
//   0    ~ 11000 m : 대류권 (기온 감률 -6.5 K/km)
//   11000 ~ 20000 m : 성층권 하부 (등온 216.65 K)
//   20000 ~ 32000 m : 성층권 중부 (기온 감률 +1.0 K/km)
#pragma once
#include <cmath>

struct FAtmoData
{
    double Temperature;    // [K]
    double Pressure;       // [Pa]
    double Density;        // [kg/m³]
    double SpeedOfSound;   // [m/s]
};

class FAtmosphereISA
{
public:
    // h : 기하 고도 [m] (음수면 해수면으로 클램프)
    static FAtmoData GetAtmo(double h)
    {
        // 상수
        static constexpr double R      = 287.058;   // 기체상수 [J/(kg·K)]
        static constexpr double gamma  = 1.4;        // 비열비
        static constexpr double g0     = 9.80665;    // 중력가속도 [m/s²]

        // 해수면 기준값
        static constexpr double T0     = 288.15;     // [K]
        static constexpr double P0     = 101325.0;   // [Pa]

        // 음수 고도 방지
        if (h < 0.0) h = 0.0;

        double T, P;

        if (h <= 11000.0)
        {
            // 대류권: 기온 감률 L = -6.5 K/km
            const double L = -0.0065;
            T = T0 + L * h;
            P = P0 * std::pow(T / T0, -g0 / (L * R));
        }
        else if (h <= 20000.0)
        {
            // 성층권 하부: 등온 216.65 K
            const double T11 = 216.65;
            const double P11 = P0 * std::pow(T11 / T0, -g0 / (-0.0065 * R));
            T = T11;
            P = P11 * std::exp(-g0 * (h - 11000.0) / (R * T11));
        }
        else if (h <= 32000.0)
        {
            // 성층권 중부: 기온 감률 +1.0 K/km
            const double T11 = 216.65;
            const double P11 = P0 * std::pow(T11 / T0, -g0 / (-0.0065 * R));
            const double P20 = P11 * std::exp(-g0 * (20000.0 - 11000.0) / (R * T11));
            const double T20 = 216.65;
            const double L   = 0.001;
            T = T20 + L * (h - 20000.0);
            P = P20 * std::pow(T / T20, -g0 / (L * R));
        }
        else
        {
            // 32km 이상: 외삽 (로켓 비행 범위 초과 시)
            const double T11 = 216.65;
            const double P11 = P0 * std::pow(T11 / T0, -g0 / (-0.0065 * R));
            const double P20 = P11 * std::exp(-g0 * (20000.0 - 11000.0) / (R * T11));
            const double T20 = 216.65;
            const double L   = 0.001;
            T = T20 + L * (32000.0 - 20000.0);
            P = P20 * std::pow(T / T20, -g0 / (L * R));
            // 32km 이상은 등온 외삽
            T = 228.65;
            P *= std::exp(-g0 * (h - 32000.0) / (R * T));
        }

        FAtmoData out;
        out.Temperature  = T;
        out.Pressure     = P;
        out.Density      = P / (R * T);
        out.SpeedOfSound = std::sqrt(gamma * R * T);
        return out;
    }
};
