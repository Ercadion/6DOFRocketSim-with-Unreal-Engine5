#pragma once
#include <vector>
#include <algorithm>

class FAeroModel {
public:
    // Mach-Cd Table(arbitrary value, CFD needed)
    static double GetCd(double Mach, double AlphaRad) {
        static const std::vector<std::pair<double, double>> CdTable = {
            {0.0, 0.35}, {0.5, 0.33}, {0.8, 0.38},
            {1.0, 0.65}, {1.2, 0.55}, {2.0, 0.42}, {3.0, 0.38}
        };
        // Over Mach range case handling
        if (Mach <= CdTable.front().first) 
        {
            return CdTable.front().second;
        }
        if (Mach >= CdTable.back().first)
        {
            return CdTable.back().second;
        }
		// Linear interpolation for Mach
        for (size_t i = 1; i < CdTable.size(); ++i) 
        {
            if (Mach < CdTable[i].first) {
                double t = (Mach - CdTable[i - 1].first) / (CdTable[i].first - CdTable[i - 1].first);
                return CdTable[i - 1].second + t * (CdTable[i].second - CdTable[i - 1].second);
            }
        }
        return 0.4;
    }

	// AeroDynamic Pitching Moment Coefficient (Simplified Model)
    static double GetCm(double AlphaRad) {
        return -0.5 * AlphaRad;  // 정적 안정 로켓 가정
    }
};
