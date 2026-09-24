#pragma once

#include <Eigen/Dense>

class GNSS2NED {
    
    public:
        GNSS2NED();
        GNSS2NED(float lati0_deg, float longi0_deg, float elev0);

        Eigen::Vector3d toNED(float lati_deg, float longi_deg, float elev);

    private:
        Eigen::Vector3d lla2ecef(float lati_rad, float longi_rad, float elev);

        float lati0_rad;
        float longi0_rad;
        float elev0;

        static constexpr double pi = 3.14159265358979323846;
        static constexpr double a_Wgs84 = 6378137.0;
        static constexpr double f_Wgs84 = 1.0 / 298.257223563;
        static constexpr double e2_Wgs84 = f_Wgs84 * (2 - f_Wgs84);

        Eigen::Vector3d originECEF;
        Eigen::Matrix3d Recef2NED;


};