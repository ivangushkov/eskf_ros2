#include "gnss2ned/gnss2ned.h"

#include <Eigen/Dense>
#include <cmath>
#include <iostream> 


GNSS2NED::GNSS2NED(float lati0_deg, float longi0_deg, float elev0)
{

    // Save the origin of the NED frame in radians
    lati0_rad = lati0_deg * pi / 180.0;
    longi0_rad = longi0_deg * pi / 180.0;
    elev0 = elev0;

    originECEF = lla2ecef(lati0_rad, longi0_rad, elev0);

    // Rotation between ECEF and NED
    Recef2NED << - std::sin(lati0_rad)*std::cos(longi0_rad), -std::sin(lati0_rad)*std::sin(longi0_rad), std::cos(lati0_rad),
                -std::sin(longi0_rad), std::cos(longi0_rad), 0,
                -std::cos(lati0_rad)*std::cos(longi0_rad), -std::cos(lati0_rad)*std::sin(longi0_rad), -std::sin(lati0_rad);


}

Eigen::Vector3d GNSS2NED::lla2ecef(float lati_rad, float longi_rad, float elev) {

    Eigen::Vector3d ecef;

    float N = a_Wgs84 / std::sqrt(1 - e2_Wgs84 * std::pow(std::sin(lati_rad), 2));

    ecef(0) = (N + elev) * std::cos(lati_rad) * std::cos(longi_rad);
    ecef(1) = (N + elev) * std::cos(lati_rad) * std::sin(longi_rad);
    ecef(2) = (N*(1 - e2_Wgs84) + elev) * std::sin(lati_rad);

    return ecef;
}

Eigen::Vector3d GNSS2NED::toNED(float lati_deg, float longi_deg, float elev) {


    Eigen::Vector3d ecef = this->lla2ecef(lati_deg * pi / 180, longi_deg * pi/180 , elev);
    Eigen::Vector3d ned = Recef2NED * (ecef - originECEF);

    //std::cout << "ECEC - origin: " << ecef - originECEF << std::endl;
    //std::cout << "EFEC position: " << ecef << std::endl;
    
    return ned;

}