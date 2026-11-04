#include "NavigationConfigurationV2.h"
#include <cmath>
#include <exception>
#include <set>
namespace fsim::ms::weapon {
bool navigationConfigurationV2(const StoreCard& card,std::optional<NavigationStrapdownConfigV2>& out,std::string& error) {
    out.reset();error.clear();
    const auto selected=card.parameters.find("own_navigation_version");
    if(selected==card.parameters.end())return true;
    if(selected->second.unit!="1"||selected->second.value!=2||!std::isfinite(selected->second.uncertainty)||selected->second.uncertainty<0){error="unsupported own navigation version";return false;}
    NavigationStrapdownConfigV2 config;
    const auto number=[&](const char* key,const char* unit,double& value) {
        const auto found=card.parameters.find(key);
        if(found==card.parameters.end()||found->second.unit!=unit||!std::isfinite(found->second.value)||
           !std::isfinite(found->second.uncertainty)||found->second.uncertainty<0){error=std::string("invalid own navigation parameter: ")+key;return false;}
        value=found->second.value;return true;
    };
    double fitted=0;if(!number("own_nav_gnss_fitted","1",fitted))return false;
    if(fitted!=0&&fitted!=1){error="own navigation GNSS fit must be boolean";return false;}config.gnssAiding=fitted==1;
    struct Field {const char* key;const char* unit;double NavigationStrapdownConfigV2::*member;};
    const Field fields[]={
        {"own_nav_imu_period_s","s",&NavigationStrapdownConfigV2::imuPeriodS},
        {"own_nav_gnss_period_s","s",&NavigationStrapdownConfigV2::gnssPeriodS},
        {"own_nav_maximum_integration_step_s","s",&NavigationStrapdownConfigV2::maximumIntegrationStepS},
        {"own_nav_accelerometer_noise_density","m/s^2 sqrt(s)",&NavigationStrapdownConfigV2::accelerometerNoiseDensity},
        {"own_nav_gyro_noise_density","rad/s sqrt(s)",&NavigationStrapdownConfigV2::gyroNoiseDensity},
        {"own_nav_accelerometer_bias_sigma","m/s^2",&NavigationStrapdownConfigV2::accelerometerBiasSigma},
        {"own_nav_gyro_bias_sigma","rad/s",&NavigationStrapdownConfigV2::gyroBiasSigma},
        {"own_nav_accelerometer_bias_walk_density","m/s^2/sqrt(s)",&NavigationStrapdownConfigV2::accelerometerBiasWalkDensity},
        {"own_nav_gyro_bias_walk_density","rad/s/sqrt(s)",&NavigationStrapdownConfigV2::gyroBiasWalkDensity},
        {"own_nav_initial_position_sigma_m","m",&NavigationStrapdownConfigV2::initialPositionSigmaM},
        {"own_nav_initial_velocity_sigma_ms","m/s",&NavigationStrapdownConfigV2::initialVelocitySigmaMs},
        {"own_nav_initial_attitude_sigma_rad","rad",&NavigationStrapdownConfigV2::initialAttitudeSigmaRad},
        {"own_nav_gnss_position_sigma_m","m",&NavigationStrapdownConfigV2::gnssPositionSigmaM},
        {"own_nav_gnss_velocity_sigma_ms","m/s",&NavigationStrapdownConfigV2::gnssVelocitySigmaMs},
    };
    std::set<std::string> admitted{"own_nav_gnss_fitted"};
    for(const auto& field:fields)admitted.insert(field.key);
    for(const auto& parameter:card.parameters)
        if(parameter.first.compare(0,8,"own_nav_")==0&&!admitted.count(parameter.first)) {
            error="unknown own navigation parameter: "+parameter.first;return false;
        }
    for(const auto& field:fields)if(!number(field.key,field.unit,config.*field.member))return false;
    try {NavigationStrapdownV2 validate(config,0);(void)validate;}
    catch(const std::exception& e){error=e.what();return false;}
    out=config;return true;
}
}
