#include "visualization/scene/Sky.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>

namespace fsim::world {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
} // namespace

vsg::dvec3 sunDirectionEcef(int dayOfYear, double utcHours) {
    const double declination = 23.44 * kDeg * std::sin(2.0 * kPi * (284.0 + dayOfYear) / 365.0);
    const double subsolarLon = (180.0 - 15.0 * utcHours) * kDeg; // sun over Greenwich at 12:00 UTC
    return vsg::dvec3(std::cos(declination) * std::cos(subsolarLon), std::cos(declination) * std::sin(subsolarLon),
                      std::sin(declination));
}

namespace {
float smooth(double a, double b, double x) {
    const double t = std::clamp((x - a) / (b - a), 0.0, 1.0);
    return static_cast<float>(t * t * (3.0 - 2.0 * t));
}
vsg::vec3 mix(const vsg::vec3& a, const vsg::vec3& b, float t) { return a * (1.0f - t) + b * t; }
} // namespace

Daylight daylightAt(const vsg::dvec3& toSunEcef, const vsg::dvec3& upEcef, double overcast) {
    Daylight d;
    const double s = vsg::dot(vsg::normalize(toSunEcef), vsg::normalize(upEcef));
    d.sinElevation = s;
    // the sun: up from 1 deg below the horizon to 3.5 deg above it (its disc, refraction, the air's thickness), warm
    // below 20 deg
    const float up = smooth(-0.02, 0.06, s);
    const float low = 1.0f - smooth(0.02, 0.35, s);
    d.sun = 0.70f * up;
    d.sunColour = mix(vsg::vec3(1.0f, 0.97f, 0.92f), vsg::vec3(1.0f, 0.60f, 0.34f), 0.85f * low);
    // the sky: full by 8 deg up, gone to a night's below -11 deg (nautical twilight)
    const float day = smooth(-0.19, 0.14, s);
    d.sky = 0.04f + 0.18f * day;
    d.skyColour = mix(vsg::vec3(0.45f, 0.55f, 0.85f), vsg::vec3(0.78f, 0.87f, 1.00f), day);
    d.ground = 0.07f * day * std::max(up, 0.3f);
    d.ambient = 0.07f + 0.10f * day;
    d.ambientColour = mix(vsg::vec3(0.55f, 0.62f, 0.85f), vsg::vec3(1.0f, 1.0f, 1.0f), day);
    // under a deck of cloud: less sun, and what there is diffused from a grey sky
    const float o = static_cast<float>(std::clamp(overcast, 0.0, 1.0));
    if (o > 0.0f) {
        d.sky *= 1.0f + 0.5f * o;
        d.ambient *= 1.0f + 0.3f * o;
        d.skyColour = mix(d.skyColour, vsg::vec3(0.82f, 0.84f, 0.88f) * (0.9f + 0.1f * day), o);
        d.ground *= 1.0f - 0.6f * o;
        d.sun *= 1.0f - o;
    }
    return d;
}

vsg::ref_ptr<vsg::Node> createSunLight(int dayOfYear, double utcHours) {
    auto group = vsg::Group::create();
    auto ambientLight = vsg::AmbientLight::create();
    ambientLight->name = "ambient";
    group->addChild(ambientLight);
    // the sun first: the haze (world/Scattering) takes its light from the first directional light
    auto sun = vsg::DirectionalLight::create();
    sun->name = "sun";
    group->addChild(sun);
    auto sky = vsg::DirectionalLight::create();
    sky->name = "sky";
    group->addChild(sky);
    auto ground = vsg::DirectionalLight::create();
    ground->name = "ground";
    ground->color.set(0.85f, 0.80f, 0.70f); // the ground's colour, more or less
    group->addChild(ground);
    const vsg::dvec3 toSun = sunDirectionEcef(dayOfYear, utcHours);
    setDaylight(group, toSun, toSun); // (lit as under the sun until a place is given)
    return group;
}

void utcOf(double unixSeconds, int& dayOfYear, double& utcHours) {
    const std::time_t t = static_cast<std::time_t>(unixSeconds);
    std::tm utc{};
#ifdef _WIN32
    gmtime_s(&utc, &t);
#else
    gmtime_r(&t, &utc);
#endif
    dayOfYear = utc.tm_yday + 1;
    utcHours = utc.tm_hour + utc.tm_min / 60.0 + utc.tm_sec / 3600.0;
}

void setDaylight(vsg::Node* sunLight, const vsg::dvec3& toSunEcef, const vsg::dvec3& upEcef, double overcast) {
    auto* group = dynamic_cast<vsg::Group*>(sunLight);
    if (!group) return;
    const Daylight d = daylightAt(toSunEcef, upEcef, overcast);
    const vsg::dvec3 up = vsg::normalize(upEcef);
    for (auto& child : group->children) {
        if (auto* light = dynamic_cast<vsg::DirectionalLight*>(child.get())) { // direction: the way the light travels
            if (light->name == "sun") {
                light->direction = -vsg::normalize(toSunEcef);
                light->color = d.sunColour;
                light->intensity = d.sun;
            } else if (light->name == "sky") {
                light->direction = -up;
                light->color = d.skyColour;
                light->intensity = d.sky;
            } else if (light->name == "ground") {
                light->direction = up;
                light->intensity = d.ground;
            }
        } else if (auto* ambient = dynamic_cast<vsg::AmbientLight*>(child.get())) {
            ambient->color = d.ambientColour;
            ambient->intensity = d.ambient;
        }
    }
}

void currentUtc(int& dayOfYear, double& utcHours) {
    const std::time_t now = std::time(nullptr);
    std::tm utc{};
#ifdef _WIN32
    gmtime_s(&utc, &now);
#else
    gmtime_r(&now, &utc);
#endif
    dayOfYear = utc.tm_yday + 1;
    utcHours = utc.tm_hour + utc.tm_min / 60.0 + utc.tm_sec / 3600.0;
}

} // namespace fsim::world
