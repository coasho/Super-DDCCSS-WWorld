#pragma once

#include <vsg/all.h>

namespace fsim::world {

/// Sun direction in ECEF for a UTC time of year/day (design 8.2 "Sky / lighting").
/// Approximate solar position (declination from day of year, hour angle from
/// UTC, no equation of time): good to ~2 degrees, which is all lighting needs.
vsg::dvec3 sunDirectionEcef(int dayOfYear, double utcHours);

/// The light a place gets from the sun and the sky, as one function of the sun's elevation there, so that what it
/// lights - the aircraft, the terrain - and the haze (which scatters the same sun: world/Scattering) agree at every
/// hour. The sun fades through its rising and setting and is gone below the horizon (it used to shine up through the
/// Earth at night, lighting bellies from beneath); low, it is warm. The sky lights from above - a fill from the zenith
/// over a little ambient - and the ground bounces a little back from below, so in daylight upper surfaces are lit
/// more than the belly. Scaled so level ground under a high sun sees about 1: the imagery, sunlit already, keeps its
/// own brightness instead of being washed out.
struct Daylight {
    double sinElevation = 1.0;         ///< the sun's elevation there (sine)
    float sun = 0.0f;                  ///< the sun's intensity
    vsg::vec3 sunColour{1.0f, 1.0f, 1.0f};
    float sky = 0.0f;                  ///< the sky's light from the zenith
    vsg::vec3 skyColour{1.0f, 1.0f, 1.0f};
    float ground = 0.0f;               ///< the light the ground bounces up
    float ambient = 0.0f;              ///< from everywhere
    vsg::vec3 ambientColour{1.0f, 1.0f, 1.0f};
};
/// `overcast` (0..1, world::Clouds::overcast) is how much of the sun a deck of cloud overhead takes away: the light
/// it lets through comes from the whole of a grey sky instead.
Daylight daylightAt(const vsg::dvec3& toSunEcef, const vsg::dvec3& upEcef, double overcast = 0.0);

/// The sun (the first directional light: the haze takes its light from it), the sky's light from above, the ground's
/// from below and the ambient, positioned by sunDirectionEcef() and lit for the subsolar point until setDaylight()
/// says where the eye is. Replaces the headlight for scenes with terrain relief.
vsg::ref_ptr<vsg::Node> createSunLight(int dayOfYear, double utcHours);

/// Day of year (1-366) and UTC hour (0-24) of "now".
void currentUtc(int& dayOfYear, double& utcHours);

/// Day of year and UTC hour of a Unix time (seconds).
void utcOf(double unixSeconds, int& dayOfYear, double& utcHours);

/// Re-aim and re-light a group made by createSunLight() for a place (its up, ECEF): every frame is cheap enough.
void setDaylight(vsg::Node* sunLight, const vsg::dvec3& toSunEcef, const vsg::dvec3& upEcef, double overcast = 0.0);

} // namespace fsim::world
