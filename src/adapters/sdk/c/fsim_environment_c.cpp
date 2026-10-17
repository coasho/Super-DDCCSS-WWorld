// C ABI Environment, terrain and magnetic adapters over session::World.

#include "fsim/EnvironmentState.h"
#include "fsim/Magnetic.h"
#include "fsim/fsim_c.h"

#include "CAbiSupport.h"
#include "Handles.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <optional>

extern "C" {

FSIM_API void fsim_environment_init(fsim_environment* e) {
    if (!e) return;
    const fsim::sim::EnvironmentState d;
    e->struct_size = sizeof(*e);
    e->epoch_utc_seconds = d.epochUtcSeconds;
    e->time_factor = d.timeFactor;
    e->temperature_sl_k = d.temperatureSeaLevelK;
    e->pressure_sl_pa = d.pressureSeaLevelPa;
    e->humidity = d.humidity;
    e->wind_direction_deg = d.windDirectionDeg;
    e->wind_speed_ms = d.windSpeedMs;
    e->wind_gust_ms = d.windGustMs;
    e->turbulence = d.turbulence;
    e->visibility_m = d.visibilityM;
    e->cloud_base_m = d.cloudBaseM;
    e->cloud_cover = d.cloudCover;
    e->precipitation = d.precipitation;
}

FSIM_API int fsim_world_get_environment(const fsim_world* world, fsim_environment* out) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || !out || out->struct_size < sizeof(fsim_environment)) return FSIM_INVALID_ARGUMENT;
    const auto& d = world->world.environment();
    out->epoch_utc_seconds = d.epochUtcSeconds; out->time_factor = d.timeFactor;
    out->temperature_sl_k = d.temperatureSeaLevelK; out->pressure_sl_pa = d.pressureSeaLevelPa; out->humidity = d.humidity;
    out->wind_direction_deg = d.windDirectionDeg; out->wind_speed_ms = d.windSpeedMs; out->wind_gust_ms = d.windGustMs; out->turbulence = d.turbulence;
    out->visibility_m = d.visibilityM; out->cloud_base_m = d.cloudBaseM; out->cloud_cover = d.cloudCover; out->precipitation = d.precipitation;
    return FSIM_OK;
}

FSIM_API int fsim_world_set_environment(fsim_world* world, const fsim_environment* e) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || !e || e->struct_size < sizeof(fsim_environment)) return FSIM_INVALID_ARGUMENT;
    auto d = world->world.environment();
    d.epochUtcSeconds = e->epoch_utc_seconds; d.timeFactor = e->time_factor;
    d.temperatureSeaLevelK = e->temperature_sl_k; d.pressureSeaLevelPa = e->pressure_sl_pa; d.humidity = e->humidity;
    d.windDirectionDeg = e->wind_direction_deg; d.windSpeedMs = e->wind_speed_ms; d.windGustMs = e->wind_gust_ms; d.turbulence = e->turbulence;
    d.visibilityM = e->visibility_m; d.cloudBaseM = e->cloud_base_m; d.cloudCover = e->cloud_cover; d.precipitation = e->precipitation;
    world->world.setEnvironment(d);
    return FSIM_OK;
}

FSIM_API int fsim_world_terrain(const fsim_world* world, uint32_t count, const double* latitude_rad, const double* longitude_rad, double* height_m) {
    FSIM_AGENT_CALL(world, FSIM_NOT_ALLOWED);
    if (!world || (count && (!latitude_rad || !longitude_rad || !height_m))) return FSIM_INVALID_ARGUMENT;
    int known = 0;
    for (uint32_t i = 0; i < count; ++i) {
        const std::optional<double> h = world->world.terrainHeightM(latitude_rad[i], longitude_rad[i]);
        height_m[i] = h ? *h : std::numeric_limits<double>::quiet_NaN();
        known += h ? 1 : 0;
    }
    return known;
}

FSIM_API void fsim_magnetic_field_init(fsim_magnetic_field* field) {
    if (!field) return;
    std::memset(field, 0, sizeof *field);
    field->struct_size = sizeof *field;
    field->north_nt = field->east_nt = field->down_nt = field->horizontal_nt = field->total_nt = std::numeric_limits<double>::quiet_NaN();
    field->declination_rad = field->inclination_rad = std::numeric_limits<double>::quiet_NaN();
}

FSIM_API int fsim_magnetic_field_at(double latitude_rad, double longitude_rad, double height_m, double decimal_year, fsim_magnetic_field* out) {
    if (!std::isfinite(latitude_rad) || std::abs(latitude_rad) > 0.5 * 3.14159265358979323846 || !std::isfinite(longitude_rad) ||
        !std::isfinite(height_m) || !std::isfinite(decimal_year))
        return FSIM_INVALID_ARGUMENT;
    const fsim::control::MagneticField m = fsim::control::magneticField(latitude_rad, longitude_rad, height_m, decimal_year);
    fsim_magnetic_field f;
    fsim_magnetic_field_init(&f);
    f.north_nt = m.northNt, f.east_nt = m.eastNt, f.down_nt = m.downNt, f.horizontal_nt = m.horizontalNt, f.total_nt = m.totalNt;
    f.declination_rad = m.declinationRad, f.inclination_rad = m.inclinationRad;
    return copyOut(f, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API double fsim_decimal_year(double unix_seconds) { return fsim::control::decimalYear(unix_seconds); }

FSIM_API double fsim_world_magnetic_year(const fsim_world* world) {
    FSIM_NOT_AGENT(world, std::numeric_limits<double>::quiet_NaN());
    if (!world) return std::numeric_limits<double>::quiet_NaN();
    return fsim::control::magneticYear(world->world.environment().epochUtcSeconds + world->world.simTime());
}

} // extern "C"
