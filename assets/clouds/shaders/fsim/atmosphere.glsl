// fsim: Bruneton's precomputed atmosphere as takram's three-geospatial sets it up (packages/atmosphere:
// AtmosphereParameters.ts, constants.ts, AtmosphereMaterialBase.ts) - its default Earth atmosphere, the sizes of the
// precomputed tables it ships (assets/clouds/atmosphere: float16 RGBA, combined scattering with single Mie in alpha),
// lengths in kilometres, and luminance relative to the sun's.

#define TRANSMITTANCE_TEXTURE_WIDTH 256
#define TRANSMITTANCE_TEXTURE_HEIGHT 64
#define SCATTERING_TEXTURE_R_SIZE 32
#define SCATTERING_TEXTURE_MU_SIZE 128
#define SCATTERING_TEXTURE_MU_S_SIZE 32
#define SCATTERING_TEXTURE_NU_SIZE 8
#define IRRADIANCE_TEXTURE_WIDTH 64
#define IRRADIANCE_TEXTURE_HEIGHT 16
#define METER_TO_LENGTH_UNIT 0.001
#define COMBINED_SCATTERING_TEXTURES 1

#include "bruneton/definitions.glsl"

// AtmosphereParameters.DEFAULT.toUniform()
const AtmosphereParameters ATMOSPHERE = AtmosphereParameters(
  vec3(1.474, 1.8504, 1.91198), // solar_irradiance
  0.004675,                     // sun_angular_radius
  6360.0,                       // bottom_radius (km)
  6420.0,                       // top_radius (km)
  DensityProfile(DensityProfileLayer[2](DensityProfileLayer(0.0, 0.0, 0.0, 0.0, 0.0),
                                        DensityProfileLayer(0.0, 1.0, -0.125, 0.0, 0.0))),
  vec3(0.005802, 0.013558, 0.0331), // rayleigh_scattering
  DensityProfile(DensityProfileLayer[2](DensityProfileLayer(0.0, 0.0, 0.0, 0.0, 0.0),
                                        DensityProfileLayer(0.0, 1.0, -0.833333, 0.0, 0.0))),
  vec3(0.003996), // mie_scattering
  vec3(0.00444),  // mie_extinction
  0.8,            // mie_phase_function_g
  DensityProfile(DensityProfileLayer[2](DensityProfileLayer(25.0, 0.0, 0.0, 1.0 / 15.0, -2.0 / 3.0),
                                        DensityProfileLayer(0.0, 0.0, 0.0, -1.0 / 15.0, 8.0 / 3.0))),
  vec3(0.00065, 0.001881, 0.000085), // absorption_extinction
  vec3(0.1),                         // ground_albedo
  -0.5                               // mu_s_min: cos(120 deg)
);

// sunRadianceToRelativeLuminance and skyRadianceToRelativeLuminance: the luminance conversions divided by the
// luminance of the sun at unit radiance (as the tables are stored, to fit half floats)
const vec3 FSIM_SUN_RADIANCE_TO_LUMINANCE = vec3(98242.786222, 69954.398112, 66475.012354);
const float FSIM_SUN_UNIT_LUMINANCE = 0.2126 * 98242.786222 + 0.7152 * 69954.398112 + 0.0722 * 66475.012354;
const vec3 SUN_SPECTRAL_RADIANCE_TO_LUMINANCE = FSIM_SUN_RADIANCE_TO_LUMINANCE / FSIM_SUN_UNIT_LUMINANCE;
const vec3 SKY_SPECTRAL_RADIANCE_TO_LUMINANCE = vec3(114974.916437, 71305.954816, 65310.548555) / FSIM_SUN_UNIT_LUMINANCE;

layout(set = 0, binding = 6) uniform sampler2D transmittance_texture;
layout(set = 0, binding = 7) uniform sampler3D scattering_texture;
layout(set = 0, binding = 8) uniform sampler2D irradiance_texture;
// (combined: single Mie scattering is in the scattering table's alpha; this is the same table, only passed along)
layout(set = 0, binding = 20) uniform sampler3D single_mie_scattering_texture;

#include "bruneton/common.glsl"
#include "bruneton/runtime.glsl"
