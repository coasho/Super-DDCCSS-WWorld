// fsim: the contrails' shared declarations (world::Airflow). A young contrail is the wake's: two tubes of ice in the
// vortex pair, descending, bending in the Crow instability and pinching into rings, over a fainter curtain of ice left
// above them (the secondary wake); then CoCiP's plume (world/Cocip.cpp, after pycontrails): a Gaussian section - its
// covariance sigma_yy (across the track, horizontal), sigma_zz (vertical), sigma_yz - holding an extinction of E per
// metre of trail. Each part is ice the way takram's clouds are: its section's Gaussian, as coverage, eroded by their
// two noises (Perlin-Worley, Worley) into a sharp, turbulent edge, the part's extinction per metre kept on average;
// marched along the eye's ray through it, each step lit by the sun through the part (its Gaussian's optical depth
// towards the sun, in closed form) with takram's phase function and multiple scattering, by the same atmosphere,
// mapped the same way.

#define fsimClampMax vec2(1.0)
#include "fsim/prelude.glsl"
#include "fsim/atmosphere.glsl"

layout(push_constant) uniform PushConstants { mat4 projection; mat4 modelView; } pc;
// per sample, CONTRAIL_STRIDE rows (world::Airflow::Drawn): [0] the point of the flight path from the eye (ECEF axes,
// m), its sequence (< 0: none); [1] the plume's sigma_yy, sigma_zz, sigma_yz (m^2), its extinction per metre of trail
// (m); [2] the tubes' half spacing, the pair's descent, the Crow wave's amplitude (m), how far it has pinched into
// rings (0..1); [3] a tube's sigma (m), the distance flown (m), the primary wake's extinction per metre (both tubes),
// the plume's centre below the flight path (m); [4] b0 (m), the trail's phase, the secondary wake's extinction per
// metre, all three's; [5] the aircraft's up as it laid it (ECEF): the wake's tubes and curtain lie in its frame, banked
// with it (the plume's section is CoCiP's, horizontal and vertical), the plume's albedo (1: ice; a fire's soot less);
// [6] a fire's flame there: its soot's mean temperature (K, 0: none), fluctuating by its flicker (rms share). After the
// samples, the black body's table (blackbodyAt)
layout(std430, set = 0, binding = 0) readonly buffer Samples { vec4 samples[]; };
layout(set = 0, binding = 3) uniform sampler3D shapeTexture;       // takram's noises, mipmapped
layout(set = 0, binding = 4) uniform sampler3D shapeDetailTexture;
layout(set = 0, binding = 1) uniform Frame {
  vec4 F_eye;        // the eye (ECEF, m); the angle a pixel spans (rad)
  vec4 F_correction; // takram's altitude correction (m); their bottom radius (m)
  vec4 F_sun;        // towards the sun (ECEF); the time (s)
  vec4 F_night;      // the viewer's night light; the exposure
};

#define RECIPROCAL_PI4 0.07957747154594767
const float CROW_PLANE = 0.838;      // rad: the Crow mode's plane, 48 degrees from the horizontal (Crow 1970)
const float CROW_WAVELENGTH = 8.6;   // b0 (Crow 1970)

// takram's phase function and multiple scattering (packages/clouds/src/shaders/clouds.frag, MIT), their defaults
vec2 henyeyGreenstein(const vec2 g, const float cosTheta) {
  vec2 g2 = g * g;
  return RECIPROCAL_PI4 * ((1.0 - g2) / max(vec2(1e-7), pow(1.0 + g2 - 2.0 * g * cosTheta, vec2(1.5))));
}
float phaseFunction(const float cosTheta, const float attenuation) {
  const vec2 g = vec2(0.7, -0.2);
  const vec2 weights = vec2(0.5, 0.5);
  return dot(henyeyGreenstein(g * attenuation, cosTheta), weights);
}
float approximateMultipleScattering(const float opticalDepth, const float cosTheta) {
  vec3 coeffs = vec3(1.0);
  const vec3 attenuation = vec3(0.5, 0.5, 0.5);
  float scattering = 0.0;
  for (int i = 0; i < 8; ++i) {
    scattering += coeffs.x * exp(-opticalDepth * coeffs.y) * phaseFunction(cosTheta, coeffs.z);
    coeffs *= attenuation;
  }
  return scattering;
}

// The optical depth along a line through a Gaussian tube: its direction across the tube u (y, z) and how long a step
// along the line is across it (|u|, the rest along the trail), from a point p (y, z) on it relative to the tube's
// centre. The Gaussian's line integral, in closed form.
float plumeDepth(const vec3 sigma, const float E, const vec2 p, const vec2 u, const float alongLength) {
  if (E <= 0.0) return 0.0;
  float det = max(sigma.x * sigma.y - sigma.z * sigma.z, 1e-6);
  mat2 S = mat2(sigma.y, -sigma.z, -sigma.z, sigma.x) / det;
  float A0 = E / (6.283185307 * sqrt(det));
  float l = length(u);
  if (l < 1e-4) return A0 * exp(-0.5 * dot(p, S * p)) * alongLength;
  vec2 n = u / l;
  float a = dot(n, S * n), b = dot(n, S * p), c = dot(p, S * p);
  float peak = exp(-0.5 * max(c - b * b / a, 0.0));
  return min(A0 * sqrt(6.283185307 / a) * peak / l, A0 * peak * alongLength);
}

// A black body's light (world::SunlitBlackbody, its table BLACKBODY_ROWS rows from BLACKBODY_AT in the samples): its
// colour (linear sRGB at luminance 1) and its luminance against the sun's disk at T kelvin, 500 to 6000 K (the
// luminance between rows geometrically: it is steep in T)
vec4 blackbodyAt(const float T) {
  if (!(T >= 500.0)) return vec4(1.0, 1.0, 1.0, 0.0);
  float u = clamp((T - 500.0) / 5500.0, 0.0, 1.0) * float(BLACKBODY_ROWS - 1);
  int i0 = min(int(u), BLACKBODY_ROWS - 1), i1 = min(i0 + 1, BLACKBODY_ROWS - 1);
  float w = u - float(i0);
  vec4 a = samples[BLACKBODY_AT + i0], b = samples[BLACKBODY_AT + i1];
  vec4 c = mix(a, b, w);
  if (a.w > 0.0 && b.w > 0.0) c.w = a.w * pow(b.w / a.w, w);
  return c;
}

// A turbulent flame's light, against the sun's disk: its temperature T (its mean) fluctuating by `flicker` of it (rms);
// n the fluctuation here in deviations (the noise, as far as the pixel resolves it), `resolved` (0..1) how much of
// the fluctuation that is - the rest, finer than the pixel, its mean taken (three-point Gauss-Hermite)
vec3 flameLight(const float T, const float flicker, const float n, const float resolved) {
  if (!(T >= 500.0)) return vec3(0.0);
  float centre = T * (1.0 + flicker * resolved * n), spread = 1.7320508 * T * flicker * sqrt(max(1.0 - resolved * resolved, 0.0));
  vec4 a = blackbodyAt(centre), b = blackbodyAt(centre + spread), c = blackbodyAt(centre - spread);
  return (4.0 * a.w * a.rgb + b.w * b.rgb + c.w * c.rgb) / 6.0;
}
// the shape noise as deviations (takram's shape.bin: mean 0.80, deviation 0.076), and how much of it a read at this
// level of detail resolves
float noiseDeviations(const float shape) { return clamp((shape - 0.80) / 0.076, -3.0, 3.0); }
float noiseResolved(const float lod) { return 1.0 - smoothstep(0.0, 4.0, lod); }

// erfc, to 1e-3 (Abramowitz and Stegun 7.1.27)
float erfcApprox(float x) {
  float z = abs(x);
  float t = 1.0 + z * (0.278393 + z * (0.230389 + z * (0.000972 + z * 0.078108)));
  float e = 1.0 / (t * t * t * t);
  return x >= 0.0 ? e : 2.0 - e;
}

// The optical depth from a point p (y, z, relative to the Gaussian's centre) along a half-line, its direction across
// the tube u and how long a step along it is across it (|u|): the line's integral times the share of the Gaussian
// along it ahead of the point.
float halfLineDepth(const vec3 sigma, const float E, const vec2 p, const vec2 u, const float alongLength) {
  float full = plumeDepth(sigma, E, p, u, alongLength);
  float det = max(sigma.x * sigma.y - sigma.z * sigma.z, 1e-6);
  mat2 S = mat2(sigma.y, -sigma.z, -sigma.z, sigma.x) / det;
  float a = dot(u, S * u);
  if (a < 1e-8) return 0.5 * full;
  return full * 0.5 * erfcApprox(dot(u, S * p) / sqrt(2.0 * a));
}

// The coverage of a Gaussian part eroded by takram's noises: the share of the Gaussian above a threshold the noises
// set, `erosion` (0: the bare Gaussian .. 1) of it; and the mass that keeps against the Gaussian's, at the mean
// threshold (the Gaussian above t holds 1 - t ln(1/t) / (1 - t) of it).
const float EROSION_MEAN = 0.32;
// (the noises as takram ship them sit high and narrow - shape.bin's mean 0.80, its deviation 0.076; shape_detail.bin's
// 0.72, 0.095 - so each is spread to two deviations either side of a half first)
float erodedCoverage(const float gaussian, const float shape, const float detail, const float erosion) {
  float s = clamp((shape - 0.80) / 0.30 + 0.5, 0.0, 1.0), d = clamp((detail - 0.72) / 0.38 + 0.5, 0.0, 1.0);
  float t = erosion * clamp(0.62 * (1.0 - s) + 0.3 * (1.0 - d) - 0.12, 0.0, 0.9);
  return max(gaussian - t, 0.0) / (1.0 - t);
}
float erodedMass(const float erosion) {
  float t = max(erosion * EROSION_MEAN, 1e-4);
  return 1.0 - t * log(1.0 / t) / (1.0 - t);
}
