// fsim: a detonation (world::Detonations): the box round its fireball and its smoke, its far faces drawn (as a contrail
// segment's); and the light there - the sun's and the sky's from the atmosphere, the aerial perspective between it and
// the eye, and the sun's disk's luminance the fireball's glow is reckoned against.

#version 450
#include "contrail.glsl"

layout(location = 0) out vec3 v_pos;           // the fragment from the eye
layout(location = 1) flat out vec3 v_centre;   // the detonation's middle from the eye
layout(location = 2) flat out vec4 v_burst;    // the fireball's radius (m), its luminance against the sun's disk, the
                                               // smoke's sigma (m), its total extinction (m^2)
layout(location = 3) flat out vec4 v_chroma;   // the fireball's colour (linear sRGB at luminance 1), the erosion
layout(location = 4) flat out vec4 v_more;     // the soot's albedo, the noises' offset, the age (s)
layout(location = 5) flat out vec3 v_sunLight; // the sun's irradiance and the sky's radiance there, through the aerial
layout(location = 6) flat out vec3 v_skyLight; // perspective
layout(location = 7) flat out vec3 v_inscatter;
layout(location = 8) flat out vec3 v_solar;    // the sun's disk's luminance
layout(location = 9) flat out vec2 v_flame;    // a fuel fireball's: its flame's mean temperature (K), its flicker
out gl_PerVertex { vec4 gl_Position; };

const int STRIDE = DETONATION_STRIDE;

void main() {
  int i = gl_InstanceIndex * STRIDE;
  vec4 r0 = samples[i], r1 = samples[i + 1], r2 = samples[i + 2], r3 = samples[i + 3];
  if (r0.w < 0.0) { gl_Position = vec4(0.0, 0.0, -2.0, 1.0); return; }
  vec3 C = r0.xyz;
  // round all of it: the fireball, 3.2 sigma of smoke, and a pixel
  float h = max(1.2 * r1.x, 3.2 * r1.z) + 0.5 * F_eye.w * length(C);
  uint bit = 1u << uint(gl_VertexIndex);
  vec3 corner = vec3((0x287au & bit) != 0u ? 1.0 : 0.0, (0x02afu & bit) != 0u ? 1.0 : 0.0, (0x31e3u & bit) != 0u ? 1.0 : 0.0);
  vec3 pos = C + (2.0 * corner - 1.0) * h;

  vec3 sun = F_sun.xyz;
  vec3 at = (F_eye.xyz + C + F_correction.xyz) * METER_TO_LENGTH_UNIT;
  vec3 eye = (F_eye.xyz + F_correction.xyz) * METER_TO_LENGTH_UNIT;
  vec3 skyIrradiance;
  vec3 sunIrradiance = GetSunAndSkyScalarIrradiance(at, sun, skyIrradiance);
  vec3 transmittance;
  vec3 inscatter = GetSkyRadianceToPoint(eye, at, 0.0, sun, transmittance);
  // (the aerial perspective over a short path - a detonation some tens of metres off - is the difference of two large
  // table values, mostly their interpolation's error: a blue wash over a black puff. The air over the path scatters at
  // most (1 - its transmittance) of the light reaching it, the sun's and the sky's, into the eye's direction)
  inscatter = min(inscatter, (vec3(1.0) - transmittance) * (sunIrradiance + skyIrradiance) * RECIPROCAL_PI4 * 2.0);

  v_pos = pos;
  v_centre = C;
  v_burst = r1;
  v_chroma = r2;
  v_more = r3;
  v_sunLight = sunIrradiance * transmittance;
  v_skyLight = skyIrradiance * RECIPROCAL_PI4 * transmittance;
  v_inscatter = inscatter;
  v_solar = GetSolarLuminance() * transmittance;
  v_flame = samples[i + 4].xy;
  gl_Position = pc.projection * pc.modelView * vec4(pos, 1.0);
}
