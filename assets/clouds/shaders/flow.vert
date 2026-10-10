// fsim: NVIDIA Flow's fire and smoke (world::FlowVolumes): a page's box (an instance a page), its far faces drawn; and
// the light there - the sun's and the sky's from the atmosphere, the aerial perspective between it and the eye, and the
// sun's disk's luminance the hot soot's glow is reckoned against (as a detonation's, detonation.vert).

#version 450
#include "flow.glsl"

layout(location = 0) out vec3 v_pos;           // the fragment from the eye (ECEF axes, m)
layout(location = 1) flat out vec3 v_lo;       // the page's box (the solver's frame, m)
layout(location = 2) flat out vec3 v_size;
layout(location = 3) flat out vec3 v_sunLight; // the sun's irradiance and the sky's radiance there, through the aerial
layout(location = 4) flat out vec3 v_skyLight; // perspective
layout(location = 5) flat out vec3 v_inscatter;
layout(location = 6) flat out vec3 v_solar;    // the sun's disk's luminance
out gl_PerVertex { vec4 gl_Position; };

void main() {
  vec4 box = samples[PAGES_AT + 2 * gl_InstanceIndex];
  vec3 size = samples[PAGES_AT + 2 * gl_InstanceIndex + 1].xyz;
  mat3 axes = mat3(samples[1].xyz, samples[2].xyz, samples[3].xyz); // the solver's frame to ECEF
  vec3 origin = samples[0].xyz;
  uint bit = 1u << uint(gl_VertexIndex);
  vec3 corner = vec3((0x287au & bit) != 0u ? 1.0 : 0.0, (0x02afu & bit) != 0u ? 1.0 : 0.0, (0x31e3u & bit) != 0u ? 1.0 : 0.0);
  vec3 pos = origin + axes * (box.xyz + corner * size);
  vec3 C = origin + axes * (box.xyz + 0.5 * size);

  vec3 sun = F_sun.xyz;
  vec3 at = (F_eye.xyz + C + F_correction.xyz) * METER_TO_LENGTH_UNIT;
  vec3 eye = (F_eye.xyz + F_correction.xyz) * METER_TO_LENGTH_UNIT;
  vec3 skyIrradiance;
  vec3 sunIrradiance = GetSunAndSkyScalarIrradiance(at, sun, skyIrradiance);
  vec3 transmittance;
  vec3 inscatter = GetSkyRadianceToPoint(eye, at, 0.0, sun, transmittance);
  // (over a short path the tables' difference is mostly their interpolation's error: bounded as detonation.vert's)
  inscatter = min(inscatter, (vec3(1.0) - transmittance) * (sunIrradiance + skyIrradiance) * RECIPROCAL_PI4 * 2.0);

  v_pos = pos;
  v_lo = box.xyz;
  v_size = size;
  v_sunLight = sunIrradiance * transmittance;
  v_skyLight = skyIrradiance * RECIPROCAL_PI4 * transmittance;
  v_inscatter = inscatter;
  v_solar = GetSolarLuminance() * transmittance;
  gl_Position = pc.projection * pc.modelView * vec4(pos, 1.0);
}
