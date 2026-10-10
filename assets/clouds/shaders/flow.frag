// fsim: NVIDIA Flow's fire and smoke (world::FlowVolumes), a pixel's: the eye's ray marched through a page's box -
// radiative transfer through soot. Its extinction is the solver's smoke times the soot's (per unit), of which its
// albedo scatters and the rest absorbs; what absorbs also emits (Kirchhoff), Planck's light at the gas's temperature
// against the sun's disk - the fire's light is its hot soot's, nothing painted on. What scatters is lit by the sun
// through the smoke towards it and by the sky through the smoke above (both marched in the fields), with takram's
// phase function. Leaves not drawn are stepped over. Mapped as takram's clouds are (AgX at their exposure, the night
// light added).

#version 450
#include "flow.glsl"

layout(location = 0) in vec3 v_pos;
layout(location = 1) flat in vec3 v_lo;
layout(location = 2) flat in vec3 v_size;
layout(location = 3) flat in vec3 v_sunLight;
layout(location = 4) flat in vec3 v_skyLight;
layout(location = 5) flat in vec3 v_inscatter;
layout(location = 6) flat in vec3 v_solar;
layout(location = 0) out vec4 outColor;

const int STEPS = 160;      // along the eye's ray through the page, at most (its leaves not drawn stepped over)
const int SUN_STEPS = 5;    // towards the sun, over SUN_REACH m
const int SKY_STEPS = 3;    // upwards (the sky's light comes mostly from above), over SKY_REACH m
const float SUN_REACH = 24.0, SKY_REACH = 12.0;

// three.js r180 tonemapping_pars_fragment: AgX (as detonation.frag's)
vec3 agx(vec3 color) {
  const mat3 LINEAR_SRGB_TO_LINEAR_REC2020 = mat3(vec3(0.6274, 0.0691, 0.0164), vec3(0.3293, 0.9195, 0.0880), vec3(0.0433, 0.0113, 0.8956));
  const mat3 LINEAR_REC2020_TO_LINEAR_SRGB = mat3(vec3(1.6605, -0.1246, -0.0182), vec3(-0.5876, 1.1329, -0.1006), vec3(-0.0728, -0.0083, 1.1187));
  const mat3 AgXInsetMatrix = mat3(vec3(0.856627153315983, 0.137318972929847, 0.11189821299995),
                                   vec3(0.0951212405381588, 0.761241990602591, 0.0767994186031903),
                                   vec3(0.0482516061458583, 0.101439036467562, 0.811302368396859));
  const mat3 AgXOutsetMatrix = mat3(vec3(1.1271005818144368, -0.1413297634984383, -0.14132976349843826),
                                    vec3(-0.11060664309660323, 1.157823702216272, -0.11060664309660294),
                                    vec3(-0.016493938717834573, -0.016493938717834257, 1.2519364065950405));
  const float AgxMinEv = -12.47393, AgxMaxEv = 4.026069;
  color = LINEAR_SRGB_TO_LINEAR_REC2020 * color;
  color = AgXInsetMatrix * color;
  color = max(color, 1e-10);
  color = log2(color);
  color = (color - AgxMinEv) / (AgxMaxEv - AgxMinEv);
  color = clamp(color, 0.0, 1.0);
  vec3 x2 = color * color, x4 = x2 * x2;
  color = 15.5 * x4 * x2 - 40.14 * x4 * color + 31.96 * x4 - 6.868 * x2 * color + 0.4298 * x2 + 0.1191 * color - 0.00232;
  color = AgXOutsetMatrix * color;
  color = pow(max(vec3(0.0), color), vec3(2.2));
  color = LINEAR_REC2020_TO_LINEAR_SRGB * color;
  return clamp(color, 0.0, 1.0);
}

// takram's phase function (packages/clouds/src/shaders/clouds.frag, MIT), as the detonations' and the contrails'
vec2 henyeyGreenstein(const vec2 g, const float cosTheta) {
  vec2 g2 = g * g;
  return RECIPROCAL_PI4 * ((1.0 - g2) / max(vec2(1e-7), pow(1.0 + g2 - 2.0 * g * cosTheta, vec2(1.5))));
}
float phaseFunction(const float cosTheta) { return dot(henyeyGreenstein(vec2(0.7, -0.2), cosTheta), vec2(0.5)); }

// a box [lo, lo + size]: where a ray from o along d is in it
vec2 slab(vec3 o, vec3 d, vec3 lo, vec3 size) {
  vec3 inv = 1.0 / d;
  vec3 a = (lo - o) * inv, b = (lo + size - o) * inv;
  vec3 near = min(a, b), far = max(a, b);
  return vec2(max(max(near.x, near.y), max(near.z, 0.0)), min(min(far.x, far.y), far.z));
}

void main() {
  mat3 axes = mat3(samples[1].xyz, samples[2].xyz, samples[3].xyz); // the solver's frame to ECEF (orthonormal)
  mat3 toLocal = transpose(axes);
  float voxel = samples[0].w;
  vec3 zero = samples[4].xyz;
  float extinction = samples[5].x, albedo = samples[5].y, unitK = samples[5].z, airK = samples[5].w;
  vec4 flicker = samples[6];
  vec3 o = -(toLocal * samples[0].xyz); // the eye in the solver's frame
  vec3 d = normalize(toLocal * v_pos);
  vec2 t = slab(o, d, v_lo, v_size);
  if (t.y <= t.x) discard;
  float ds = 0.75 * voxel;
  // (each pixel's samples offset by its own fraction of a step, a hash of the pixel and the frame: banding becomes
  // fine noise, without a pattern of its own)
  uvec3 q = uvec3(uvec2(gl_FragCoord.xy), uint(F_sun.w * 60.0));
  q = q * 1664525u + 1013904223u;
  q.x += q.y * q.z; q.y += q.z * q.x; q.z += q.x * q.y;
  q ^= q >> 16u;
  q.x += q.y * q.z;
  float jitter = float(q.x & 0xffffffu) / 16777216.0;

  vec3 sunL = normalize(toLocal * F_sun.xyz);
  const vec3 upL = vec3(0.0, 0.0, 1.0);
  float phase = phaseFunction(dot(d, sunL));
  // soot absorbs blue more than red (an absorption Angstrom exponent near 1): the puffs brown-black (detonation.frag's)
  vec3 albedoRgb = albedo * vec3(1.15, 1.0, 0.8);
  float leafM = 8.0 * voxel;

  vec3 radiance = vec3(0.0);
  float through = 1.0;
  float s = t.x + jitter * ds;
  for (int j = 0; j < STEPS && s < t.y; ++j) {
    vec3 p = o + d * s;
    vec3 u = (p - zero) / voxel;
    bool drawn;
    vec2 v = flowAt(u, drawn);
    if (!drawn) {
      // over the leaf: to where the ray leaves it
      vec3 lo = zero + vec3(ivec3(floor(u)) >> 3) * leafM;
      s = max(slab(o, d, lo, vec3(leafM)).y, s) + 0.5 * ds;
      continue;
    }
    float k = extinction * max(v.x, 0.0);
    if (k > 1e-5) {
      float dtau = k * ds;
      // the sun through the smoke towards it, the sky through the smoke above
      float tauSun = 0.0;
      for (int i = 0; i < SUN_STEPS; ++i)
        tauSun += extinction * max(flowAt((p + sunL * (SUN_REACH * (float(i) + 0.5) / float(SUN_STEPS)) - zero) / voxel).r, 0.0);
      tauSun *= SUN_REACH / float(SUN_STEPS);
      float tauSky = 0.0;
      for (int i = 0; i < SKY_STEPS; ++i)
        tauSky += extinction * max(flowAt((p + upL * (SKY_REACH * (float(i) + 0.5) / float(SKY_STEPS)) - zero) / voxel).r, 0.0);
      tauSky *= SKY_REACH / float(SKY_STEPS);
      vec3 scattered = albedoRgb * (v_sunLight * phase * exp(-tauSun) + v_skyLight * exp(-tauSky));
      // what absorbs, emits: Planck's light at the gas's temperature - where it burns, fluctuating about its mean
      // below the cells (two of the noise's reads drifting apart as the flame rises: its eddies evolving)
      float theta = max(v.y, 0.0);
      float burning = smoothstep(0.45, 0.95, theta);
      float T = airK + unitK * theta;
      if (burning > 0.0) {
        // (read at the pixel's footprint: what it does not resolve is its mean, the fluctuation's share it does)
        vec3 q0 = p / flicker.z;
        float lod = max(log2(F_eye.w * length(p - o) * 128.0 / flicker.z), 0.0);
        float n0 = textureLod(flowNoise, q0 + vec3(0.0, 0.0, -0.8 * flicker.w / flicker.z), lod).r;
        float n1 = textureLod(flowNoise, q0 * 1.7 + vec3(0.37, 0.71, -1.9 * flicker.w / flicker.z), lod + 0.77).r;
        float n = clamp(((n0 - 0.80) + (n1 - 0.80)) / (0.076 * 1.41421356), -3.0, 3.0);
        float resolved = 1.0 - smoothstep(0.0, 4.0, lod);
        T = mix(T, T * flicker.y * (1.0 + flicker.x * resolved * n), burning);
      }
      vec4 bb = blackbodyAt(T);
      vec3 source = scattered + (1.0 - albedo) * v_solar * bb.w * bb.rgb;
      float absorbed = 1.0 - exp(-dtau);
      radiance += through * absorbed * source;
      through *= 1.0 - absorbed;
      if (through < 0.003) break;
    }
    s += ds;
  }
  float alpha = 1.0 - through;
  if (alpha < 1e-4) discard;
  vec3 perOpacity = radiance / alpha + v_inscatter;
  outColor = vec4((agx(perOpacity * F_night.w) + F_night.rgb) * alpha, alpha);
}
