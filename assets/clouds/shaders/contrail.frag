// fsim: a contrail segment's pixel (world::Airflow): the eye's ray marched through each part of the trail between the
// planes that end the segment - the wake's two tubes (bent by the Crow wave, pinched into rings), the curtain above
// them, CoCiP's plume - each part's ice its Gaussian eroded by takram's noises (contrail.glsl), each step lit by the
// sun through the part; the parts put together front to back; mapped as takram's clouds are (AgX at their exposure,
// the viewer's night light added).

#version 450
#include "contrail.glsl"

layout(location = 0) in vec3 v_pos;
layout(location = 1) flat in vec3 v_a;
layout(location = 2) flat in vec3 v_b;
layout(location = 3) flat in vec3 v_y;
layout(location = 4) flat in vec3 v_z;
layout(location = 5) flat in vec3 v_nA;
layout(location = 6) flat in vec3 v_nB;
layout(location = 7) flat in vec3 v_sunLight;
layout(location = 8) flat in vec3 v_skyLight;
layout(location = 9) flat in vec3 v_inscatter;
layout(location = 10) flat in vec4 v_plumeA;
layout(location = 11) flat in vec4 v_wakeA;
layout(location = 12) flat in vec4 v_moreA;
layout(location = 13) flat in vec4 v_plumeB;
layout(location = 14) flat in vec4 v_wakeB;
layout(location = 15) flat in vec4 v_moreB;
layout(location = 16) flat in vec4 v_trail;
layout(location = 17) flat in vec2 v_secondary;
layout(location = 18) flat in vec3 v_yWake;
layout(location = 19) flat in vec3 v_zWake;
layout(location = 20) flat in vec4 v_fire;
layout(location = 21) flat in vec2 v_flicker;
layout(location = 22) flat in vec3 v_solar;
layout(location = 0) out vec4 outColor;

const int STEPS[4] = int[4](10, 10, 6, 10); // per part: the tubes, the curtain, the plume
const float STRETCH = 3.0;                  // the eddies' length along the trail against their width (shear)

// three.js r180 tonemapping_pars_fragment: AgX (as the clouds' composite.frag)
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

// A part of the trail where it is a share f of the way along the segment: its section's centre (y across, z up, from
// the flight path) and covariance, its extinction per metre of trail, how much its edge is eroded, its noises'
// periods (m), and how far its centre moves along the segment (for the bounds)
struct Part {
  vec2 centre;
  vec3 sigma;
  float E, erosion;
  vec2 period;
};

// fsim: smooth noise along the trail, -1..1: a value per unit, eased between (a hash of the unit's index, no texture)
float trailNoise(const float x) {
  float i = floor(x), t = x - i;
  uint a = uint(int(i) & 0x7fffffff) * 747796405u + 2891336453u, b = a + 747796405u;
  a = ((a >> ((a >> 28u) + 4u)) ^ a) * 277803737u;
  b = ((b >> ((b >> 28u) + 4u)) ^ b) * 277803737u;
  float va = float((a >> 22u) ^ a) / 4294967295.0, vb = float((b >> 22u) ^ b) / 4294967295.0;
  return mix(va, vb, t * t * (3.0 - 2.0 * t)) * 2.0 - 1.0;
}

Part partAt(const int k, const float f) {
  vec4 plume = mix(v_plumeA, v_plumeB, f), wake = mix(v_wakeA, v_wakeB, f), more = mix(v_moreA, v_moreB, f);
  float b0 = max(v_trail.x, 1.0), tube = more.x;
  Part q;
  if (k < 2) {
    // a tube: the Crow wave bends the pair towards each other in planes 48 degrees from the horizontal; where they
    // touched, the rings gather the ice (the mean along the trail kept). fsim: the wave grows from the air's
    // turbulence, so it is not one sinusoid but the band of wavelengths the instability amplifies round its most
    // unstable one (observed some 6 - 12 b0): three of them here, 0.7, 1 and 1.4 times 8.6 b0, each its phase wandering
    // and its amplitude coming and going along the trail on its own, and the whole grown 40 - 100 % of the way to
    // touching - so the pair bends, links and rings irregularly. Drawn at one wavelength (even wandering 25 % either
    // way), its loops repeated down the trail
    float side = k == 0 ? -1.0 : 1.0;
    float lambda = CROW_WAVELENGTH * b0;
    float wave = 0.0, weights = 0.0;
    for (int m = 0; m < 3; ++m) {
      float lm = lambda * (m == 0 ? 1.0 : (m == 1 ? 0.7 : 1.4));
      float am = 0.5 + 0.5 * trailNoise(more.y / (1.1 * lambda) + 31.0 * float(m) + 5.0);
      float ph = 6.2831853 * more.y / lm + v_trail.y * float(m + 1) + 3.0 * trailNoise(more.y / (1.3 * lambda) + 17.0 * float(m));
      wave += am * sin(ph);
      weights += am;
    }
    wave /= max(weights, 1e-3);
    float grown = 0.7 + 0.3 * trailNoise(more.y / (2.0 * lambda) + 47.0); // (at most touching, as drawn)
    vec2 bend = wake.z * grown * wave * vec2(cos(CROW_PLANE), sin(CROW_PLANE));
    float lump = 0.5 + 0.5 * wave;
    q.centre = vec2(side * (wake.x - bend.x), -wake.y + bend.y);
    q.sigma = vec3(tube * tube, tube * tube, 0.0);
    q.E = more.z * 0.5 * mix(1.0, lump * lump * (8.0 / 3.0), wake.w);
    q.erosion = 1.0;
    q.period = vec2(2.5, 0.6) * b0;
  } else if (k == 2) {
    // the curtain: from the flight path down to the pair, thinly
    float halfDepth = max(0.5 * wake.y, tube);
    q.centre = vec2(0.0, -halfDepth);
    q.sigma = vec3(2.25 * tube * tube, 0.35 * halfDepth * halfDepth, 0.0);
    q.E = mix(v_secondary.x, v_secondary.y, f);
    q.erosion = 0.8;
    q.period = vec2(2.5, 0.6) * b0;
  } else {
    // CoCiP's plume, as the wake dissolves into it; its eddies grow with it
    float scale = max(sqrt(min(plume.x, plume.y)), 1.0);
    q.centre = vec2(0.0, -more.w);
    q.sigma = plume.xyz;
    q.E = plume.w;
    q.erosion = 0.7;
    q.period = vec2(5.0, 1.25) * max(scale, 0.5 * b0);
    // (a fire's smoke, a metre across where it leaves: its eddies its own size, as the wake's are b0's)
    if (min(v_fire.x, v_fire.y) < 1.0) q.period = vec2(2.5, 0.6) * max(sqrt(min(plume.x, plume.y)), 0.2);
  }
  return q;
}

// the ray's stretch [s0, s1] within the ellipse of `reach` sigmas round a part's centre, from p0 + u s
bool within(const vec3 sigma, const vec2 c, const vec2 p0, const vec2 u, const float reach, inout float s0, inout float s1) {
  float det = max(sigma.x * sigma.y - sigma.z * sigma.z, 1e-8);
  mat2 S = mat2(sigma.y, -sigma.z, -sigma.z, sigma.x) / det;
  vec2 q = p0 - c;
  float a = dot(u, S * u), b = dot(u, S * q), cc = dot(q, S * q) - reach * reach;
  if (a < 1e-10) return cc <= 0.0;
  float disc = b * b - a * cc;
  if (disc <= 0.0) return false;
  float r = sqrt(disc);
  s0 = max(s0, (-b - r) / a);
  s1 = min(s1, (-b + r) / a);
  return s1 > s0;
}

void main() {
  vec3 d = normalize(v_pos);
  // the segment's slab: past the plane through v_a, short of the one through v_b (the eye at the origin)
  float s0 = 0.0, s1 = 1e7;
  float dA = dot(d, v_nA), hA = dot(v_a, v_nA), dB = dot(d, v_nB), hB = dot(v_b, v_nB);
  if (abs(dA) > 1e-7) { if (dA > 0.0) s0 = max(s0, hA / dA); else s1 = min(s1, hA / dA); } else if (hA > 0.0) discard;
  if (abs(dB) > 1e-7) { if (dB > 0.0) s1 = min(s1, hB / dB); else s0 = max(s0, hB / dB); } else if (hB < 0.0) discard;
  if (s1 <= s0) discard;

  vec3 axis = v_b - v_a;
  float L = length(axis);
  vec3 T = axis / L;
  float b = dot(d, T);
  // where it passes closest to the segment, for the parts' bounds
  float sc = 0.5 * (s0 + s1);
  if (1.0 - b * b > 1e-4) sc = clamp((dot(d, v_a) - b * dot(T, v_a)) / (1.0 - b * b), s0, s1);
  float fc = clamp(dot(d * sc - v_a, T) / L, 0.0, 1.0);

  // the light: takram's phase function and multiple scattering, the phase part of each octave once a pixel
  vec3 sun = F_sun.xyz;
  float sunAlong = 1.0 / max(abs(dot(sun, T)), 1e-3);
  float cosTheta = dot(d, sun);
  float octavePhase[8];
  float octave = 1.0;
  for (int i = 0; i < 8; ++i, octave *= 0.5) octavePhase[i] = octave * phaseFunction(cosTheta, octave);
  // a fire's smoke: soot, not ice (its albedo below 1). It scatters as the detonation's soot does (world::Detonations:
  // the n-th order of scattering lit albedo^n as much, the sky's light off it as off an absorbing half-space, a little
  // redder than it is blue), and its flame, the young soot hot, glows as much as it absorbs (Kirchhoff): (1 - albedo)
  // of Planck's light at its temperature. Ice (albedo 1) as before
  bool fire = min(v_fire.x, v_fire.y) < 1.0;
  vec3 tint = fire ? vec3(1.15, 1.0, 0.8) : vec3(1.0);

  float tauPart[4], sMid[4];
  vec3 lightPart[4];
  for (int k = 0; k < 4; ++k) {
    tauPart[k] = 0.0;
    lightPart[k] = vec3(0.0);
    sMid[k] = 0.0;
    // the ray in the part's section: from the flight path at v_a, across (y) and up (z) - the wake's in the frame the
    // aircraft was banked in, the plume's level
    vec3 Yk = k < 3 ? v_yWake : v_y, Zk = k < 3 ? v_zWake : v_z;
    vec2 p0 = vec2(-dot(v_a, Yk), -dot(v_a, Zk));
    vec2 u = vec2(dot(d, Yk), dot(d, Zk));
    vec2 uSun = vec2(dot(sun, Yk), dot(sun, Zk));
    Part q = partAt(k, fc);
    // (bounds wide enough for the part anywhere along the segment: its ends, the Crow wave's reach)
    Part qa = partAt(k, 0.0), qb = partAt(k, 1.0);
    if (max(qa.E, qb.E) <= 0.0) continue;
    float pixel = 0.5 * F_eye.w * sc;
    float spread = 0.5 * length(qa.centre - qb.centre) + (k < 2 ? max(v_wakeA.z, v_wakeB.z) : 0.0) + pixel;
    vec3 bound = max(qa.sigma, qb.sigma) + vec3(spread * spread, spread * spread, 0.0);
    float e0 = s0, e1 = s1;
    if (!within(bound, mix(qa.centre, qb.centre, 0.5), p0, u, 3.2, e0, e1)) continue;
    int n = STEPS[k];
    float ds = (e1 - e0) / float(n);
    float transmittance = 1.0;
    for (int j = 0; j < n; ++j) {
      float s = e0 + (float(j) + 0.5) * ds;
      vec3 x = d * s;
      float f = clamp(dot(x - v_a, T) / L, 0.0, 1.0);
      q = partAt(k, f);
      if (q.E <= 0.0) continue;
      // the pixel's footprint widens a part thinner than it (its extinction kept), and takes its edge's erosion
      float sigmaPixel = 0.5 * F_eye.w * s;
      vec3 sigma = q.sigma + vec3(sigmaPixel * sigmaPixel, sigmaPixel * sigmaPixel, 0.0);
      float det = max(sigma.x * sigma.y - sigma.z * sigma.z, 1e-8);
      mat2 S = mat2(sigma.y, -sigma.z, -sigma.z, sigma.x) / det;
      vec2 rel = p0 + u * s - q.centre;
      float gaussian = exp(-0.5 * dot(rel, S * rel));
      if (gaussian < 1e-3) continue;
      float thinnest = sqrt(min(q.sigma.x, q.sigma.y));
      float erosion = q.erosion * (1.0 - smoothstep(0.25, 1.0, sigmaPixel / max(thinnest, 1e-3)));
      float coverage = gaussian;
      // (a fire's flame: its temperature fluctuating with the same eddies, read however far)
      float flameK = k == 3 && fire ? mix(v_fire.z, v_fire.w, f) : 0.0;
      float shape = 0.80, lodShape = 8.0;
      if (erosion > 0.01 || flameK > 0.0) {
        // takram's noises, moving with the part, read at the level the step and the pixel take in
        float along = mix(v_moreA.y, v_moreB.y, f);
        // (the point read drifts across the noise as the trail goes on: read along one axis only, the tiles came
        // round every few periods, the same eddies again and again down the trail)
        vec3 at = vec3(along / STRETCH, rel + vec2(0.29, 0.17) * (along / STRETCH)) + vec3(0.0, 31.0 * float(k), 17.0 * float(k));
        // (a step's length in the noises' space: along the trail they are stretched)
        float footprint = max(F_eye.w * s, ds * length(vec3(b / STRETCH, u)));
        // (a flame's flicker read at the pixel's footprint alone: along the ray only its glowing surface counts)
        lodShape = max(log2((flameK > 0.0 ? F_eye.w * s : footprint) * 128.0 / q.period.x), 0.0);
        shape = textureLod(shapeTexture, at / q.period.x, lodShape).r;
        if (erosion > 0.01) {
          float detail = textureLod(shapeDetailTexture, at / q.period.y, max(log2(footprint * 32.0 / q.period.y), 0.0)).r;
          coverage = erodedCoverage(gaussian, shape, detail, erosion);
        }
      }
      if (coverage <= 0.0) continue;
      float extinction = q.E / (6.283185307 * sqrt(det) * erodedMass(erosion)) * coverage;
      float dtau = extinction * ds;
      // the sun through the part ahead of this point; the light scattered here, through what is in front of it
      float toSun = halfLineDepth(sigma, q.E, rel, uSun, sunAlong);
      float scattering = 0.0;
      octave = 1.0;
      vec3 light;
      if (k == 3 && fire) {
        float albedo = mix(v_fire.x, v_fire.y, f), kept = albedo;
        for (int i = 0; i < 8; ++i, octave *= 0.5, kept *= albedo) scattering += kept * exp(-toSun * octave) * octavePhase[i];
        vec3 root1 = sqrt(max(1.0 - albedo * tint, vec3(0.0)));
        vec3 skyReflect = (1.0 - root1) / (1.0 + root1);
        vec3 flame = flameLight(flameK, v_flicker.x, noiseDeviations(shape), noiseResolved(lodShape));
        light = v_sunLight * scattering * tint + v_skyLight * skyReflect + (1.0 - albedo) * flame * v_solar;
      } else {
        for (int i = 0; i < 8; ++i, octave *= 0.5) scattering += exp(-toSun * octave) * octavePhase[i];
        light = v_sunLight * scattering + v_skyLight;
      }
      float absorbed = 1.0 - exp(-dtau);
      lightPart[k] += transmittance * absorbed * light;
      transmittance *= 1.0 - absorbed;
      tauPart[k] += dtau;
    }
    sMid[k] = 0.5 * (e0 + e1);
  }

  // the parts front to back
  int order[4] = int[4](0, 1, 2, 3);
  for (int i = 1; i < 4; ++i)
    for (int j = i; j > 0 && sMid[order[j]] < sMid[order[j - 1]]; --j) {
      int t = order[j];
      order[j] = order[j - 1];
      order[j - 1] = t;
    }
  vec3 radiance = vec3(0.0);
  float through = 1.0;
  for (int i = 0; i < 4; ++i) {
    int k = order[i];
    radiance += through * lightPart[k];
    through *= exp(-tauPart[k]);
  }
  float alpha = 1.0 - through;
  if (alpha < 1e-4) discard;
  // (mapped per opacity, as the clouds' composite maps theirs)
  vec3 perOpacity = radiance / alpha + v_inscatter;
  outColor = vec4((agx(perOpacity * F_night.w) + F_night.rgb) * alpha, alpha);
}
