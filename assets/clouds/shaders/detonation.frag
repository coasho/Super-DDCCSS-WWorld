// fsim: a detonation's pixel (world::Detonations): the eye's ray marched through its smoke - soot as a Gaussian puff,
// eroded by takram's noises into a billowing edge, its extinction kept on average, lit by the sun through the puff
// ahead (its Gaussian's optical depth in closed form) with takram's phase function and multiple scattering, darkened
// by the soot's albedo - and its fireball, an optically thick sphere of hot gas glowing at Planck's luminance for its
// temperature against the sun's disk; mapped as takram's clouds are (AgX at their exposure, the night light added).

#version 450
#include "contrail.glsl"

layout(location = 0) in vec3 v_pos;
layout(location = 1) flat in vec3 v_centre;
layout(location = 2) flat in vec4 v_burst;
layout(location = 3) flat in vec4 v_chroma;
layout(location = 4) flat in vec4 v_more;
layout(location = 5) flat in vec3 v_sunLight;
layout(location = 6) flat in vec3 v_skyLight;
layout(location = 7) flat in vec3 v_inscatter;
layout(location = 8) flat in vec3 v_solar;
layout(location = 9) flat in vec2 v_flame;
layout(location = 0) out vec4 outColor;

const int STEPS = 28;
const int OCTAVES = 4; // (takram's multiple scattering's first four: the rest add under 7 % in dark soot)
const float FIRE_DEPTH = 4.0; // the fireball's optical depth across its radius: opaque, its edge soft

// three.js r180 tonemapping_pars_fragment: AgX (as the clouds' composite.frag and contrail.frag)
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

void main() {
  vec3 d = normalize(v_pos);
  vec3 C = v_centre;
  float R = v_burst.x, fire = v_burst.y, sigma = max(v_burst.z, 0.05), E = v_burst.w;
  float erosion = v_chroma.w, albedo = v_more.x, seed = v_more.y;
  // soot absorbs blue more than red (an absorption Angstrom exponent near 1): its albedo a little higher in the red -
  // the puffs photographed brown-black
  vec3 albedoRgb = albedo * vec3(1.15, 1.0, 0.8);
  // the sky's light off an optically thick puff of it: an absorbing half-space reflects (1 - sqrt(1 - w)) /
  // (1 + sqrt(1 - w)) - 5.6 % at w = 0.2 - what is scattered deep inside is absorbed before it gets out
  vec3 root1 = sqrt(max(1.0 - albedoRgb, vec3(0.0)));
  vec3 skyReflect = (1.0 - root1) / (1.0 + root1);
  // the ray's stretch within the sphere round all of it
  float reach = max(1.15 * R, 3.2 * sigma);
  float b = dot(d, C), c2 = dot(C, C) - reach * reach, disc = b * b - c2;
  if (disc <= 0.0) discard;
  float root = sqrt(disc);
  float s0 = max(b - root, 0.0), s1 = b + root;
  if (s1 <= s0) discard;
  float ds = (s1 - s0) / float(STEPS);

  vec3 sun = F_sun.xyz;
  float cosTheta = dot(d, sun);
  float octavePhase[OCTAVES];
  float octave = 1.0;
  for (int i = 0; i < OCTAVES; ++i, octave *= 0.5) octavePhase[i] = octave * phaseFunction(cosTheta, octave);

  float sig2 = sigma * sigma;
  float A = E / (15.7496099 * sig2 * sigma * erodedMass(erosion)); // the Gaussian's peak, (2 pi)^1.5 sigma^3
  vec2 period = vec2(4.0, 1.0) * sigma;                             // the eddies grow with the puff
  vec3 emitted = v_solar * fire * v_chroma.rgb;
  vec3 radiance = vec3(0.0);
  float through = 1.0;
  for (int j = 0; j < STEPS; ++j) {
    float s = s0 + (float(j) + 0.5) * ds;
    vec3 x = d * s - C;
    float r2 = dot(x, x);
    float gaussian = exp(-0.5 * r2 / sig2);
    float ext = 0.0, shape = 0.8, lodShape = 8.0;
    // (the noise read for the fireball too: its edge, a fuel fireball's flicker - before its smoke shows)
    if ((gaussian > 1e-3 && E > 0.0) || (fire > 0.0 && r2 < 1.44 * R * R)) {
      vec3 at = x + vec3(seed, 0.37 * seed, 0.71 * seed) * sigma;
      float footprint = max(F_eye.w * s, ds);
      // (a fuel fireball's flicker read at the pixel's footprint alone: along the ray only its glowing surface counts)
      lodShape = max(log2((v_flame.y > 0.0 ? F_eye.w * s : footprint) * 128.0 / period.x), 0.0);
      shape = textureLod(shapeTexture, at / period.x, lodShape).r;
      if (gaussian > 1e-3 && E > 0.0) {
        float detail = textureLod(shapeDetailTexture, at / period.y, max(log2(footprint * 32.0 / period.y), 0.0)).r;
        ext = A * erodedCoverage(gaussian, shape, detail, erosion);
      }
    }
    // the fireball: inside its radius, its edge broken by the same noise
    float kf = 0.0;
    if (fire > 0.0 && R > 0.0) {
      float r = sqrt(r2) * (0.85 + 0.3 * clamp((shape - 0.65) / 0.3, 0.0, 1.0));
      kf = (1.0 - smoothstep(0.7 * R, R, r)) * FIRE_DEPTH / R;
    }
    float k = ext + kf;
    if (k <= 0.0) continue;
    float dtau = k * ds;
    // the sun through the smoke ahead of this point: the Gaussian's half-line integral
    float p = dot(x, sun), q2 = max(r2 - p * p, 0.0);
    float toSun = E / (6.283185307 * sig2) * exp(-0.5 * q2 / sig2) * 0.5 * erfcApprox(p / (1.414213562 * sigma));
    // (takram's octaves stand for the orders of scattering, for ice that scatters what it meets; soot absorbs most of
    // it at each: the n-th order is lit albedo^n as much - the puffs photographed near black, drawn mid-grey before)
    float scattering = 0.0, kept = albedo;
    octave = 1.0;
    for (int i = 0; i < OCTAVES; ++i, octave *= 0.5, kept *= albedo) scattering += kept * exp(-toSun * octave) * octavePhase[i];
    vec3 scattered = (v_sunLight * scattering) * (albedoRgb / albedo) + skyReflect * v_skyLight;
    // (a fuel fireball's flame hotter and cooler with its eddies: its mean light the black body's of its emissive power)
    vec3 glow = v_flame.y > 0.0 ? v_solar * flameLight(v_flame.x, v_flame.y, noiseDeviations(shape), noiseResolved(lodShape)) : emitted;
    vec3 source = (ext * scattered + kf * glow) / k;
    float absorbed = 1.0 - exp(-dtau);
    radiance += through * absorbed * source;
    through *= 1.0 - absorbed;
    if (through < 0.003) break;
  }
  float alpha = 1.0 - through;
  if (alpha < 1e-4) discard;
  vec3 perOpacity = radiance / alpha + v_inscatter;
  outColor = vec4((agx(perOpacity * F_night.w) + F_night.rgb) * alpha, alpha);
}
