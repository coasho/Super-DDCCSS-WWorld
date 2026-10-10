// fsim: the sea, drawn over the ground wherever the ground is the sea's floor (world::Clouds, after the clouds' shade
// on the ground and before the vehicles). One pass over the frame: each pixel's ray meets the WGS84 ellipsoid (mean sea
// level, as the terrain has it), and the sea there is lit as Bruneton, Neyret & Holzschuch light it ("Real-time
// Realistic Ocean Lighting using Seamless Transitions from Geometry to BRDF", Eurographics 2010; Proland, INRIA, BSD-3,
// see LICENSE-Proland.txt):
//
// - The waves are GodotOceanWaves' FFT ocean (ocean/spectrum.comp ...): three cascades, each a band of the spectrum,
//   their slopes and squared slopes read at the level the pixel's footprint asks for (LEAN mapping). What a pixel holds
//   is a mean slope (its normal) and the variance round it (its roughness), so a sea seen from far off is as rough as
//   it is near, only finer than a pixel: the sun's glitter becomes the broad glint it is from the air, with no aliasing
//   and no tiling. What is finer than the smallest cascade (capillaries) comes from Cox & Munk's measured slope variance
//   for the wind.
// - The light: the sky reflected (Bruneton's atmosphere, the same tables the clouds are lit by) with the Fresnel
//   averaged over the slopes; the sun reflected by the waves' slope distribution (an anisotropic Beckmann BRDF with
//   Smith's masking), dimmed by the clouds' shadow; light scattered up out of deep water; the foam where crests break.
// - The sea's floor: GEBCO 2026's water depth (tools/ocean_map) - the imagery, which shows shallow water's colours as
//   they are, is seen through the sea as far as the depth lets light down and back (Jerlov's clear ocean water, its
//   reflectance and attenuation), so reefs and banks keep their colours while the deep ocean is the sea's own.
// - Seen as the land beside it is: lit by the terrain's light (world::setDaylight) as a surface of its apparent
//   reflectance, and hazed by the terrain's aerial perspective (world/Scattering.cpp).
// - The shore: the terrain's sea floor is drawn below sea level (world::setSeaFloorDepth) and the sea is depth-tested
//   between the two, so the land at the shore hides it however low it lies.
//
// Blended as out = S + frame * A: S the sea's own light, A how much of the imagery under it is still seen. Drawn on
// ocean.vert's grid: near the eye its waves stand up (the grid's points, interpolated, are where the sea is); farther
// off each pixel meets the ellipsoid itself, the waves flat by then and the grid's triangles kilometres wide.

#version 450

#define fsimClampMax P_screen.zw
#define SHADOW_CASCADE_COUNT 3

#include "clouds/types.glsl"
#include "fsim/params.glsl"
#include "fsim/prelude.glsl"
#include "fsim/atmosphere.glsl"
#include "core/math.glsl"
#include "core/raySphereIntersection.glsl"
#define OCEAN_SHORE_MAP
#include "ocean/ocean.glsl"

layout(set = 0, binding = 9) uniform sampler2DArray shadowBuffer; // the clouds' shadow map, this frame's
layout(set = 0, binding = 21) uniform sampler2DArray slopeMap;     // each cascade's sx, sz, sx^2, sz^2, mipmapped
layout(set = 0, binding = 22) uniform sampler2DArray foamMap;      // each cascade's foam, mipmapped
layout(set = 0, binding = 23) uniform sampler2D seaMap;            // GEBCO: the depth (log-coded), the water's share
layout(set = 0, binding = 31) uniform sampler2D cloudsResolved;    // the clouds this frame (full size, premultiplied)
layout(location = 0) in vec3 vSurface;  // ocean.vert's: the undisplaced point (from the eye, m)
layout(location = 1) in vec3 vPosition; // and where the waves have moved it
layout(location = 2) in float vMiss;    // over 0 in a triangle reaching past the horizon
layout(location = 0) out vec4 outColor;

#include "fsim/cloudShadow.glsl"

// Value noise (a hash per lattice point, smoothly interpolated) and three octaves of it: the sea's large-scale
// patches of rougher and smoother water
float hash12(vec2 p) {
  vec3 q = fract(vec3(p.xyx) * 0.1031);
  q += dot(q, q.yzx + 33.33);
  return fract((q.x + q.y) * q.z);
}
float valueNoise(vec2 x) {
  vec2 i = floor(x), f = fract(x);
  vec2 u = f * f * (3.0 - 2.0 * f);
  return mix(mix(hash12(i), hash12(i + vec2(1.0, 0.0)), u.x), mix(hash12(i + vec2(0.0, 1.0)), hash12(i + vec2(1.0, 1.0)), u.x), u.y);
}
float patches(vec2 x) {
  return 0.55 * valueNoise(x) + 0.3 * valueNoise(x * 2.7 + 17.0) + 0.15 * valueNoise(x * 7.3 + 41.0);
}

// The clouds a reflected ray sees, from the frame's own clouds (their resolve, in the sky's units, premultiplied by
// their opacity): the direction is taken to the screen, and read there smeared along the screen's vertical by the
// spread the waves' slopes give the reflection (sigma, rad). How much of it the screen holds in .w (0 off it).
vec4 reflectedClouds(const vec3 r, const float sigma, out float onScreen) {
  onScreen = 0.0;
  vec4 clip = P_projection * (P_view * vec4(r, 0.0));
  if (clip.w <= 1e-6) return vec4(0.0);
  vec2 uv = clip.xy / clip.w * 0.5 + 0.5;
  vec2 inside = min(uv, 1.0 - uv);
  onScreen = smoothstep(0.0, 0.04, min(inside.x, inside.y));
  if (onScreen <= 0.0) return vec4(0.0);
  float pixelsPerRadian = 0.5 * P_screen.y * abs(P_projection[1][1]);
  float spread = min(2.0 * sigma * pixelsPerRadian, 0.25 * P_screen.y);
  vec2 pixel = uv * P_screen.xy;
  // (nine taps a Gaussian apart, staggered sideways a little: five, a quarter of the spread apart each, drew each
  // cloud's edge again and again down the streak)
  vec4 sum = vec4(0.0);
  float total = 0.0;
  for (int i = -4; i <= 4; ++i) {
    float f = float(i) / 4.0, w = exp(-2.0 * f * f);
    vec2 at = clamp(pixel + vec2(0.15 * spread * ((i & 1) == 0 ? 0.5 : -0.5), f * spread), vec2(0.5), P_screen.xy - 0.5);
    sum += w * texture(cloudsResolved, at / P_alloc.zw);
    total += w;
  }
  return sum / total;
}

// erfc (Abramowitz & Stegun 7.1.26, to 1.5e-7) and its inverse on (0, 1] (Giles 2010's single-precision erfinv)
float erfcApprox(const float x) {
  float z = abs(x), t = 1.0 / (1.0 + 0.3275911 * z);
  float y = t * (0.254829592 + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429)))) * exp(-z * z);
  return x >= 0.0 ? y : 2.0 - y;
}
float erfinvApprox(const float x) {
  float w = -log(max((1.0 - x) * (1.0 + x), 1e-30)), p;
  if (w < 5.0) {
    w -= 2.5;
    p = 2.81022636e-08;
    p = 3.43273939e-07 + p * w; p = -3.5233877e-06 + p * w; p = -4.39150654e-06 + p * w; p = 0.00021858087 + p * w;
    p = -0.00125372503 + p * w; p = -0.00417768164 + p * w; p = 0.246640727 + p * w; p = 1.50140941 + p * w;
  } else {
    w = sqrt(w) - 3.0;
    p = -0.000200214257;
    p = 0.000100950558 + p * w; p = 0.00134934322 + p * w; p = -0.00367342844 + p * w; p = 0.00573950773 + p * w;
    p = -0.0076224613 + p * w; p = 0.00943887047 + p * w; p = 1.00167406 + p * w; p = 2.83297682 + p * w;
  }
  return p * x;
}

// Highlights rolled off rather than clipped at white
vec3 shoulder(vec3 x) {
  const float knee = 0.7;
  return mix(x, knee + (1.0 - knee) * (1.0 - exp(-(x - knee) / (1.0 - knee))), step(knee, x));
}

// Clear ocean water (Jerlov type I): the reflectance of water too deep to see the bottom through, just under the
// surface (Morel 1988's R(0-) for chlorophyll ~0.05 mg/m3 at 650, 550, 450 nm), and the depth over which the light
// the floor sends back up is lost (down and up again: 1 / 2Kd, Kd at 490 nm ~0.03/m, red and green faster).
const vec3 DEEP_REFLECTANCE = vec3(0.0006, 0.0060, 0.0340);
const vec3 FLOOR_DEPTH = vec3(3.0, 11.0, 16.0);

// Smith's masking for a Beckmann distribution (Walter et al. 2007's rational fit): a slope variance seen at a cosine
float smithLambda(const float cosTheta, const float sigmaSq) {
  float c = clamp(cosTheta, 1e-4, 0.9999);
  float v = c / sqrt((1.0 - c * c) * 2.0 * sigmaSq);
  return v >= 1.6 ? 0.0 : (1.0 - 1.259 * v + 0.396 * v * v) / (3.535 * v + 2.181 * v * v);
}

// The sun reflected by a sea whose slopes, round the frame (tx, ty, n), have variances sigmaSq: radiance per unit of
// the sun's illuminance (Bruneton et al. 2010, eq. 13: the slope distribution at the half vector, its Fresnel, its
// masking; the cosine to the sun included)
float reflectedSun(const vec3 l, const vec3 v, const vec3 n, const vec3 tx, const vec3 ty, const vec2 sigmaSq) {
  vec3 h = normalize(l + v);
  float zh = dot(h, n), zl = dot(l, n), zv = dot(v, n);
  if (zh <= 0.0 || zl <= 0.0) return 0.0;
  float zetaX = dot(h, tx) / zh, zetaY = dot(h, ty) / zh;
  float p = exp(-0.5 * (zetaX * zetaX / sigmaSq.x + zetaY * zetaY / sigmaSq.y)) / (2.0 * PI * sqrt(sigmaSq.x * sigmaSq.y));
  // the variance in the plane of each direction
  vec2 lt = vec2(dot(l, tx), dot(l, ty)), vt = vec2(dot(v, tx), dot(v, ty));
  float sigmaL = dot(lt * lt, sigmaSq) / max(dot(lt, lt), 1e-8);
  float sigmaV = dot(vt * vt, sigmaSq) / max(dot(vt, vt), 1e-8);
  float c = 1.0 - dot(v, h);
  float fresnel = 0.02 + 0.98 * c * c * c * c * c;
  zv = max(zv, 0.01);
  return fresnel * p / ((1.0 + smithLambda(zl, sigmaL) + smithLambda(zv, sigmaV)) * zv * zh * zh * zh * zh * 4.0);
}

// The Fresnel reflectance averaged over the slopes seen at a cosine, sigma the slopes' deviation in the view's plane
// (Bruneton et al. 2010's fit, over Schlick's 0.02 at normal incidence)
float meanFresnel(const float cosThetaV, const float sigmaV) {
  float c = clamp(cosThetaV, 0.0, 1.0);
  return 0.02 + 0.98 * pow(1.0 - c, 5.0 * exp(-2.69 * sigmaV)) / (1.0 + 22.7 * pow(sigmaV, 1.5));
}

void main() {
  vec2 uv = gl_FragCoord.xy / P_screen.xy;
  vec4 hv = P_inverseProjection * vec4(uv * 2.0 - 1.0, 1.0, 1.0);
  vec3 d = normalize(mat3(P_inverseView) * normalize(hv.xyz / hv.w));

  // the ray meets the ellipsoid: in its unit sphere, |o + t D|^2 = 1 with c = |o|^2 - 1 worked out in doubles (the
  // eye's height in it, which floats lose near the ground); the nearer root in the form that keeps it
  vec3 o = P_oceanEye.xyz, ds = d * P_oceanRadii.xyz;
  float c = P_oceanEye.w, a = dot(ds, ds), b = dot(o, ds);
  float disc = b * b - a * c;
  bool hit = c > 0.0 && b < 0.0 && disc >= 0.0;
  // near the eye, the grid's own point (a wave's crest may stand against the sky over the horizon); farther, the ray's
  bool geometry = length(vSurface) < OCEAN_REACH && vMiss < 1e-4;
  if (!geometry && !hit) discard;
  vec3 p = geometry ? vSurface : (c / (-b + sqrt(disc))) * d; // the undisplaced point, from the eye (m)
  vec3 q = geometry ? vPosition : p;                          // the point seen
  float t = length(q);
  d = q / t;

  // where it is: the GEBCO map (row 0 north, longitude from -180 at its left edge), its slope of uv across the pixel
  // unwrapped at the date line
  vec3 ecef = P_camera.xyz + p;
  float lon = atan(ecef.y, ecef.x);
  float lat = atan(ecef.z, 0.99330562 * length(ecef.xy)); // geodetic (1 - e^2)
  vec2 mapUv = vec2(lon / (2.0 * PI) + 0.5, 0.5 - lat / PI);
  vec2 mapDx = dFdx(mapUv), mapDy = dFdy(mapUv);
  mapDx.x -= round(mapDx.x);
  mapDy.x -= round(mapDy.x);
  vec2 sea = textureGrad(seaMap, mapUv, mapDx, mapDy).rg;
  // (generous: a texel that is partly sea is sea, and the terrain's depth test draws the coast where land rises)
  float water = smoothstep(0.02, 0.3, sea.g);
  if (water <= 0.0) discard;
  float seaDepth = seaDepthAt(sea.r);
  bool shoreKnown;
  float shore = shoreSigned(p, shoreKnown);
  // near the eye the shore map draws the coast: its signed distance is smooth where the terrain's mesh steps by its
  // elevation texels (the bay's flats, cut in jagged polygons from a few kilometres up)
  if (shoreKnown) {
    water *= smoothstep(-3.0, 3.0, shore);
    if (water <= 0.0) discard;
  }
  float depth = min(seaDepth, OCEAN_BEACH_SLOPE * shore); // (the waves' depth: the beach's near the shore, ocean.glsl)

  // depth-tested a few metres below sea level: between the terrain's sea floor (world::setSeaFloorDepth, further down)
  // and its land (above sea level, however low) - the shore is where the terrain rises out of the water. (Tested a
  // few metres above it, the sea flooded every coast lower than that: the Netherlands, the Everglades, the bay's
  // marshes.)
  // the local frame: up the ellipsoid's normal; the waves' plane's axes laid on it
  vec3 up = normalize(ecef * P_oceanRadii.xyz * P_oceanRadii.xyz);
  // (the waves near the eye: their own surface, half a metre down - a vehicle's hull under the water stays under it)
  vec3 tested;
  if (shoreKnown) {
    // (where the shore map draws the coast, tested a metre and a half above the water: only relief hides it - a cliff,
    // a levee - not the terrain's mesh stepping between its land and its sea floor)
    tested = (geometry ? q : p) + up * 1.5;
  } else if (geometry) {
    tested = q - up * mix(0.5, P_oceanRadii.w, smoothstep(0.5 * OCEAN_REACH, OCEAN_REACH, t));
  } else {
    float cr = c + 2.0 * P_oceanRadii.w * P_oceanRadii.x;
    float discr = b * b - a * cr;
    tested = (discr > 0.0 ? cr / (-b + sqrt(discr)) : t) * d;
  }
  vec4 clip = P_projection * (P_view * vec4(tested, 1.0));
  gl_FragDepth = clamp(clip.z / clip.w, 0.0, 1.0);
  vec3 tx = normalize(P_oceanU.xyz - dot(P_oceanU.xyz, up) * up);
  vec3 ty = cross(up, tx);
  if (dot(ty, P_oceanV.xyz) < 0.0) ty = -ty;

  // the waves: each cascade's slopes at the pixel's footprint (LEAN), and the whole tile's variance (the top level),
  // what Cox & Munk measure beyond what the cascades resolve
  vec2 plane = vec2(dot(p, P_oceanU.xyz), dot(p, P_oceanV.xyz));
  vec2 planeDx = dFdx(plane), planeDy = dFdy(plane);
  vec2 meanSlope = vec2(0.0), variance = vec2(0.0), resolved = vec2(0.0);
  float foam = 0.0;
  float divMean = 0.0, divVariance = 0.0, divWhole = 0.0; // the displacement's divergence: here, and over the tiles
  for (int k = 0; k < OCEAN_CASCADES; ++k) {
    float tile = P_cascade[k].x;
    vec3 at = vec3(P_cascadePhase[k].xy + plane / tile, float(k));
    vec2 gx = planeDx / tile, gy = planeDy / tile;
    vec4 s = textureGrad(slopeMap, at, gx, gy);
    float shallow = shallowAttenuation(k, depth);
    s *= vec4(shallow, shallow, shallow * shallow, shallow * shallow);
    meanSlope += s.xy;
    // (the variance only where the pixel holds a texel or more: magnified, the bilinear filter's mean of squares less
    // the square of its mean is a ripple at the texels' own spacing, and the glints near the eye came out as squares)
    float texels = max(length(gx), length(gy)) * float(OCEAN_MAP_SIZE);
    variance += max(s.zw - s.xy * s.xy, vec2(0.0)) * smoothstep(0.5, 1.5, texels);
    resolved += textureLod(slopeMap, vec3(0.5, 0.5, float(k)), 8.0).zw * shallow * shallow;
    vec4 fm = textureGrad(foamMap, at, gx, gy);
    divMean += fm.z * shallow;
    divVariance += max(fm.w - fm.z * fm.z, 0.0) * shallow * shallow;
    divWhole += textureLod(foamMap, vec3(0.5, 0.5, float(k)), 8.0).w * shallow * shallow;
  }
  // the sea is not the same everywhere: patches of rougher and smoother water kilometres across (gusts, slicks) -
  // the waves' slopes scaled by them, so far off the tiles' one pattern does not repeat to the horizon
  vec2 fromAnchor = P_cascadePhase[0].zw + plane;
  float rough = mix(0.55, 1.25, smoothstep(0.2, 0.8, patches(fromAnchor / 9000.0)));
  meanSlope *= rough;
  // (at least a quarter of Cox & Munk's: the capillaries finer than the smallest cascade are always there - with none,
  // the glints near the eye were the smallest cascade's texels, magnified into squares)
  vec2 sigmaSq = (variance + max(OCEAN_COX_MUNK - resolved, 0.25 * OCEAN_COX_MUNK)) * rough * rough + 2e-5;
  // whitecaps (Dupuy & Bruneton 2012): the crests compressed past a threshold break - the share of the pixel where the
  // displacement's divergence is below it, from the pixel's mean and variance of it (Gaussian). The threshold is set
  // so that over the whole sea that share is the whitecap cover observed for the wind (Monahan & O'Muircheartaigh
  // 1980: 3.84e-6 U^3.41 - 0.3 % at 7 m/s, 1 % at 10, 4 % at 15), more where the water is rougher. Far off the
  // pixel holds the whole distribution and is that share white; near the eye the foam is on the crests.
  {
    float cover = clamp(3.84e-6 * pow(OCEAN_WIND_SPEED, 3.41) * rough * rough, 0.0, 0.3);
    float threshold = -sqrt(2.0 * max(divWhole, 1e-8)) * erfinvApprox(1.0 - 2.0 * max(cover, 1e-6));
    // (at least a twentieth of the whole sea's variance: magnified, the texels' bilinear mean has none, and the foam's
    // edge followed the texel grid in squares)
    float local = sqrt(2.0 * (divVariance + 0.05 * divWhole + 1e-8));
    foam = cover > 1e-5 ? 0.5 * erfcApprox((divMean - threshold) / local) : 0.0;
  }
  // the surf: waves break where the water is shallower than about 0.8 of their height (Miche; McCowan's 0.78), in lines
  // along the depth's contours - along the shore - running in towards it, broken up along their length
  {
    float hs = P_oceanGrid.w;
    float zone = shore < 1e8 ? 1.0 - smoothstep(0.4 * hs, 1.3 * hs, depth) : 0.0; // (only where the shore is known)
    if (zone > 0.0 && hs > 0.05) {
      // only on a shore open to the sea, where the swell comes in: 400 m out from the shore (down the distance's
      // gradient) the water still deepens - in a harbour, a lagoon or behind a breakwater it meets the far shore
      // first, and the water there is calm (Pillar Point's harbour was ringed with surf)
      float gx = shoreDistance(p + tx * 15.0) - shore, gy = shoreDistance(p + ty * 15.0) - shore;
      vec3 out_ = normalize(tx * gx + ty * gy + up * 1e-6);
      float ahead = shoreDistance(p + out_ * 400.0);
      zone *= smoothstep(150.0, 350.0, min(ahead, 1e4) - shore);
    }
    if (zone > 0.0 && hs > 0.05) {
      float phase = depth / (0.35 * hs) + OCEAN_TIME * 0.12 + 0.8 * patches(fromAnchor / 90.0);
      // (each breaker's crest a bright line trailing a fading wash shoreward of it; broken along its length; at the
      // very edge the swash, patchy white water)
      float f = fract(phase);
      float line = smoothstep(0.0, 0.08, f) * (1.0 - smoothstep(0.08, 0.6, f));
      float broken = smoothstep(0.35, 0.75, patches(fromAnchor / 30.0 + vec2(0.0, OCEAN_TIME * 0.25)));
      float swash = 0.6 * smoothstep(0.3, 0.8, patches(fromAnchor / 12.0 - vec2(OCEAN_TIME * 0.2, 0.0)));
      float spread = smoothstep(0.0, 0.4 * hs, depth);
      foam = max(foam, zone * mix(swash, 0.9 * line * broken, spread));
    }
  }
  vec3 n = normalize(up - meanSlope.x * tx - meanSlope.y * ty);
  vec3 nx = normalize(tx - dot(tx, n) * n), ny = cross(n, nx);
  if (dot(ny, ty) < 0.0) ny = -ny;

  // the light at the surface (Bruneton's atmosphere: lengths in km from the planet's centre, takram's altitude
  // correction), the sun's dimmed by the clouds' shadow
  vec3 surface = P_camera.xyz + altitudeCorrection + p;
  vec3 skyIrradiance;
  vec3 sunIlluminance = GetSunAndSkyIrradiance(surface * METER_TO_LENGTH_UNIT, up, sunDirection, skyIrradiance);
  vec3 sunUnshadowed = sunIlluminance;
  float shadow = P_ocean.w > 0.5 ? cloudSunTransmittance(surface) : 1.0;
  sunIlluminance *= shadow;
  float cosSun = max(dot(up, sunDirection), 0.0);

  vec3 v = -d;
  // the sky reflected about the mean normal, the Fresnel averaged over the slopes in the view's plane (kept above the
  // horizon: a ray reflected into the sea sees the sky just above it)
  vec2 vt = vec2(dot(v, nx), dot(v, ny));
  float sigmaV = sqrt(dot(vt * vt, sigmaSq) / max(dot(vt, vt), 1e-8));
  float fresnel = meanFresnel(dot(v, n), sigmaV);
  vec3 r = reflect(-v, n);
  r = normalize(r + up * max(0.02 - dot(r, up), 0.0));
  vec3 skyTransmittance;
  vec3 sky = GetSkyRadiance(surface * METER_TO_LENGTH_UNIT, r, 0.0, sunDirection, skyTransmittance);
  // the clouds in it: the frame's own where the reflected ray is on the screen (seen from below or edge-on, where
  // the Fresnel makes the reflection count), an overcast's grey underside for the cover where the eye is where it is
  // not (looking down from high up, the sea reflects the zenith - and only 2 % of it)
  float cover = P_ground.z;
  vec3 overcast = (sunIlluminance * cosSun + skyIrradiance) * (0.35 / PI);
  vec3 skyCovered = mix(sky, overcast, cover * cover);
  if (P_ocean.w > 0.5) {
    float onScreen;
    vec4 clouds = reflectedClouds(r, sigmaV, onScreen);
    sky = mix(skyCovered, sky * (1.0 - clouds.a) + clouds.rgb, onScreen);
  } else {
    sky = skyCovered;
  }
  vec3 reflected = fresnel * sky + reflectedSun(sunDirection, v, n, nx, ny, sigmaSq) * sunIlluminance;

  // light from below: deep water's own, and the floor's (the imagery) as far as it is seen through the depth
  vec3 down = (sunIlluminance * cosSun + skyIrradiance) / PI;
  // (at most 0.7 of it: the imagery's own water - a photograph of it, mosaicked from different days - showed through
  // shallows as rectangles of another colour with no surface over them)
  vec3 floorSeen = (1.0 - fresnel) * 0.7 * exp(-seaDepth / FLOOR_DEPTH);
  vec3 upwelling = (1.0 - fresnel) * DEEP_REFLECTANCE * down * (1.0 - floorSeen);

  // light through the crests (GodotOceanWaves' water shader, after Atlas's (GDC 2019) wave-peak scattering): sunlight
  // entering the back of a wave standing between the eye and the sun leaves its face blue-green, the red absorbed on
  // the way through - most in the highest crests, the face turned from the sun, the eye looking into it. Near the eye,
  // where the waves stand up.
  if (geometry) {
    float crest = saturate(dot(q - p, up) / max(0.5 * P_oceanGrid.w, 0.05));
    float into = pow(max(dot(sunDirection, d), 0.0), 4.0);
    float away = pow(0.5 - 0.5 * dot(sunDirection, n), 3.0);
    const vec3 THROUGH = vec3(0.02, 0.20, 0.17); // (clear water's transmission through a crest's metre, relative)
    upwelling += 1.6 * crest * into * away * THROUGH * sunIlluminance / PI *
                 (1.0 - smoothstep(0.5 * OCEAN_REACH, OCEAN_REACH, t));
  }

  // foam: white (an albedo of 0.6), over all of it
  vec3 foamLight = 0.6 * down;
  vec3 own = mix(reflected + upwelling, foamLight, foam);
  vec3 through = floorSeen * (1.0 - foam);

  // lit as the land beside it is: the sea's light as a share of a white surface's in the same light (its apparent
  // reflectance), times the light world::setDaylight gives the terrain - the glint as bright as a mirror of the sun is
  // against the land, not mapped by a camera of its own (at the clouds' exposure the glint was a white sheet half the
  // view across, and every cloud in front of it looked lit from below). Then hazed as the terrain is
  // (world/Scattering.cpp's aerial perspective: Bruneton's haze, the clouds', veiled the sea far more than the land at
  // its shore). The imagery seen through the shallows is the terrain's, lit and hazed already: the sea's own light
  // gets the haze over the part of the pixel that is its own.
  float seen = dot(vec3(0.2126, 0.7152, 0.0722), through);
  vec3 white = max((sunUnshadowed * cosSun + skyIrradiance) / PI, vec3(1e-6));
  vec3 color = own / white * P_oceanLight.rgb;
  {
    const vec3 betaR = vec3(5.8e-6, 13.5e-6, 33.1e-6);
    const float betaM = 4.0e-6, scaleHeight = 8000.0, g = 0.76, multi = 0.9;
    float cosZenith = max(abs(dot(d, up)), 0.02);
    float path = min(t, scaleHeight / cosZenith);
    vec3 betaT = betaR + vec3(betaM);
    vec3 fex = exp(-betaT * path);
    float cosTheta = dot(d, sunDirection);
    float phaseR = 0.0596831 * (1.0 + cosTheta * cosTheta);
    float denom = max(1.0 + g * g - 2.0 * g * cosTheta, 1e-4);
    float phaseM = 0.1193662 * (1.0 - g * g) / (denom * sqrt(denom));
    vec3 haze = (betaR * (phaseR + multi * (1.0 - cosZenith)) + vec3(betaM * phaseM)) / betaT * P_oceanHaze.rgb * (1.0 - fex);
    color = shoulder(color * fex + haze * (1.0 - seen));
  }

  outColor = vec4(color * water, 1.0 - water * (1.0 - seen));
}
