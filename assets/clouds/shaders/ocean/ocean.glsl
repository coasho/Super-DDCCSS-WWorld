// fsim: what the sea's shaders share (world::Clouds; its uniform block, fsim/params.glsl, carries the sea's numbers).

#ifndef PI
#define PI 3.141592653589793
#endif
#define G 9.81

#define OCEAN_CASCADES 3
#define OCEAN_SPECTRA 4  // the FFT's packed spectra per cascade (GodotOceanWaves' NUM_SPECTRA)
#define OCEAN_MAP_SIZE 256
#define OCEAN_PERIOD 1000.0 // s: the waves repeat in time (OCEAN_TIME wraps at it)

#define OCEAN_TIME P_ocean.x          // s, 0..OCEAN_PERIOD
#define OCEAN_WIND_SPEED P_ocean.y
#define OCEAN_WIND_ANGLE P_ocean.z        // rad: where the wind blows to, from east towards north in the waves' plane
#define OCEAN_ALPHA P_oceanSpectrum.x     // JONSWAP's alpha
#define OCEAN_PEAK_FREQUENCY P_oceanSpectrum.y
#define OCEAN_SPECTRUM_DEPTH P_oceanSpectrum.z
#define OCEAN_SWELL P_oceanSpectrum.w
#define OCEAN_SPREAD P_oceanSpectrum2.x
#define OCEAN_DETAIL P_oceanSpectrum2.y
#define OCEAN_WHITECAP P_oceanSpectrum2.z // the Jacobian below which a crest breaks
#define OCEAN_FOAM_GROW P_oceanSpectrum2.w  // this frame's
#define OCEAN_FOAM_DECAY P_oceanFoam.x      // this frame's
// how much of a cascade's waves the sea floor leaves at a depth (m): a wave feels the bottom shallower than half its
// length, its orbits squashed - its amplitude as sqrt(tanh(k d)), k the cascade's typical wavenumber (the spectrum's
// peak in the first, which holds it; the middle of its band in the others)
float shallowAttenuation(const int cascade, const float depth) {
  float k = cascade == 0 ? P_oceanGrid.z : sqrt(max(P_cascade[cascade].z, 1e-4) * min(P_cascade[cascade].w, 12.0));
  return sqrt(tanh(k * max(depth, 0.0)));
}
float seaDepthAt(const float code) { return exp(code * log(11001.0)) - 1.0; }

#ifdef OCEAN_SHORE_MAP
layout(set = 0, binding = 32) uniform sampler2D shoreMap; // world::Clouds's: the distance to the shore round the eye

// The signed distance from a point (from the eye, m) to the shore, from the shore map round the eye: positive over the
// sea, negative on land; 1e9 off the map or far out to sea. `known`: the map covers the point.
float shoreSigned(const vec3 p, out bool known) {
  known = false;
  if (P_shoreEast.w < 0.5) return 1e9;
  vec3 r = P_shore0.xyz + p;
  vec2 uv = vec2(dot(r, P_shoreEast.xyz), dot(r, P_shoreNorth.xyz)) / (2.0 * P_shore0.w) + 0.5;
  vec2 inside = min(uv, 1.0 - uv);
  if (min(inside.x, inside.y) < 0.01) return 1e9;
  known = true;
  float d = (textureLod(shoreMap, uv, 0.0).r * 255.0 - 128.0) * P_shoreNorth.w;
  return d > 125.0 * P_shoreNorth.w ? 1e9 : d;
}
float shoreDistance(const vec3 p) {
  bool known;
  return shoreSigned(p, known);
}

// The water's depth: GEBCO's, and near the shore the beach's - the sea floor rising to it at 1.5 % (sand beaches' surf
// zones, 1 - 3 %; GEBCO's 3.7 km texels set the depth a kilometre from the shore, not at it)
#define OCEAN_BEACH_SLOPE 0.015
float effectiveDepth(const float gebco, const vec3 p) { return min(gebco, OCEAN_BEACH_SLOPE * shoreDistance(p)); }
#endif

#define OCEAN_REACH 8000.0

#define OCEAN_REACH 8000.0                  // m: the waves' geometry this far (ocean.vert), their shading beyond
#define OCEAN_COX_MUNK P_oceanFoam.yz       // the sea's whole slope variance (along the wind, across it): Cox & Munk 1954

// the FFT's buffer: per cascade 2 x OCEAN_SPECTRA maps (in, then out), each OCEAN_MAP_SIZE^2 complex values
#define FFT_INDEX(cascade, half_, layer, x, y) \
  ((((cascade) * 2 + (half_)) * OCEAN_SPECTRA + (layer)) * OCEAN_MAP_SIZE * OCEAN_MAP_SIZE + (y) * OCEAN_MAP_SIZE + (x))
