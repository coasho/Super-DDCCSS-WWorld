// fsim: the fire and smoke NVIDIA Flow solves (world::FlowVolumes): its shared declarations. The solver's smoke and
// temperature as bricks in a 3D texture (a leaf's 8^3 voxels and the next voxel each way after them: the hardware's
// trilinear filter then the field's), found by a hash of the leaf's place; drawn page by page, each page's box by its
// far faces, the eye's ray marched through it.

#define fsimClampMax vec2(1.0)
#include "fsim/prelude.glsl"
#include "fsim/atmosphere.glsl"

layout(push_constant) uniform PushConstants { mat4 projection; mat4 modelView; } pc;
// [0] the solver's frame's origin from the eye (ECEF, m), its voxel (m); [1] - [3] its east, north, up (ECEF);
// [4] the index (0, 0, 0)'s point (the solver's frame, m); [5] the soot's extinction per unit of the solver's smoke
// (1/m), its albedo, the solver's unit of temperature (K) and the air's (K); [6] the flame's flicker (rms of its
// temperature), its mean against the solver's, its eddies' size (m), the time (s). From BLACKBODY_AT the black body's table
// (blackbodyAt); from PAGES_AT, two rows a page: its box's corner, its size (the solver's frame, m).
layout(std430, set = 0, binding = 0) readonly buffer Samples { vec4 samples[]; };
layout(set = 0, binding = 1) uniform Frame {
  vec4 F_eye;        // the eye (ECEF, m); the angle a fragment spans (rad)
  vec4 F_correction; // takram's altitude correction (m); their bottom radius (m)
  vec4 F_sun;        // towards the sun (ECEF); the time (s)
  vec4 F_night;      // the viewer's night light; the exposure
};
layout(set = 0, binding = 2) uniform sampler3D flowAtlas; // smoke, temperature (the solver's units)
layout(set = 0, binding = 3) uniform sampler3D flowNoise; // takram's shape noise (mean 0.80, deviation 0.076): the flicker
// the leaves drawn: (x, y, z, brick + 1) by their place's hash, open addressing (0 in w: none)
layout(std430, set = 0, binding = 5) readonly buffer Leaves { uvec4 leaves[]; };

#define RECIPROCAL_PI4 0.07957747154594767

// A black body's light (world::SunlitBlackbody): its colour (linear sRGB at luminance 1) and its luminance against the
// sun's disk at T kelvin, 500 to 6000 K (as contrail.glsl's)
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

// (world::FlowVolumes's leafHash)
uint flowHash(ivec3 L) { return (uint(L.x) * 73856093u) ^ (uint(L.y) * 19349663u) ^ (uint(L.z) * 83492791u); }

// a leaf's brick, or -1: none drawn
int flowBrick(ivec3 L) {
  uint h = flowHash(L) & uint(FLOW_HASH - 1);
  for (int probe = 0; probe < 32; ++probe) {
    uvec4 e = leaves[h];
    if (e.w == 0u) return -1;
    if (ivec3(e.xyz) == L) return int(e.w) - 1;
    h = (h + 1u) & uint(FLOW_HASH - 1);
  }
  return -1;
}

// The fields at a point given in voxels from the index's zero (the solver's values at whole voxels), and whether its
// leaf is drawn (the ray may step over a leaf that is not)
// (a ray's samples follow one another through a leaf: the leaf found for the last is the next one's, mostly)
ivec3 flowLastLeaf = ivec3(-2147483647);
int flowLastBrick = -1;
vec2 flowAt(vec3 u, out bool drawn) {
  ivec3 L = ivec3(floor(u)) >> 3;
  if (L != flowLastLeaf) {
    flowLastLeaf = L;
    flowLastBrick = flowBrick(L);
  }
  int b = flowLastBrick;
  drawn = b >= 0;
  if (b < 0) return vec2(0.0);
  ivec3 base = ivec3(b % FLOW_BRICKS, (b / FLOW_BRICKS) % FLOW_BRICKS, b / (FLOW_BRICKS * FLOW_BRICKS)) * FLOW_BRICK;
  vec3 texel = vec3(base) + (u - vec3(L * 8)) + 0.5;
  return texture(flowAtlas, texel / float(FLOW_BRICKS * FLOW_BRICK)).rg;
}
// (the light's samples: each a step of its own away, looked up as it is - the ray's own leaf is kept for the ray)
vec2 flowAt(vec3 u) {
  ivec3 L = ivec3(floor(u)) >> 3;
  int b = L == flowLastLeaf ? flowLastBrick : flowBrick(L);
  if (b < 0) return vec2(0.0);
  ivec3 base = ivec3(b % FLOW_BRICKS, (b / FLOW_BRICKS) % FLOW_BRICKS, b / (FLOW_BRICKS * FLOW_BRICKS)) * FLOW_BRICK;
  vec3 texel = vec3(base) + (u - vec3(L * 8)) + 0.5;
  return texture(flowAtlas, texel / float(FLOW_BRICKS * FLOW_BRICK)).rg;
}
