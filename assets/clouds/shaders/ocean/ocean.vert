// fsim: the sea's geometry - a grid over the screen, a vertex every few pixels, each on the ellipsoid where its ray meets
// it and moved by the waves' displacement there (Bruneton, Neyret & Holzschuch 2010's screen-space grid; their ocean
// demo, BSD-3, (c) 2009 INRIA). The displacement is read at the level the vertices' spacing on the sea resolves: near
// the eye the waves stand up and hide what is behind them; as they get smaller than the grid they flatten, and the
// shading (ocean.frag, from the slopes' statistics) carries them on to the horizon.

#version 450

#include "clouds/types.glsl"
#include "fsim/params.glsl"
#define OCEAN_SHORE_MAP
#include "ocean/ocean.glsl"

layout(location = 0) in vec2 gridUv; // where on the screen (0..1, a margin beyond it each side)
layout(set = 0, binding = 30) uniform sampler2DArray displacementMap;
layout(set = 0, binding = 23) uniform sampler2D seaMap; // GEBCO: the depth (log-coded), the water's share

layout(location = 0) out vec3 vSurface;  // the sea's undisplaced point (from the eye, m): where the waves' maps are read
layout(location = 1) out vec3 vPosition; // the point the waves move it to
layout(location = 2) out float vMiss;    // 1 where the vertex's ray misses the sea (put at the horizon, not moved)
out gl_PerVertex { vec4 gl_Position; };

vec3 rayAt(vec2 uv) {
  vec4 h = P_inverseProjection * vec4(uv * 2.0 - 1.0, 1.0, 1.0);
  return normalize(mat3(P_inverseView) * normalize(h.xyz / h.w));
}

void main() {
  vec3 d = rayAt(gridUv);
  // the ray and the ellipsoid (as ocean.frag meets them); a ray over the horizon goes to the horizon's distance
  vec3 o = P_oceanEye.xyz, ds = d * P_oceanRadii.xyz;
  float c = P_oceanEye.w, a = dot(ds, ds), b = dot(o, ds);
  float disc = b * b - a * c;
  float horizon = sqrt(max(c, 0.0)) / sqrt(a); // (the tangent's length, in the unit sphere's scale)
  bool hit = c > 0.0 && b < 0.0 && disc >= 0.0;
  float t = hit ? c / (-b + sqrt(disc)) : horizon;
  vec3 p = t * d;

  // the vertices' spacing on the sea: the angle to the next vertex, stretched by how obliquely the ray meets it
  vec3 ecef = P_camera.xyz + p;
  vec3 up = normalize(ecef * P_oceanRadii.xyz * P_oceanRadii.xyz);
  float angle = length(rayAt(gridUv + vec2(P_oceanGrid.x, 0.0)) - d);
  float spacing = t * angle / max(abs(dot(d, up)), 0.05);

  vec3 tx = normalize(P_oceanU.xyz - dot(P_oceanU.xyz, up) * up);
  vec3 ty = cross(up, tx);
  if (dot(ty, P_oceanV.xyz) < 0.0) ty = -ty;
  vec2 plane = vec2(dot(p, P_oceanU.xyz), dot(p, P_oceanV.xyz));
  // (the sea floor's depth there: the waves that feel it smaller)
  float lon = atan(ecef.y, ecef.x), lat = atan(ecef.z, 0.99330562 * length(ecef.xy));
  float depth = effectiveDepth(seaDepthAt(textureLod(seaMap, vec2(lon / (2.0 * PI) + 0.5, 0.5 - lat / PI), 0.0).r), p);
  vec3 displaced = vec3(0.0);
  for (int k = 0; k < OCEAN_CASCADES; ++k) {
    float tile = P_cascade[k].x;
    float lod = log2(max(spacing * 2.0 / (tile / float(OCEAN_MAP_SIZE)), 1.0));
    displaced += shallowAttenuation(k, depth) * textureLod(displacementMap, vec3(P_cascadePhase[k].xy + plane / tile, float(k)), lod).xyz;
  }
  displaced *= 1.0 - smoothstep(0.6 * OCEAN_REACH, OCEAN_REACH, t); // (none where the pixels' own rays take over)
  // the grid's margin beyond the screen held where it is (Skybolt's OceanProjected.vert pins it the same way): near the
  // eye the waves moved its edge onto the screen, and the frame showed through behind it
  vec2 outside = max(-gridUv, gridUv - 1.0) / P_oceanGrid.y;
  displaced *= 1.0 - clamp(max(outside.x, outside.y), 0.0, 1.0);
  // (a ray over the horizon is not the sea: low over the water the horizon is a few kilometres off, inside the waves'
  // reach, and its vertices, moved by the waves, drew the sea up into the sky in streaks)
  if (!hit) displaced = vec3(0.0);
  vMiss = hit ? 0.0 : 1.0;
  vSurface = p;
  vPosition = p + tx * displaced.x + up * displaced.y + ty * displaced.z;
  gl_Position = P_projection * (P_view * vec4(vPosition, 1.0));
  // (nearer than the near plane, the sea is still drawn - at it: the viewer moves the plane out as the eye climbs, and
  // low over the water the sea under the eye was cut away)
  gl_Position.z = min(gl_Position.z, gl_Position.w);
}
