// fsim: the clouds laid on the frame. takram composites their clouds over a scene lit and hazed by their own
// atmosphere (packages/atmosphere aerialPerspectiveEffect.frag: color = scene * (1 - a) + clouds) and maps the whole
// frame with three.js's AgX at the exposure their examples use (10). The viewer's frame is already its own colours, so
// the clouds are mapped on their own - AgX at exposure 10 - and laid over it; where they start is written as the
// fragment's depth, so the terrain and the vehicles in front of them hide them. MODE 0: the clouds' shadow on the
// ground, multiplied into the frame before the vehicles are drawn (worked out here for each pixel, from the shadow
// map: marched with the clouds, a ray a 4 x 4 block, it was taken at a pixel of the block the frame's Bayer offset
// moved and read back as if it stood still - every shadow's edge on the ground shifted up to 2 px a frame on a
// sixteen-frame cycle, and flickered); 1: the clouds; 2: their depth where they are all
// but opaque, for what is drawn after them (trails, contrails, vapour). By night the viewer's own night light is added
// (an ambient the ground is lit by too): their atmosphere has no moon or night sky, and the clouds were black.

#version 450

#define fsimClampMax P_screen.zw
#define SHADOW_CASCADE_COUNT 3

#include "clouds/types.glsl"
#include "fsim/params.glsl"
#include "fsim/prelude.glsl"
#include "core/math.glsl"
#include "core/raySphereIntersection.glsl"

layout(set = 0, binding = 9) uniform sampler2DArray shadowBuffer;    // the clouds' shadow map, this frame's
layout(set = 0, binding = 15) uniform sampler2D colorBuffer;        // full size: the resolve's
layout(set = 0, binding = 16) uniform sampler2D depthVelocityBuffer; // low-res: w where the cloud starts
layout(location = 0) out vec4 outColor;

const float exposure = 10.0;

// three.js r180 tonemapping_pars_fragment: AgX (Filament's, after Blender's), linear sRGB in and out, exposure 1
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

vec2 screenUv() { return gl_FragCoord.xy / P_screen.xy; }

// the depth of the point `distance` along the ray through this fragment
float depthAt(const float distance) {
  vec4 h = P_inverseProjection * vec4(screenUv() * 2.0 - 1.0, 1.0, 1.0);
  vec3 ray = normalize(h.xyz / h.w);
  vec4 clip = P_projection * vec4(ray * distance, 1.0);
  return clamp(clip.z / clip.w, 1e-12, 1.0);
}

// where the cloud starts: the nearest of the four low-res texels round the fragment (a blend with a texel that has
// no cloud would put the edge behind the terrain)
// Where the pixel's ray enters the cloud layers from outside them (m; 1e30 if it misses them, or the eye is inside them:
// then it would be 0, and cloud drawn in front of everything - an aircraft flown among the layers, at 0.65 - 8 km, was
// stippled with the cloud behind it wherever its pixel's rays missed): no nearer than that is cloud.
float layersEntry() {
  vec4 h = P_inverseProjection * vec4(screenUv() * 2.0 - 1.0, 1.0, 1.0);
  vec3 direction = normalize(mat3(P_inverseView) * normalize(h.xyz / h.w));
  vec3 origin = P_camera.xyz + altitudeCorrection;
  float r = length(origin), b = dot(origin, direction);
  float bottom = bottomRadius + minHeight, top = bottomRadius + maxHeight;
  if (r >= bottom && r <= top) return 1e30;
  if (r < bottom) { // below: up through the lower sphere
    return -b + sqrt(max(b * b - (r * r - bottom * bottom), 0.0));
  }
  float disc = b * b - (r * r - top * top); // above: down onto the upper one
  if (disc < 0.0 || b > 0.0) return 1e30;
  return -b - sqrt(disc);
}

float cloudStart() {
  vec2 at = screenUv() * P_screen.xy / 4.0 - 0.5;
  ivec2 base = ivec2(floor(at));
  float front = 1e30;
  for (int k = 0; k < 4; ++k) {
    ivec2 texel = clamp(base + ivec2(k & 1, k >> 1), ivec2(0), ivec2(P_screen.zw) - 1);
    front = min(front, texelFetch(depthVelocityBuffer, texel, 0).w);
  }
  // fsim: none of the four rays round it found cloud this frame (one ray a texel, jittered: in thin cloud all four
  // can miss) while the resolve, drawing on the frames before, has cloud here: the rays round those, then where the
  // ray enters the layers. Dropping the pixel cut 4 x 4 holes in thin cloud and stepped its edges on the low-res grid,
  // differently every frame - the edges crawled.
  if (front > 1e8) {
    for (int k = 0; k < 16; ++k) {
      ivec2 texel = clamp(base + ivec2(k & 3, k >> 2) - 1, ivec2(0), ivec2(P_screen.zw) - 1);
      front = min(front, texelFetch(depthVelocityBuffer, texel, 0).w);
    }
  }
  if (front > 1e8) front = layersEntry(); // (and with none, the pixel is dropped as before)
  // fsim: and, from far off, a step nearer: there everything lies near the far plane, where the depth (reversed) is
  // the difference of two nearly equal small numbers, the terrain's as well as the cloud's - at 13,000 km the test
  // fell either way from frame to frame, and the clouds seen from far off sparkled with holes onto the ground. 1 %
  // from 5,000 km (none within 1,000 km: in any view from in the air, an aircraft in front of a cloud, a cloud behind a
  // ridge, still sorted as they lie): still, flicker 0.77 -> 0.07
  // grey levels, the pixels flickering by 3 levels or more 5 % -> none
  front *= 1.0 - 0.01 * smoothstep(1.0e6, 5.0e6, front);
  return front;
}

#include "fsim/cloudShadow.glsl"

// the share of the light the ground under the pixel keeps in the clouds' shadow
float groundShade() {
  vec4 h = P_inverseProjection * vec4(screenUv() * 2.0 - 1.0, 1.0, 1.0);
  vec3 direction = normalize(mat3(P_inverseView) * normalize(h.xyz / h.w));
  vec3 origin = P_camera.xyz + altitudeCorrection;
  float t = raySphereFirstIntersection(origin, direction, vec3(0.0), bottomRadius + P_ground.x);
  if (t <= 0.0 || t >= P_nearFar.z * 2.0) return 1.0;
  return 1.0 - P_ground.y * (1.0 - cloudSunTransmittance(origin + t * direction));
}

void main() {
#if MODE == 0
  float s = groundShade();
  outColor = vec4(s, s, s, 1.0);
#else
  vec4 c = texelFetch(colorBuffer, ivec2(gl_FragCoord.xy), 0);
#if MODE == 1
  if (c.a < 0.002) discard;
  float front = cloudStart();
  if (front > 1e8) discard;
  gl_FragDepth = depthAt(front);
  outColor = vec4((agx(c.rgb / c.a * exposure) + P_night.rgb) * c.a, c.a);
#else
  if (c.a < 0.98) discard;
  float front = cloudStart();
  if (front > 1e8) discard;
  gl_FragDepth = depthAt(front);
  outColor = vec4(0.0);
#endif
#endif
}
