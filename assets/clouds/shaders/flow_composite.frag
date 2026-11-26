// fsim: full-resolution depth-aware bilateral composite for half-res NVIDIA Flow volumes (VIS-M12).
// Rejects background fire texels behind foreground aircraft/terrain edges so 4K silhouettes remain razor-sharp.

#version 450

layout(set = 0, binding = 0) uniform CompositeParams {
  mat4 inverseProjection;
  vec4 screen; // fullW, fullH, halfW, halfH
  vec4 region; // rx0, ry0, rx1, ry1 (valid half-res texel range)
  vec4 laid;   // full-res pixel scissor rect x0, y0, x1, y1
  vec4 counts; // pageCount, scale, 0, 0
};
layout(set = 0, binding = 1) uniform sampler2D sceneDepth;
layout(set = 0, binding = 2) uniform sampler2D halfColor;
layout(set = 0, binding = 3) uniform sampler2D halfFront;
layout(location = 0) out vec4 outColor;

float distanceAt(vec2 pixel, float z) {
  vec4 h = inverseProjection * vec4(pixel / screen.xy * 2.0 - 1.0, z, 1.0);
  return length(h.xyz / h.w);
}

void main() {
  vec2 px = gl_FragCoord.xy;
  if (px.x < laid.x || px.y < laid.y || px.x > laid.z || px.y > laid.w) discard;

  float scale = max(counts.y, 1.0);
  ivec2 maxDepthPx = max(min(textureSize(sceneDepth, 0), ivec2(screen.xy)) - 1, ivec2(0));
  ivec2 pInt = clamp(ivec2(px), ivec2(0), maxDepthPx);
  float z = texelFetch(sceneDepth, pInt, 0).r;
  float pixelDist = z > 0.0 ? distanceAt(px, z) : 1e30;

  vec2 hCoord = px / scale - 0.5;
  ivec2 h0 = ivec2(floor(hCoord));
  vec2 f = clamp(hCoord - vec2(h0), vec2(0.0), vec2(1.0));
  ivec2 rMin = ivec2(region.xy);
  ivec2 rMax = max(ivec2(region.zw) - 1, rMin);

  vec4 accum = vec4(0.0);
  float wSum = 0.0;
  for (int dy = 0; dy <= 1; ++dy) {
    float wy = dy == 0 ? (1.0 - f.y) : f.y;
    for (int dx = 0; dx <= 1; ++dx) {
      float wx = dx == 0 ? (1.0 - f.x) : f.x;
      ivec2 tc = clamp(h0 + ivec2(dx, dy), rMin, rMax);
      vec4 col = texelFetch(halfColor, tc, 0);
      float front = texelFetch(halfFront, tc, 0).r;
      float w = wx * wy;
      // Edge rejection: if this full-res pixel is a solid surface strictly in front of the fire, drop background fire.
      if (front > 0.0 && pixelDist < front) {
        float depthWeight = smoothstep(front - 1.5, front, pixelDist);
        w *= depthWeight;
      }
      accum += col * w;
      wSum += w;
    }
  }
  if (wSum <= 1e-5) discard;
  vec4 result = accum / wSum;
  if (result.a < 1e-4) discard;
  outColor = result;
}
