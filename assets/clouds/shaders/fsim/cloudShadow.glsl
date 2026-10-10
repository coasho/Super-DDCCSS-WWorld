// fsim: the clouds' shadow at a point on the ground, from their shadow maps (composite.frag's shade on the ground, the
// sea's sunlight). Needs shadowBuffer (the shadow map, this frame's), fsim/params.glsl, core/math.glsl and
// core/raySphereIntersection.glsl, and SHADOW_CASCADE_COUNT.

// the cloud's optical depth over a point on the ground in one cascade of the shadow map (aerialPerspectiveEffect.frag's
// readShadowOpticalDepth without the tail); -1 off the cascade
float shadowOpticalDepth(const vec3 worldPosition, const float distanceToTop, const int cascade) {
  vec4 clip = P_shadow[cascade] * vec4(worldPosition, 1.0);
  vec2 uv = clip.xy / clip.w * 0.5 + 0.5;
  if (any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0)))) return -1.0;
  vec4 shadow = textureLod(shadowBuffer, vec3(uv, float(cascade)), 0.0);
  return min(shadow.b, shadow.g * max(0.0, distanceToTop - shadow.r));
}

// The share of the sun's light that comes through the clouds to a point on the ground (takram's altitude-corrected
// ECEF, m); 1 off the shadow map. The cascades are taken as getFadedCascadeIndex picks them, the two overlapping ones
// blended across their margin where it dithers between them.
float cloudSunTransmittance(const vec3 ground) {
  float distanceToTop = raySphereSecondIntersection(ground, sunDirection, vec3(0.0), bottomRadius + shadowTopHeight);
  if (distanceToTop <= 0.0) return 1.0;
  vec3 worldPosition = ground - altitudeCorrection - P_camera.xyz;
  float depth = viewZToOrthographicDepth((P_view * vec4(worldPosition, 1.0)).z, P_nearFar.x, P_nearFar.z);
  int nextIndex = -1, prevIndex = -1;
  float alpha = 1.0;
  for (int i = 0; i < SHADOW_CASCADE_COUNT; ++i) {
    vec2 interval = P_shadowIntervals[i].xy;
    float closestEdge = depth < (interval.x + interval.y) * 0.5 ? interval.x : interval.y;
    float margin = closestEdge * closestEdge * 0.5;
    interval += margin * vec2(-0.5, 0.5);
    if (i < SHADOW_CASCADE_COUNT - 1 ? (depth >= interval.x && depth < interval.y) : depth >= interval.x) {
      prevIndex = nextIndex;
      nextIndex = i;
      alpha = i < SHADOW_CASCADE_COUNT - 1 ? saturate(min(depth - interval.x, interval.y - depth) / margin)
                                           : saturate((depth - interval.x) / margin);
    }
  }
  if (nextIndex < 0) return 1.0;
  float opticalDepth = shadowOpticalDepth(worldPosition, distanceToTop, nextIndex);
  if (prevIndex >= 0 && alpha < 1.0) {
    float other = shadowOpticalDepth(worldPosition, distanceToTop, prevIndex);
    opticalDepth = opticalDepth < 0.0 ? other : (other < 0.0 ? opticalDepth : mix(other, opticalDepth, alpha));
  }
  if (opticalDepth < 0.0) return 1.0;
  return exp(-opticalDepth);
}
