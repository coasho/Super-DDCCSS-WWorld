// From takram-design-engineering/three-geospatial (MIT, see ../../LICENSE-three-geospatial.txt): packages/core/src/shaders/cascadedShadowMaps.glsl; changes marked fsim.
// Reference: https://github.com/mrdoob/three.js/blob/r171/examples/jsm/csm/CSMShader.js

#ifndef SHADOW_CASCADE_COUNT
#error "SHADOW_CASCADE_COUNT macro must be defined."
#endif // SHADOW_CASCADE_COUNT

int getCascadeIndex(
  const mat4 viewMatrix,
  const vec3 worldPosition,
  const vec2 intervals[SHADOW_CASCADE_COUNT],
  const float near,
  const float far
) {
  vec4 viewPosition = viewMatrix * vec4(worldPosition, 1.0);
  float depth = viewZToOrthographicDepth(viewPosition.z, near, far);
  vec2 interval;
  // fsim: three.js's unrolled loop as a plain loop
  for (int i = 0; i < SHADOW_CASCADE_COUNT; ++i) {
    interval = intervals[i];
    if (depth >= interval.x && depth < interval.y) {
      return i;
    }
  }
  return SHADOW_CASCADE_COUNT - 1;
}

int getFadedCascadeIndex(
  const mat4 viewMatrix,
  const vec3 worldPosition,
  const vec2 intervals[SHADOW_CASCADE_COUNT],
  const float near,
  const float far,
  const float jitter
) {
  vec4 viewPosition = viewMatrix * vec4(worldPosition, 1.0);
  float depth = viewZToOrthographicDepth(viewPosition.z, near, far);

  vec2 interval;
  float intervalCenter;
  float closestEdge;
  float margin;
  int nextIndex = -1;
  int prevIndex = -1;
  float alpha;

  // fsim: three.js's unrolled loop as a plain loop
  for (int i = 0; i < SHADOW_CASCADE_COUNT; ++i) {
    interval = intervals[i];
    intervalCenter = (interval.x + interval.y) * 0.5;
    closestEdge = depth < intervalCenter ? interval.x : interval.y;
    margin = closestEdge * closestEdge * 0.5;
    interval += margin * vec2(-0.5, 0.5);

    if (i < SHADOW_CASCADE_COUNT - 1) {
      if (depth >= interval.x && depth < interval.y) {
        prevIndex = nextIndex;
        nextIndex = i;
        alpha = saturate(min(depth - interval.x, interval.y - depth) / margin);
      }
    } else {
      // Don't fade out the last cascade.
      if (depth >= interval.x) {
        prevIndex = nextIndex;
        nextIndex = i;
        alpha = saturate((depth - interval.x) / margin);
      }
    }
  }

  return jitter <= alpha
    ? nextIndex
    : prevIndex;
}
