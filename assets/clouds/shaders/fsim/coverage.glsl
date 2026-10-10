// fsim: the coverage takram's sampleWeather is given at a point. takram has one coverage for the whole sky and does
// not draw views from space; Skybolt (Prograda/Skybolt, MPL-2.0: Assets/Core/Shaders/Clouds.h, whose coverage
// modulation takram's sampleWeather follows) reads a global cloud map instead, cover = clamp(map * 1.3 - 0.001, 0, 1),
// and turns it into the coverage its detail map needs with a fit of its own (pow 0.45). Here: the world's weather
// where the eye is, and NASA's Blue Marble cloud map farther off, its cover turned into takram's coverage by the
// curve measured on takram's own weather and shape textures (tools/cloud_map/coverage_curve.py; the same curve
// world::Clouds turns the world's cloud cover with).

const int FSIM_CURVE = 21;
const float fsimCurveCover[FSIM_CURVE] = float[](0.0, 0.006, 0.016, 0.034, 0.059, 0.093, 0.136, 0.187, 0.249, 0.321,
                                                 0.398, 0.481, 0.569, 0.656, 0.739, 0.824, 0.904, 0.960, 0.985, 0.995, 1.0);
const float fsimCurveCoverage[FSIM_CURVE] = float[](0.10, 0.15, 0.175, 0.2, 0.225, 0.25, 0.275, 0.3, 0.325, 0.35,
                                                    0.375, 0.4, 0.425, 0.45, 0.475, 0.5, 0.525, 0.55, 0.575, 0.6, 0.65);

float fsimCoverageForCover(const float cover) {
  if (cover <= 0.0) return 0.0;
  for (int i = 1; i < FSIM_CURVE; ++i) {
    if (cover <= fsimCurveCover[i]) {
      float t = (cover - fsimCurveCover[i - 1]) / (fsimCurveCover[i] - fsimCurveCover[i - 1]);
      return mix(fsimCurveCoverage[i - 1], fsimCurveCoverage[i], t);
    }
  }
  return fsimCurveCoverage[FSIM_CURVE - 1];
}

// The world's cover near the eye, laid out as the map's clouds lie there: the map's cover at the point over its mean
// round the eye (P_ground.w, within 150 km) - real cloud fields' clusters and clear stretches, tens of kilometres
// across - so that the cover is the world's on average (1 - (1 - cover)^f has the world's cover where f = 1, none
// where the map is clear, an overcast where it is cloudy) and no two stretches of the weather texture's 100 km tiles
// look alike. Where the map round the eye is nearly clear it has no pattern worth following: the world's cover as it
// is. Without it, takram's single coverage made an even field of like puffs out to the horizon.
float fsimNearCover(const float map) {
  float cover = P_ground.z, around = P_ground.w;
  float f = clamp(map / max(around, 1e-3), 0.2, 3.0);
  f = mix(1.0, f, smoothstep(0.03, 0.12, around));
  return 1.0 - pow(1.0 - cover, f);
}

// the coverages near the eye and far off from the global map's cover last read on this invocation's ray, and where
// (w -1: none yet)
vec2 fsimMapCoverage = vec2(0.0);
vec4 fsimMapCoverAt = vec4(0.0, 0.0, 0.0, -1.0);

// position: takram's (ECEF plus the altitude correction, m)
float fsimCoverageAt(const vec3 position) {
  float local = P_sun.w;
  if (P_global.x < 0.5) {
    return local;
  }
  // the world's weather is the weather round the eye: within P_global.y along the ground of the point under it,
  // the map's beyond P_global.z, and everywhere once the eye is high (P_global.w, from its height)
  vec3 ecef = position - altitudeCorrection;
  float along = acos(clamp(dot(normalize(ecef), normalize(P_camera.xyz)), -1.0, 1.0)) * length(P_camera.xyz);
  float away = max(smoothstep(P_global.y, P_global.z, along), P_global.w);
  // Skybolt's cloudUvFromPosRelPlanet (the map's north row first); read, and the coverages worked out from it, once a
  // ray and again only 20 km on along it (its texels are 4.9 km; worked out at every step it cost 0.05 ms in a chase
  // at 1600 x 900, read again every 2 km 0.03 ms - the steps towards the sun between the ray's own missed it - now
  // 0.01 ms; 20 km along a ray near the horizon is a few pixels)
  vec3 fromLast = position - fsimMapCoverAt.xyz;
  if (fsimMapCoverAt.w < 0.0 || dot(fromLast, fromLast) > 4.0e8) {
    vec3 n = normalize(ecef);
    vec2 uv = vec2(atan(n.y, n.x) * RECIPROCAL_PI2 + 0.5, 0.5 - asin(n.z) * RECIPROCAL_PI);
    float cover = clamp(textureLod(globalCoverageTexture, uv, fsimGlobalLod).r * 1.3 - 0.001, 0.0, 1.0);
    fsimMapCoverage = vec2(P_ground.z > 0.0 && P_ground.z < 1.0 ? fsimCoverageForCover(fsimNearCover(cover)) : local,
                           fsimCoverageForCover(cover));
    fsimMapCoverAt = vec4(position, 1.0);
  }
  return mix(fsimMapCoverage.x, fsimMapCoverage.y, away);
}

// The levels for a pixel footprint (m): the shape's (128 texels a repeat), the detail's (32), the turbulence's (128 a
// tile), the weather's (512 a repeat, a cube face's uv spanning its 1.0e7 m) and the global map's (4.9 km a texel).
void fsimSetFootprint(const float footprint) {
  fsimShapeLod = max(0.0, log2(footprint * P_shapeRepeat.x * 128.0));
  fsimDetailLod = max(0.0, log2(footprint * P_shapeDetailRepeat.x * 32.0));
  fsimTurbulenceLod = max(0.0, log2(footprint * localWeatherRepeat.x * P_turbulence.x * 128.0 / 1.0e7));
  fsimWeatherLod = max(0.0, log2(footprint * localWeatherRepeat.x * 512.0 / 1.0e7));
  fsimGlobalLod = max(0.0, log2(footprint / 4.9e3));
}
