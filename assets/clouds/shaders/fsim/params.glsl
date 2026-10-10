// fsim: takram's uniforms (packages/clouds/src/shaders/parameters.glsl, clouds.frag, shadow.frag and their
// materials) as one uniform block - Vulkan has no loose uniforms - under their own names. Mirrored by
// world::Clouds's Params (src/world/Clouds.cpp); "world" is ECEF less the eye (metres), as precise as the GPU's floats
// allow near the camera.

layout(set = 0, binding = 0, std140) uniform CloudsParams {
  mat4 P_inverseProjection;     // clip -> view (this frame, unjittered; Vulkan clip, reverse depth)
  mat4 P_projection;            // view -> clip
  mat4 P_inverseView;           // view -> world
  mat4 P_view;                  // world -> view
  mat4 P_reprojection;          // world -> last frame's clip
  mat4 P_viewReprojection;      // view -> last frame's clip
  mat4 P_shadow[3];             // world -> each cascade's clip
  mat4 P_inverseShadow[3];      // each cascade's clip -> world
  mat4 P_shadowReprojection[3]; // world -> each cascade's last clip
  vec4 P_camera;                // the eye (ECEF, m); its height above the ellipsoid
  vec4 P_altitudeCorrection;    // takram's altitude correction (m); bottomRadius (m)
  vec4 P_sun;                   // towards the sun (ECEF); coverage where the eye is
  vec4 P_screen;                // the window (px); the low-res images' part in use (texels)
  vec4 P_alloc;                 // the low-res images' size; the full-size images' size
  vec4 P_frame;                 // the frame's Bayer offset (0..1); frame number; mipLevelScale
  vec4 P_nearFar;               // cameraNear, cameraFar, shadowFar, maxShadowFilterRadius
  vec4 P_shadowIntervals[3];    // xy
  vec4 P_shadowTexel;           // shadowTexelSize; the shadow map's size
  vec4 P_weather;               // localWeatherRepeat; localWeatherOffset
  vec4 P_shapeRepeat;
  vec4 P_shapeOffset;
  vec4 P_shapeDetailRepeat;
  vec4 P_shapeDetailOffset;
  vec4 P_turbulence;            // turbulenceRepeat; turbulenceDisplacement
  vec4 minLayerHeights;
  vec4 maxLayerHeights;
  vec4 P_minIntervalHeights;
  vec4 P_maxIntervalHeights;
  vec4 densityScales;
  vec4 shapeAmounts;
  vec4 shapeDetailAmounts;
  vec4 weatherExponents;
  vec4 shapeAlteringBiases;
  vec4 coverageFilterWidths;
  vec4 P_heights;               // minHeight, maxHeight, shadowTopHeight, shadowBottomHeight
  vec4 shadowLayerMask;
  CloudDensityProfile densityProfile;
  vec4 P_march;                 // minDensity, minExtinction, minTransmittance, maxRayDistance (the clouds' march)
  vec4 P_march2;                // minStepSize, maxStepSize, perspectiveStepScale, maxIterationCount
  vec4 P_secondary;             // maxIterationCountToSun, maxIterationCountToGround, minSecondaryStepSize, secondaryStepScale
  vec4 P_light;                 // skyLightScale, groundBounceScale, powderScale, powderExponent
  vec4 P_medium;                // scatteringCoefficient, absorptionCoefficient; history: clouds', shadow's (1 valid)
  vec4 P_shadowMarch;           // the shadow's march: maxIterationCount, minStepSize, maxStepSize, opticalDepthTailScale
  vec4 P_shadowMarch2;          // the shadow's minDensity, minExtinction, minTransmittance; -
  vec4 P_ground;                // the ground's height (m above the ellipsoid); the sun's share of its light; the world's
                                // cloud cover; the global map's mean cover round the eye
  vec4 P_global;                // the global coverage map: on (1); where it takes over (m along the ground, from and to);
                                // how far it has taken over everywhere (the eye's height)
  vec4 P_night;                 // the viewer's night light (its ambient, as the day fades), linear RGB
  // the sea (ocean/ocean.glsl): its waves' plane is tangent to the ellipsoid at an anchor near the eye, U east, V north
  vec4 P_ocean;                 // the waves' clock (s); the wind's speed (m/s); where it blows to (rad, from U towards V); the
                                // clouds' shadow on it (1) or none
  vec4 P_oceanSpectrum;         // JONSWAP's alpha; its peak (rad/s); the depth the spectrum is for (m); swell
  vec4 P_oceanSpectrum2;        // spread; detail; whitecap; the foam's growth this frame
  vec4 P_oceanFoam;             // the foam's decay this frame; Cox & Munk's slope variance along U and along V; -
  vec4 P_oceanEye;              // the eye in the ellipsoid's unit sphere (ECEF / (a, a, b)); |that|^2 - 1, from doubles
  vec4 P_oceanRadii;            // 1/a, 1/a, 1/b; how far below sea level the sea is depth-tested (m)
  vec4 P_oceanU;                // the plane's axes (ECEF): east
  vec4 P_oceanV;                //   and north
  vec4 P_cascade[3];            // each cascade's tile (m); -; its band of wavenumbers (rad/m), from and to
  vec4 P_cascadePhase[3];       // where the eye is in each cascade's tile (0..1 along U and V); [0].zw: from the
                                // plane's anchor (m along U and V)
  vec4 P_oceanHaze;             // the sun the terrain's haze is lit by (world::setDaylight's, colour x intensity)
  vec4 P_oceanLight;            // the light world::setDaylight gives level ground: what a white surface there shows
  vec4 P_oceanGrid;             // ocean.vert's grid: a cell's width (0..1 of the screen); its margin beyond the screen;
                                // the spectrum's peak wavenumber (rad/m); the sea's significant wave height (m)
  vec4 P_shore0;                // the sea's shore map: the eye from its middle (ECEF, m); half its width (m)
  vec4 P_shoreEast;             //   its axes (ECEF): east; in use (1) or not
  vec4 P_shoreNorth;            //   north; the distance a step of its code stands for (m; 128 the shore)
};

#define bottomRadius P_altitudeCorrection.w
#define altitudeCorrection P_altitudeCorrection.xyz
#define sunDirection P_sun.xyz
#define frame int(P_frame.z)
#define scatteringCoefficient P_medium.x
#define absorptionCoefficient P_medium.y
#define localWeatherRepeat P_weather.xy
#define localWeatherOffset P_weather.zw
#define shapeRepeat P_shapeRepeat.xyz
#define shapeOffset P_shapeOffset.xyz
#define shapeDetailRepeat P_shapeDetailRepeat.xyz
#define shapeDetailOffset P_shapeDetailOffset.xyz
#define turbulenceRepeat P_turbulence.xy
#define turbulenceDisplacement P_turbulence.z
#define minIntervalHeights P_minIntervalHeights.xyz
#define maxIntervalHeights P_maxIntervalHeights.xyz
#define minHeight P_heights.x
#define maxHeight P_heights.y
#define shadowTopHeight P_heights.z
#define shadowBottomHeight P_heights.w
#define LOCAL_WEATHER_CHANNELS rgba

// The coverage takram's sampleWeather reads: the world's where the eye is, the global map's beyond (fsim/coverage.glsl).
float fsimCoverage;
#define coverage fsimCoverage

layout(set = 0, binding = 1) uniform sampler3D stbnTexture;
layout(set = 0, binding = 2) uniform sampler2D localWeatherTexture;
layout(set = 0, binding = 3) uniform sampler3D shapeTexture;
layout(set = 0, binding = 4) uniform sampler3D shapeDetailTexture;
layout(set = 0, binding = 5) uniform sampler2D turbulenceTexture;
layout(set = 0, binding = 11) uniform sampler2D globalCoverageTexture;
