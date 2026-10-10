// fsim: what three.js's <common> and <packing> shader chunks give takram's shaders, and the glue a Vulkan compute
// shader needs in place of a WebGL fragment shader's built-ins.

#define PI 3.141592653589793
#define PI2 6.283185307179586
#define RECIPROCAL_PI 0.3183098861837907
#define RECIPROCAL_PI2 0.15915494309189535

// three.js <packing>
float viewZToOrthographicDepth(const float viewZ, const float near, const float far) {
  return (viewZ + near) / (near - far);
}

// The texel the invocation stands for (gl_FragCoord.xy of the fragment shader it replaces).
vec2 fsimFragCoord;
// The levels the textures are read at, from the footprint of a pixel where the sample is (m): what a fragment
// shader's implicit derivatives would pick, on textures with mip levels (fsimSetFootprint, fsim/coverage.glsl).
float fsimTurbulenceLod = 0.0;
float fsimShapeLod = 0.0;
float fsimDetailLod = 0.0;
float fsimWeatherLod = 0.0;
float fsimGlobalLod = 0.0;

// A texel and its neighbour, kept inside the image (texelFetchOffset's out-of-range reads are undefined in Vulkan).
ivec2 fsimClampTexel(const ivec2 coord, const ivec2 offset) {
  return clamp(coord + offset, ivec2(0), ivec2(fsimClampMax) - 1);
}
ivec3 fsimClampTexel(const ivec3 coord, const ivec2 offset) {
  return ivec3(clamp(coord.xy + offset, ivec2(0), ivec2(fsimClampMax) - 1), coord.z);
}
