#version 450

// VIS-M13 Unified Indirect GPU Particle Fragment Shader:
// Reverse-Z scene-depth soft-particle blending + 360-deg Flow depth interleaving for:
//   Kind 0: Pyrotechnic IR flare corkscrew white MgO smoke
//   Kind 1: Incandescent HDR ricochet sparks & burning debris embers (additive)
//   Kind 2: Translucent silver-grey chaff radar-dipole dispersal cloud
//   Kind 3: Gun muzzle propellant gas / HEI impact flash-smoke puff
//   Kind 4: Bullet soil/rock/water surface impact ejecta plume
//   Kind 5: Detonation shockwave mist ring puff
//   Kind 6: Twinkling metallic chaff dipole specular glints (additive)

layout(std140, set = 0, binding = 2) uniform RenderFrame {
    vec4 sunDirLight;   // xyz = sun dir ECEF, w = daylight
    vec4 eyeRight;
    vec4 eyeUp;
    vec4 eyeMeta;       // xyz = zenith, w = simTime
    vec4 originToEye;
    vec4 projDepth;     // P22, P32, 1/W, 1/H
    vec4 flowLaid;      // x0, y0, x1, y1
    vec4 flowMeta;      // x = half-res scale
} frame;

layout(set = 0, binding = 3) uniform sampler2D sceneDepthTex;
layout(set = 0, binding = 4) uniform sampler2D flowFrontTex;

layout(location = 0) in vec2 v_uv;
layout(location = 1) in vec4 v_color;
layout(location = 2) in vec4 v_params;   // x = kind, y = ageNorm, z = seed, w = extra
layout(location = 3) in vec4 v_lighting; // x = sunDot, y = upDot, z = distM, w = softRangeM

layout(location = 0) out vec4 outColor;

vec3 mod289v3(vec3 x) { return x - floor(x * (1.0 / 289.0)) * 289.0; }
vec2 mod289v2(vec2 x) { return x - floor(x * (1.0 / 289.0)) * 289.0; }
vec3 permutev3(vec3 x) { return mod289v3(((x * 34.0) + 1.0) * x); }

float snoise(vec2 v) {
    const vec4 C = vec4(0.211324865405187, 0.366025403784439, -0.577350269189626, 0.024390243902439);
    vec2 i = floor(v + dot(v, C.yy));
    vec2 x0 = v - i + dot(i, C.xx);
    vec2 i1 = (x0.x > x0.y) ? vec2(1.0, 0.0) : vec2(0.0, 1.0);
    vec4 x12 = x0.xyxy + C.xxzz;
    x12.xy -= i1;
    i = mod289v2(i);
    vec3 p = permutev3(permutev3(i.y + vec3(0.0, i1.y, 1.0)) + i.x + vec3(0.0, i1.x, 1.0));
    vec3 m = max(0.5 - vec3(dot(x0, x0), dot(x12.xy, x12.xy), dot(x12.zw, x12.zw)), 0.0);
    m = m * m;
    m = m * m;
    vec3 x = 2.0 * fract(p * C.www) - 1.0;
    vec3 h = abs(x) - 0.5;
    vec3 ox = floor(x + 0.5);
    vec3 a0 = x - ox;
    m *= 1.79284291400159 - 0.85373472095314 * (a0 * a0 + h * h);
    vec3 g;
    g.x = a0.x * x0.x + h.x * x0.y;
    g.yz = a0.yz * x12.xz + h.yz * x12.yw;
    return clamp(0.5 + 0.5 * (130.0 * dot(m, g)), 0.0, 1.0);
}

float fbm3(vec2 p) {
    float v = 0.0;
    float amp = 0.56;
    mat2 rot = mat2(0.80, -0.60, 0.60, 0.80);
    for (int i = 0; i < 3; ++i) {
        v += amp * snoise(p);
        p = rot * p * 2.07 + vec2(5.71, 11.37);
        amp *= 0.48;
    }
    return v;
}

float linearDepth(float z) {
    return frame.projDepth.y / max(z - frame.projDepth.x, 1e-7);
}

float flowInterleaveWeight(float fragDist) {
    if (gl_FragCoord.x < frame.flowLaid.x || gl_FragCoord.x > frame.flowLaid.z ||
        gl_FragCoord.y < frame.flowLaid.y || gl_FragCoord.y > frame.flowLaid.w)
        return 1.0;
    ivec2 fSz = max(textureSize(flowFrontTex, 0) - 1, ivec2(0));
    vec2 fCoord = gl_FragCoord.xy / max(frame.flowMeta.x, 1.0) - 0.5;
    ivec2 f0 = clamp(ivec2(floor(fCoord)), ivec2(0), fSz);
    vec2 da = texelFetch(flowFrontTex, f0, 0).rg;
    if (da.r <= 0.0 || da.g <= 1e-3) return 1.0;
    float front = smoothstep(da.r + 3.5, da.r - 1.5, fragDist);
    return mix(1.0, mix(0.15, 1.0, front), clamp(da.g, 0.0, 1.0));
}

void main() {
    float kind = v_params.x;
    float ageNorm = v_params.y;
    float seed = v_params.z;
    float extra = v_params.w;
    float daylight = clamp(frame.sunDirLight.w, 0.08, 1.0);

    float r2 = dot(v_uv, v_uv);
    if (r2 >= 1.0) discard;

    vec2 screenUv = gl_FragCoord.xy * frame.projDepth.zw;
    float sceneZ = texture(sceneDepthTex, screenUv).r;
    float fragDist = linearDepth(gl_FragCoord.z);
    float sceneDist = (sceneZ > 1e-7) ? linearDepth(sceneZ) : 1.0e8;
    float depthDiff = sceneDist - fragDist;
    if (depthDiff <= 0.0) discard;

    float softFade = smoothstep(0.0, max(v_lighting.w, 0.12), depthDiff) * flowInterleaveWeight(fragDist);
    if (softFade < 1e-3) discard;

    // --- Kind 1: Incandescent HDR molten amber-gold sparks & burning embers (additive) ---
    if (kind > 0.5 && kind < 1.5) {
        float core = exp(-v_uv.y * v_uv.y * 28.0) * exp(-v_uv.x * v_uv.x * 3.0);
        float fade = pow(1.0 - ageNorm, 1.45) * softFade;
        float glow = core * fade;
        if (glow < 1e-3) discard;
        // Rich molten orange-gold gradient with pinpoint hot yellow-white center (never washed-out chalk lines!)
        float hot = clamp((1.0 - ageNorm * 0.85) * 0.65 + core * 0.50, 0.0, 1.0);
        vec3 ember = mix(vec3(1.0, 0.20, 0.01), vec3(1.0, 0.72, 0.16), hot);
        ember = mix(ember, vec3(1.0, 0.96, 0.78), smoothstep(0.72, 1.0, core * (1.0 - ageNorm * 0.5)));
        vec3 col = mix(ember, v_color.rgb, 0.25);
        outColor = vec4(col * glow * 2.6, 0.0);
        return;
    }

    // --- Kind 6: Chaff metallic aluminium-foil dipole glint (delicate pinpoint specular sparkle) ---
    if (kind > 5.5) {
        float twinkle = pow(max(0.0, sin(frame.eyeMeta.w * 22.0 + seed * 97.3)), 6.0);
        float shape = exp(-v_uv.y * v_uv.y * 18.0) * exp(-v_uv.x * v_uv.x * 5.0);
        float fade = smoothstep(0.0, 0.08, ageNorm) * (1.0 - smoothstep(0.45, 1.0, ageNorm)) * softFade;
        float g = shape * (0.18 + 0.82 * twinkle) * fade;
        if (g < 1e-3) discard;
        vec3 glintCol = mix(vec3(0.70, 0.78, 0.88), vec3(0.98, 0.99, 1.0), twinkle) * (0.45 + 1.35 * daylight);
        outColor = vec4(glintCol * g * 1.45, 0.0);
        return;
    }

    vec2 seedOff = vec2(fract(seed * 19.73) * 31.0, fract(seed * 41.29) * 31.0);
    float r = sqrt(r2);
    float n = fbm3(v_uv * 2.6 + seedOff);
    float puff = smoothstep(0.96, 0.16, r + (n - 0.5) * 0.42);
    if (puff < 1e-3) discard;

    float fadeIn = smoothstep(0.0, 0.04, ageNorm);
    float fadeOut = 1.0 - smoothstep(0.32, 1.0, ageNorm);
    float sunShade = 0.44 + 0.56 * v_lighting.x + 0.10 * v_lighting.y;

    // --- Kind 0: Pyrotechnic IR flare corkscrew white MgO smoke (dense readable self-shaded plume!) ---
    if (kind < 0.5) {
        float alpha = clamp(puff * 0.82 * fadeIn * fadeOut * softFade, 0.0, 0.86);
        if (alpha < 1e-3) discard;
        // Cool grey-silver shadow side + bright warm sunlit top + brief golden flare bounce near birth
        vec3 shaded = mix(vec3(0.48, 0.51, 0.56), vec3(0.96, 0.95, 0.92), clamp(v_lighting.x * 0.85 + 0.15 * n, 0.0, 1.0)) * daylight;
        float warmBounce = (1.0 - smoothstep(0.0, 0.20, ageNorm)) * 0.42;
        vec3 smokeCol = mix(shaded, vec3(1.0, 0.82, 0.42), warmBounce);
        outColor = vec4(smokeCol * alpha, alpha);
        return;
    }

    // --- Kind 2: Translucent silver-grey chaff radar-dipole dispersal cloud ---
    if (kind < 2.5) {
        float fineGlint = smoothstep(0.68, 0.93, snoise(v_uv * 12.0 + seedOff * 2.0 + vec2(frame.eyeMeta.w * 2.0, 0.0)));
        float alpha = clamp(puff * 0.32 * fadeIn * (1.0 - smoothstep(0.38, 1.0, ageNorm)) * softFade, 0.0, 0.34);
        if (alpha < 1e-3) discard;
        vec3 col = mix(vec3(0.46, 0.51, 0.57), v_color.rgb, v_lighting.x) * daylight + vec3(0.42, 0.48, 0.55) * fineGlint * daylight;
        outColor = vec4(col * alpha, alpha);
        return;
    }

    // --- Kind 3: Muzzle propellant gas / HEI airframe impact puff ---
    if (kind < 3.5) {
        float flash = (1.0 - smoothstep(0.0, 0.24, ageNorm)) * clamp(extra, 0.0, 2.0);
        float alpha = clamp(puff * 0.52 * fadeIn * (1.0 - smoothstep(0.22, 1.0, ageNorm)) * softFade, 0.0, 0.68);
        vec3 gasCol = v_color.rgb * daylight * sunShade;
        vec3 flashCol = vec3(1.0, 0.68, 0.24) * flash * puff * 1.9 * softFade;
        outColor = vec4(gasCol * alpha + flashCol, alpha);
        return;
    }

    // --- Kind 4: Bullet surface impact ejecta (Soil / Rock / Water) ---
    if (kind < 4.5) {
        float alpha = clamp(puff * 0.68 * fadeIn * fadeOut * softFade, 0.0, 0.78);
        if (alpha < 1e-3) discard;
        vec3 col = v_color.rgb * daylight * (0.58 + 0.42 * v_lighting.x + 0.14 * n);
        outColor = vec4(col * alpha, alpha);
        return;
    }

    // --- Kind 5: Detonation condensation shock mist puff ---
    float alpha = clamp(puff * 0.34 * fadeIn * (1.0 - smoothstep(0.18, 1.0, ageNorm)) * softFade, 0.0, 0.42);
    if (alpha < 1e-3) discard;
    vec3 mistCol = v_color.rgb * daylight * sunShade;
    outColor = vec4(mistCol * alpha, alpha);
}
