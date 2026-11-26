#version 450

// Fragment shader for incandescent spark streaks, organic FBM dark soot ribbons
// trailing behind burning debris chunks, and turbulent hydrodynamic water
// splash plumes (VIS-M11). Premultiplied alpha output.

layout(std140, set = 0, binding = 1) uniform Frame {
    vec4 sunDirLight; // xyz = sun dir ECEF, w = daylight (0..1)
    vec4 eyeRight;
    vec4 eyeUp;
    vec4 eyeMeta;
} frame;

layout(location = 0) in vec2 v_uv;
layout(location = 1) in vec4 v_colorAlpha;
layout(location = 2) in vec4 v_params; // x = kind, y = ageNorm, z = seed, w = tempK
layout(location = 3) in vec2 v_lighting; // x = sunDot, y = distM

layout(location = 0) out vec4 outColor;

float hash21(vec2 p) {
    p = fract(p * vec2(123.34, 456.21));
    p += dot(p, p + 45.32);
    return fract(p.x * p.y);
}

float valueNoise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    float a = hash21(i);
    float b = hash21(i + vec2(1.0, 0.0));
    float c = hash21(i + vec2(0.0, 1.0));
    float d = hash21(i + vec2(1.0, 1.0));
    return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}

float fbm3(vec2 p) {
    float v = 0.0;
    float amp = 0.55;
    mat2 rot = mat2(0.80, -0.60, 0.60, 0.80);
    for (int i = 0; i < 3; ++i) {
        v += amp * valueNoise(p);
        p = rot * p * 2.08 + vec2(7.13, 13.71);
        amp *= 0.5;
    }
    return v;
}

void main() {
    float kind = v_params.x;
    float ageNorm = clamp(v_params.y, 0.0, 1.0);
    float seed = v_params.z;
    float tempK = v_params.w;
    float daylight = clamp(frame.sunDirLight.w, 0.08, 1.0);
    float r2 = dot(v_uv, v_uv);
    if (r2 >= 1.0) discard;

    if (kind > 0.5 && kind < 1.5) {
        // SparkEmber: razor-thin glowing white-orange spark streak (pure additive).
        float core = exp(-v_uv.y * v_uv.y * 18.0) * exp(-v_uv.x * v_uv.x * 2.4);
        float fade = (1.0 - ageNorm * ageNorm) * v_colorAlpha.a;
        float glow = core * fade;
        if (glow < 1e-3) discard;
        float hot = clamp((tempK - 900.0) / 1400.0, 0.0, 1.0);
        vec3 col = mix(vec3(1.0, 0.34, 0.03), vec3(1.0, 0.92, 0.66), clamp(core * 0.72 + hot * 0.35, 0.0, 1.0));
        outColor = vec4(col * glow * 3.0, 0.0);
        return;
    }

    vec2 seedOff = vec2(fract(seed * 17.31) * 31.0, fract(seed * 43.17) * 31.0);

    if (kind > 2.5) {
        // FlameCore: turbulent fiery envelope clinging closely to a burning debris chunk.
        float n = fbm3(v_uv * 2.8 + seedOff);
        float r = sqrt(r2);
        float edge = clamp(1.0 - r * (0.90 + 0.55 * n), 0.0, 1.0);
        float core = edge * edge * v_colorAlpha.a;
        if (core < 1e-3) discard;
        vec3 col = mix(vec3(0.95, 0.16, 0.01), vec3(1.0, 0.74, 0.24), clamp(edge * 1.15, 0.0, 1.0));
        outColor = vec4(col * core * 1.35, 0.0);
        return;
    }

    // Smoke (kind == 0) or Water SplashPlume (kind == 2): turbulent 3-octave FBM cloud puff.
    float freq = (kind > 1.5) ? 2.8 : 2.5;
    float n = fbm3(v_uv * freq + seedOff);
    float r = sqrt(r2);
    float radial = smoothstep(0.96, 0.05, r);
    radial *= radial; // quadratic edge softening avoids visible circular disc boundaries
    float density = smoothstep(0.08, 0.78, radial * (0.45 + 1.05 * n));
    if (density < 1e-3) discard;

    float fadeOut = 1.0 - smoothstep(0.55, 1.0, ageNorm);
    float fadeIn = smoothstep(0.0, 0.04, ageNorm + 0.02);
    float alpha = clamp(density * v_colorAlpha.a * fadeOut * fadeIn, 0.0, 0.90);
    if (alpha < 1e-3) discard;

    // Self-shadowing light gradient across puff volume (sunlit top-side vs shadowed core).
    float sunShade = mix(0.42, 1.18, clamp(v_lighting.x * 0.75 + n * 0.35, 0.0, 1.0)) * daylight;
    vec3 rgb = v_colorAlpha.rgb * sunShade;

    // Subtle orange internal bounce light only in the immediate flame zone (< 0.25 s) right behind a blazing chunk.
    if (kind < 0.5 && tempK > 1100.0 && ageNorm < 0.035) {
        float fireGlow = (1.0 - ageNorm / 0.035) * radial * clamp((tempK - 1100.0) / 900.0, 0.0, 1.0);
        rgb += vec3(1.0, 0.28, 0.03) * fireGlow * 0.45;
    }

    // Aerial perspective blend at multi-km distance.
    float haze = 1.0 - exp(-v_lighting.y / 22000.0);
    vec3 horizon = mix(vec3(0.10, 0.12, 0.16), vec3(0.52, 0.62, 0.76), daylight);
    rgb = mix(rgb, horizon, haze * 0.55);

    outColor = vec4(rgb * alpha, alpha);
}
