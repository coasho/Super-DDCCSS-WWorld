#version 450

// Scene-depth soft-blended fragment shader for continuous pitch-black volumetric smoke,
// 6-way sunlit hydrodynamic ocean water geysers, ocean foam wakes, terrain dust plumes,
// charred skid furrows, flame cores, and incandescent sparks (VIS-M11 v2).
// Runs in Viewer's overScene pass with full-res reverse-Z sceneDepth bound at binding 2,
// completely eliminating hard geometric intersections against ocean, terrain, and airframes.

layout(std140, set = 0, binding = 1) uniform Frame {
    vec4 sunDirLight; // xyz = sun dir ECEF, w = daylight (0..1)
    vec4 eyeRight;
    vec4 eyeUp;
    vec4 eyeMeta;     // xyz = local zenith ECEF, w = simTime
    vec4 projDepth;   // x = P[2][2], y = P[3][2], z = 1/width, w = 1/height
    vec4 flowLaid;    // x0, y0, x1, y1 full-res Flow scissor rect
    vec4 flowMeta;    // x = half-res scale
} frame;

layout(set = 0, binding = 2) uniform sampler2D sceneDepthTex;
layout(set = 0, binding = 3) uniform sampler2D flowFrontTex;

layout(location = 0) in vec2 v_uv;
layout(location = 1) in vec4 v_colorAlpha;
layout(location = 2) in vec4 v_params;   // x = kind, y = ageNorm, z = seed/arc, w = tempK
layout(location = 3) in vec4 v_lighting; // x = sunDot, y = upDot, z = distM, w = softRangeM

layout(location = 0) out vec4 outColor;

// 2D Simplex gradient noise (isotropic triangular lattice -> ZERO square/diamond grid artifacts on seafoam/spray!).
vec3 mod289v3(vec3 x) { return x - floor(x * (1.0 / 289.0)) * 289.0; }
vec2 mod289v2(vec2 x) { return x - floor(x * (1.0 / 289.0)) * 289.0; }
vec3 permutev3(vec3 x) { return mod289v3(((x * 34.0) + 1.0) * x); }

float valueNoise(vec2 v) {
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
    float amp = 0.55;
    mat2 rot = mat2(0.80, -0.60, 0.60, 0.80);
    for (int i = 0; i < 3; ++i) {
        v += amp * valueNoise(p);
        p = rot * p * 2.09 + vec2(7.13, 13.71);
        amp *= 0.48;
    }
    return v;
}

float fbm2(vec2 p) {
    float v = 0.62 * valueNoise(p);
    p = mat2(0.80, -0.60, 0.60, 0.80) * p * 2.11 + vec2(7.13, 13.71);
    return v + 0.38 * valueNoise(p);
}

float linearDepth(float z) {
    // Handles both finite and infinite reverse-Z perspective projections:
    // z_ndc = P22 + P32 / dist => dist = P32 / max(z_ndc - P22, 1e-7)
    float denom = max(z - frame.projDepth.x, 1e-7);
    return frame.projDepth.y / denom;
}

void main() {
    float kind = v_params.x;
    float ageNorm = clamp(v_params.y, 0.0, 1.0);
    float seed = v_params.z;
    float tempK = v_params.w;
    float daylight = clamp(frame.sunDirLight.w, 0.08, 1.0);

    float r2 = dot(v_uv, v_uv);
    // Grounded vertical water curtains (kind 2) use a tapered full-quad vertical silhouette rather than a unit disc.
    if (!(kind > 1.5 && kind < 2.5) && r2 >= 1.0) discard;

    // Exact scene-depth occlusion & soft-particle intersection against terrain, ocean & aircraft!
    vec2 screenUv = gl_FragCoord.xy * frame.projDepth.zw;
    float sceneZ = texture(sceneDepthTex, screenUv).r;
    float fragDist = linearDepth(gl_FragCoord.z);
    float sceneDist = (sceneZ > 1e-7) ? linearDepth(sceneZ) : 1.0e8;
    float depthDiff = sceneDist - fragDist;

    bool isFoamWake = (kind > 3.5 && kind < 4.5);
    bool isScorch = (kind > 5.5);
    bool isFrontPass = (v_lighting.w < 0.0);
    float softRangeM = abs(v_lighting.w);
    float softFade = 1.0;
    if (isFoamWake) {
        // Keep softFade == 1.0 across gentle +/- 1.5 m ocean wave triangles (eliminates ocean-mesh faceting!),
        // while cleanly occluding foam behind solid aircraft wreckage/terrain in front of it.
        if (depthDiff < -3.0) discard;
        softFade = smoothstep(-3.0, -1.6, depthDiff);
    } else if (isScorch) {
        if (depthDiff < -2.2) discard;
        softFade = smoothstep(-2.2, 0.5, depthDiff);
    } else {
        if (depthDiff <= 0.0) discard;
        softFade = smoothstep(0.0, max(softRangeM, 0.15), depthDiff);
        // True 3-D volumetric interleaving against NVIDIA Flow's per-pixel front-surface depth!
        // Particles strictly in front of the Flow cloud wrap cleanly over it from every 360-deg angle,
        // while particles intersecting protruding Flow lobes blend seamlessly inside the smoke/fire volume.
        if (isFrontPass && gl_FragCoord.x >= frame.flowLaid.x && gl_FragCoord.x <= frame.flowLaid.z &&
            gl_FragCoord.y >= frame.flowLaid.y && gl_FragCoord.y <= frame.flowLaid.w) {
            ivec2 fSz = max(textureSize(flowFrontTex, 0) - 1, ivec2(0));
            vec2 fCoord = gl_FragCoord.xy / max(frame.flowMeta.x, 1.0) - 0.5;
            ivec2 f0 = ivec2(floor(fCoord));
            vec2 ff = clamp(fCoord - vec2(f0), vec2(0.0), vec2(1.0));
            float wSum = 0.0;
            float fDistAcc = 0.0;
            float fAlphaAcc = 0.0;
            for (int dy = 0; dy <= 1; ++dy) {
                float wy = (dy == 0) ? (1.0 - ff.y) : ff.y;
                for (int dx = 0; dx <= 1; ++dx) {
                    float wx = (dx == 0) ? (1.0 - ff.x) : ff.x;
                    float w = wx * wy;
                    vec2 da = texelFetch(flowFrontTex, clamp(f0 + ivec2(dx, dy), ivec2(0), fSz), 0).rg;
                    fAlphaAcc += da.g * w;
                    if (da.r > 0.0) {
                        fDistAcc += da.r * w;
                        wSum += w;
                    }
                }
            }
            if (wSum > 1e-4 && fAlphaAcc > 1e-3) {
                float fDist = fDistAcc / wSum;
                float flowInterleave = smoothstep(fDist + 4.5, fDist - 1.8, fragDist);
                softFade *= mix(1.0, mix(0.18, 1.0, flowInterleave), clamp(fAlphaAcc, 0.0, 1.0));
            }
        }
    }
    if (softFade < 1e-3) discard;

    // --- Kind 1: Incandescent white-hot metallic spark streaks (additive HDR) ---
    if (kind > 0.5 && kind < 1.5) {
        float core = exp(-v_uv.y * v_uv.y * 22.0) * exp(-v_uv.x * v_uv.x * 2.2);
        float fade = (1.0 - ageNorm * ageNorm) * v_colorAlpha.a * softFade;
        float glow = core * fade;
        if (glow < 1e-3) discard;
        float hot = clamp((tempK - 1000.0) / 1400.0, 0.0, 1.0);
        vec3 col = mix(vec3(1.0, 0.32, 0.02), vec3(1.0, 0.94, 0.72), clamp(core * 0.78 + hot * 0.35, 0.0, 1.0));
        outColor = vec4(col * glow * 3.8, 0.0);
        return;
    }

    vec2 seedOff = vec2(fract(seed * 17.31) * 37.0, fract(seed * 43.17) * 37.0);

    // --- Kind 3: Tight blazing flame core right on burning chunks (additive HDR) ---
    if (kind > 2.5 && kind < 3.5) {
        float n = fbm3(v_uv * 3.0 + seedOff);
        float r = sqrt(r2);
        float edge = clamp(1.0 - r * (0.90 + 0.58 * n), 0.0, 1.0) * smoothstep(0.95, 0.35, r);
        float core = edge * edge * v_colorAlpha.a * softFade;
        if (core < 1e-3) discard;
        vec3 col = mix(vec3(0.95, 0.18, 0.01), vec3(1.0, 0.82, 0.32), clamp(edge * 1.25, 0.0, 1.0));
        outColor = vec4(col * core * 1.85, 0.0);
        return;
    }

    // --- Kind 6: Horizontal charred ground skid furrow / blast crater scorch ---
    if (kind > 5.5) {
        float n = fbm2(vec2(v_uv.x * 3.0, v_uv.y * 5.0) + seedOff);
        float crossFade = smoothstep(1.0, 0.15, abs(v_uv.y) + (n - 0.5) * 0.45);
        float endFade = smoothstep(1.0, 0.20, abs(v_uv.x));
        float fadeIn = smoothstep(0.0, 0.025, ageNorm);
        float alpha = clamp(crossFade * endFade * (0.55 + 0.50 * n) * v_colorAlpha.a * fadeIn * softFade, 0.0, 0.85);
        if (alpha < 1e-3) discard;
        vec3 rgb = v_colorAlpha.rgb * (0.65 + 0.35 * n) * daylight;
        outColor = vec4(rgb * alpha, alpha);
        return;
    }

    // --- Kind 4: Subtle Translucent Ocean Impact Base-Surge & Lacy Seafoam Rings (dark ocean clearly visible through!) ---
    if (kind > 3.5 && kind < 4.5) {
        float r = sqrt(r2);
        if (r > 0.94) discard;
        vec2 w = vec2(fbm2(v_uv * 2.2 + seedOff), fbm2(v_uv * 2.2 + seedOff + vec2(9.2, 5.7))) - 0.5;
        vec2 p = v_uv + w * 0.42;
        float macroCell = fbm3(p * 3.0 + seedOff);
        float f1 = fbm2(p * 5.8 + seedOff * 1.4);
        float ridgeWeb = pow(clamp(1.0 - abs(f1 * 2.0 - 1.0), 0.0, 1.0), 1.55);
        float microBubbles = fbm2(p * 12.0 - w * 0.55 + seedOff * 2.3);

        // Soft broken ring/patch envelope (slightly thicker outer splash ring as base-surge expands, hollowing center!)
        float radialRing = smoothstep(0.94, 0.22, r) * (0.55 + 0.45 * smoothstep(0.05, 0.55, r));
        float strength = clamp(v_colorAlpha.a, 0.15, 0.65);
        float ageThin = mix(1.0, 0.35, smoothstep(0.08, 0.70, ageNorm));
        float aeration = radialRing * strength * ageThin * smoothstep(0.20, 0.85, macroCell);

        // Soft choppy seafoam patches with natural underwater turquoise-bubble wash and LARGE transparent dark-ocean gaps!
        float subWash = smoothstep(0.05, 0.44, aeration) * radialRing;
        float laceFilaments = smoothstep(0.38, 0.82, ridgeWeb * 0.52 + microBubbles * 0.54) * smoothstep(0.10, 0.45, aeration);
        float frothCrests = smoothstep(0.58, 0.90, aeration * 0.72 + microBubbles * 0.40 + ridgeWeb * 0.18);

        float fadeIn = smoothstep(0.0, 0.05, ageNorm);
        float fadeOut = 1.0 - smoothstep(0.35, 1.0, ageNorm);
        float alpha = clamp((subWash * 0.14 + laceFilaments * 0.28 + frothCrests * 0.38) * fadeIn * fadeOut * softFade, 0.0, 0.40);
        if (alpha < 1e-3) discard;

        float sunLift = clamp(0.76 + 0.24 * v_lighting.x + 0.12 * microBubbles, 0.0, 1.05);
        vec3 deepBubbles = vec3(0.20, 0.35, 0.43) * daylight;
        vec3 laceCol = vec3(0.74, 0.83, 0.89) * daylight;
        vec3 frothCol = vec3(0.94, 0.97, 0.99) * daylight * sunLift;
        vec3 foamLit = mix(deepBubbles, laceCol, clamp(laceFilaments * 1.25, 0.0, 1.0));
        foamLit = mix(foamLit, frothCol, clamp(frothCrests * 1.35, 0.0, 1.0));

        float haze = 1.0 - exp(-v_lighting.z / 24000.0);
        vec3 horizon = mix(vec3(0.10, 0.12, 0.16), vec3(0.52, 0.62, 0.76), daylight);
        foamLit = mix(foamLit, horizon, haze * 0.45);
        outColor = vec4(foamLit * alpha, alpha);
        return;
    }

    // --- Kind 2: Grounded Hydrodynamic Ocean Splash Fountain (1:1 world-meter isotropic frothy spray + gentle falling mist, ZERO hair streaks!) ---
    if (kind > 1.5 && kind < 2.5) {
        // v_uv.y = -1 at sea surface base, +1 at fountain crest; v_uv.x = -1..+1 horizontally; v_params.w = aspect (halfH / widthR).
        // World quad width = 2*widthR (v_uv.x spans 2.0), height = 2*aspect*widthR (hNorm spans 1.0 -> multiply by 2.0*aspect for exact 1:1 world meters!).
        float hNorm = v_uv.y * 0.5 + 0.5; // 0 = base, 1 = tip
        float aspect = clamp(v_params.w, 1.0, 4.8);
        float flare = 0.65 + 0.45 * hNorm;
        // 0.76 vertical scale = gentle 1.3x upward spray elongation (true photographic seawater billows & droplets, ZERO 6x vertical hair bristles!)
        vec2 sprayUv = vec2(v_uv.x / flare, hNorm * 2.0 * aspect * 0.76);
        // Upward burst in first 26% of lifetime, then steady gravity fall downward (NEVER flips or rotates!)
        float flowY = (ageNorm < 0.26) ? (-ageNorm * 5.2) : (-1.352 + (ageNorm - 0.26) * 4.4);

        // Multi-scale turbulent white-water spray clouds + delicate falling rain-droplet veil!
        vec2 warp = vec2(fbm2(sprayUv * 1.15 + vec2(0.0, flowY * 0.45) + seedOff),
                         fbm2(sprayUv * 1.15 + vec2(7.3, flowY * 0.45 + 4.1) + seedOff)) - 0.5;
        float mainSpray = fbm3(sprayUv * 1.85 + warp * 0.68 + vec2(seedOff.x, flowY * 0.85 + seedOff.y));
        float fineMist = fbm3(sprayUv * 4.20 + warp * 0.45 + vec2(seedOff.x * 1.6, flowY * 1.15 + seedOff.y));

        // Fuller roaring frothy silhouette: wide aerated base tapering organically into sparkling broken mist crests
        float halfW = mix(0.96, 0.42, pow(hNorm, 0.82)) * (0.72 + 0.48 * mainSpray);
        float horizMask = smoothstep(halfW, halfW * 0.06, abs(v_uv.x));
        if (horizMask < 1e-3) discard;

        float crestCut = 0.70 + 0.30 * mainSpray;
        float topMask = smoothstep(crestCut, crestCut - 0.38, hNorm);
        float baseMask = smoothstep(0.0, 0.10, hNorm);
        // Realistic gravity collapse: column peaks cleanly then thins & rains back down into the swells!
        float collapseThin = 1.0 - smoothstep(0.24, 0.92, ageNorm + hNorm * 0.30);

        float coreDensity = smoothstep(0.22, 0.74, mainSpray * 0.64 + fineMist * 0.42);
        float mistVeil = smoothstep(0.36, 0.84, fineMist) * 0.50;
        float lowerFoamBoost = mix(1.32, 0.82, hNorm);
        float fadeIn = smoothstep(0.0, 0.04, ageNorm);
        float density = horizMask * topMask * baseMask * lowerFoamBoost * (coreDensity * 0.86 + mistVeil * 0.46);
        float alpha = clamp(density * v_colorAlpha.a * 1.28 * fadeIn * collapseThin * softFade, 0.0, 0.76);
        if (alpha < 1e-3) discard;

        // Volumetric directional shading from spray height-field derivatives (sunlit frothy crests vs cool marine-blue folds!)
        float snx = fbm2((sprayUv + vec2(0.14, 0.0)) * 1.85 + vec2(seedOff.x, flowY * 0.85 + seedOff.y)) - mainSpray;
        float sny = fbm2((sprayUv + vec2(0.0, 0.14)) * 1.85 + vec2(seedOff.x, flowY * 0.85 + seedOff.y)) - mainSpray;
        float billowLit = clamp(0.5 + snx * 1.2 + sny * 1.6, 0.0, 1.0);
        float sunTerm = clamp(v_lighting.x * 0.44 + billowLit * 0.36 + hNorm * 0.24 + fineMist * 0.16 - coreDensity * 0.12, 0.0, 1.0);
        vec3 baseShade = vec3(0.44, 0.58, 0.68) * daylight;
        vec3 sprayCrest = vec3(0.97, 0.99, 1.00) * (0.94 + 0.20 * daylight);
        vec3 rgb = mix(baseShade, sprayCrest, sunTerm);
        float haze = 1.0 - exp(-v_lighting.z / 24000.0);
        vec3 horizon = mix(vec3(0.10, 0.12, 0.16), vec3(0.52, 0.62, 0.76), daylight);
        rgb = mix(rgb, horizon, haze * 0.45);
        outColor = vec4(rgb * alpha, alpha);
        return;
    }

    // --- Kind 0 (Soft Volumetric Billowing Black Smoke Puffs) & Kind 5 (Ground Dust Plume) ---
    float r = sqrt(r2);
    if (r > 0.95) discard;
    vec2 wVec = vec2(fbm2(v_uv * 1.8 + seedOff), fbm2(v_uv * 1.8 + seedOff + vec2(7.3, 4.1))) - 0.5;
    vec2 warpedUv = v_uv + wVec * ((kind < 0.5) ? 0.16 : 0.34);
    float n = fbm3(warpedUv * 2.2 + seedOff);
    float fineN = fbm2(warpedUv * 4.8 + seedOff * 1.9);

    float density;
    float strictCircle = smoothstep(0.95, 0.32, r);
    if (kind < 0.5) {
        // Soft billowing volumetric cloud puff: dense dark core, organic feathery edge (ZERO curly-coral lace stamps or pipes!)
        float edgeDist = r + (n - 0.5) * 0.34 + (fineN - 0.5) * 0.12;
        float softBillow = pow(smoothstep(0.94, 0.08, edgeDist), 1.30);
        float billowTex = 0.60 + 0.55 * n + 0.18 * fineN;
        float erodeLo = mix(0.06, 0.28, ageNorm);
        density = smoothstep(erodeLo, erodeLo + 0.56, softBillow * billowTex) * strictCircle;
    } else {
        // Kind 5 (DustPlume): billowing warm dry-earth dust cloud with strict zero-edge mask!
        float radial = smoothstep(0.92, 0.10, r + (n - 0.5) * 0.36);
        density = smoothstep(0.16, 0.80, radial * radial * (0.45 + 1.05 * n)) * strictCircle;
    }
    if (density < 1e-3) discard;

    float fadeOut = 1.0 - smoothstep(0.45, 1.0, ageNorm);
    float fadeIn = smoothstep(0.0, 0.06, ageNorm + 0.015);
    float maxA = (kind < 0.5) ? 0.68 : 0.72;
    float alpha = clamp(density * v_colorAlpha.a * fadeOut * fadeIn * softFade, 0.0, maxA);
    if (alpha < 1e-3) discard;

    // Gentle volumetric directional shading from billow height-field derivatives (deep charcoal shadows, subtle atmospheric rim!)
    float nx = fbm2((warpedUv + vec2(0.09, 0.0)) * 2.2 + seedOff) - n;
    float ny = fbm2((warpedUv + vec2(0.0, 0.09)) * 2.2 + seedOff) - n;
    float ridgeLit = clamp(0.5 + (nx * 0.9 + ny * 1.25), 0.0, 1.0);
    float sunFactor = clamp(v_lighting.x * 0.52 + ridgeLit * 0.48 - density * 0.30, 0.0, 1.0);
    float skyFactor = clamp(v_lighting.y * 0.68 + 0.32 * fineN, 0.0, 1.0);
    vec3 rgb;
    if (kind < 0.5) {
        // Deep hydrocarbon aviation kerosene soot: dark velvet interior, subtle cool-charcoal sunlit crest (zero bright grey lace!)
        vec3 sootCore = v_colorAlpha.rgb * 0.36;
        vec3 sootRim = v_colorAlpha.rgb * 1.45 + vec3(0.022, 0.024, 0.028);
        rgb = mix(sootCore, sootRim, sunFactor * 0.62 + skyFactor * 0.38) * daylight;
    } else {
        // Warm dry-soil ground dust plume (sunlit khaki-tan crest, darker brown-earth shadowed belly).
        vec3 dustShadow = v_colorAlpha.rgb * 0.48 * daylight;
        vec3 dustLit = v_colorAlpha.rgb * 1.28 * daylight;
        rgb = mix(dustShadow, dustLit, clamp(sunFactor * 0.75 + skyFactor * 0.25, 0.0, 1.0));
    }

    // Aerial perspective fade at multi-km range.
    float haze = 1.0 - exp(-v_lighting.z / 24000.0);
    vec3 horizon = mix(vec3(0.10, 0.12, 0.16), vec3(0.52, 0.62, 0.76), daylight);
    rgb = mix(rgb, horizon, haze * 0.50);

    outColor = vec4(rgb * alpha, alpha);
}
