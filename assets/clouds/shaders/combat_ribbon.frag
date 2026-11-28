#version 450

// VIS-M13 Direct Exact-Pose Combat Ribbon & Core Fragment Shader:
//   Kind 0 = Gun tracer streak (hot white-gold tip -> amber glowing tail)
//   Kind 1 = Gun muzzle flash volumetric propellant cone & star petals
//   Kind 2 = IR flare blazing magnesium core with 6-point optical diffraction spikes
//   Kind 3 = Supersonic detonation Mach shockwave Fresnel shell + Wilson condensation flash

layout(std140, set = 0, binding = 1) uniform RenderFrame {
    vec4 sunDirLight;
    vec4 eyeRight;
    vec4 eyeUp;
    vec4 eyeMeta;
    vec4 originToEye;
    vec4 projDepth;
    vec4 flowLaid;
    vec4 flowMeta;
} frame;

layout(set = 0, binding = 2) uniform sampler2D sceneDepthTex;
layout(set = 0, binding = 3) uniform sampler2D flowFrontTex;

layout(location = 0) in vec2 v_uv;
layout(location = 1) in vec4 v_color;
layout(location = 2) in vec4 v_params;   // x = kind, y = ageNorm, z = seed, w = extra
layout(location = 3) in vec4 v_lighting; // x = distM, y = radiusM, z = softRangeM, w = sunDot

layout(location = 0) out vec4 outColor;

float linearDepth(float z) {
    return frame.projDepth.y / max(z - frame.projDepth.x, 1e-7);
}

void main() {
    float kind = v_params.x;
    float ageNorm = clamp(v_params.y, 0.0, 1.0);
    float seed = v_params.z;
    float extra = v_params.w;
    float daylight = clamp(frame.sunDirLight.w, 0.08, 1.0);

    vec2 screenUv = gl_FragCoord.xy * frame.projDepth.zw;
    float sceneZ = texture(sceneDepthTex, screenUv).r;
    float fragDist = linearDepth(gl_FragCoord.z);
    float sceneDist = (sceneZ > 1e-7) ? linearDepth(sceneZ) : 1.0e8;
    float depthDiff = sceneDist - fragDist;
    if (depthDiff <= 0.0) discard;
    float softFade = smoothstep(0.0, max(v_lighting.z, 0.15), depthDiff);

    // --- Kind 0: Gun tracer streak (additive HDR) ---
    if (kind < 0.5) {
        // u in [-1, 1]: head at +0.75, tapering tail back to -1.0
        float along = smoothstep(-1.0, 0.62, v_uv.x) * smoothstep(1.0, 0.72, v_uv.x);
        float coreAcross = exp(-v_uv.y * v_uv.y * 28.0);
        float haloAcross = exp(-v_uv.y * v_uv.y * 4.5);
        float headHot = smoothstep(0.15, 0.85, v_uv.x) * coreAcross;
        float bodyGlow = along * (coreAcross * 0.85 + haloAcross * 0.35);
        float strength = bodyGlow * v_color.a * softFade;
        if (strength < 1e-3) discard;
        vec3 tailCol = v_color.rgb;
        vec3 headCol = vec3(1.0, 0.98, 0.88);
        vec3 col = mix(tailCol, headCol, clamp(headHot * 1.15, 0.0, 1.0));
        outColor = vec4(col * strength * 4.5, 0.0);
        return;
    }

    // --- Kind 1: Gun muzzle flash cone (additive HDR) ---
    if (kind < 1.5) {
        // u in [-1, 1]: muzzle base at -1, tip at +1
        float t = v_uv.x * 0.5 + 0.5;
        float coneWidth = mix(0.28, 0.88, pow(t, 0.6)) * (1.0 - smoothstep(0.65, 1.0, t));
        float radial = abs(v_uv.y) / max(coneWidth, 0.05);
        float jet = exp(-radial * radial * 3.8) * smoothstep(0.0, 0.08, t) * smoothstep(1.0, 0.22, t);
        // Secondary side-port muzzle brake flare near base
        float sideStar = exp(-((t - 0.12) * (t - 0.12)) * 45.0) * exp(-v_uv.y * v_uv.y * 3.5);
        float flicker = 0.75 + 0.25 * sin(seed * 47.0 + t * 18.0);
        float glow = (jet * 1.1 + sideStar * 0.85) * flicker * v_color.a * (1.0 - ageNorm) * softFade;
        if (glow < 1e-3) discard;
        vec3 col = mix(vec3(1.0, 0.42, 0.06), vec3(1.0, 0.96, 0.82), clamp(jet * 0.8 + sideStar * 0.6, 0.0, 1.0));
        outColor = vec4(col * glow * 4.2, 0.0);
        return;
    }

    // --- Kind 2: Physically-attenuated MTV pyrotechnic IR flare core ---
    if (kind < 2.5) {
        float r = length(v_uv);
        if (r >= 1.0 || v_color.a <= 1e-4) discard;
        float apparent = clamp(v_lighting.w, 0.0, 1.0);
        // Tight white-hot magnesium-Teflon-Viton pellet core + compact warm combustion plasma sheath
        float core = exp(-r * r * 68.0);
        float plasma = exp(-r * r * 14.0) * 0.42;
        // Subtle optical lens diffraction spikes ONLY when close enough to produce real lens glare
        float ang = atan(v_uv.y, v_uv.x) + seed * 3.14159;
        float spikeMask = smoothstep(0.18, 0.72, apparent);
        float spikes = pow(abs(cos(ang * 3.0)), 26.0) * exp(-r * 5.2) * 0.16 * spikeMask;
        // High-frequency combustion flicker + late-burn sputtering near burnout
        float sputter = (ageNorm > 0.72)
            ? (0.68 + 0.32 * sin(frame.eyeMeta.w * 65.0 + seed * 141.0))
            : (0.90 + 0.10 * sin(frame.eyeMeta.w * 42.0 + seed * 89.0));
        float total = (core * 2.1 + plasma + spikes) * sputter * v_color.a * softFade;
        if (total < 1e-3) discard;
        // Blackbody cooling along MTV burn lifecycle: ~2200 K blinding white-gold -> ~1500 K orange -> dying ember
        float coolT = smoothstep(0.58, 0.98, ageNorm);
        vec3 outerCol = mix(vec3(1.0, 0.62, 0.14), vec3(1.0, 0.30, 0.04), coolT);
        vec3 innerCol = mix(vec3(1.0, 0.99, 0.90), vec3(1.0, 0.80, 0.38), coolT * 0.75);
        vec3 col = mix(outerCol, innerCol, clamp(core * 1.45 + spikes * 0.4, 0.0, 1.0));
        outColor = vec4(col * total * 2.8, 0.0);
        return;
    }

    // --- Kind 3: Expanding supersonic detonation Mach shockwave sphere (Fresnel shell) ---
    float r2 = dot(v_uv, v_uv);
    if (r2 >= 1.0) discard;
    float r = sqrt(r2);
    float cosTheta = sqrt(max(1.0 - r2, 0.0));
    float ang = atan(v_uv.y, v_uv.x);
    // Subtle organic turbulence breakup so the condensation shell never reads like a perfect soap bubble
    float breakup = 0.82 + 0.12 * sin(ang * 7.0 + seed * 19.0) + 0.06 * cos(ang * 13.0 - seed * 31.0);
    // Razor-thin translucent Mach compression front + wispy Wilson condensation veil
    float rim = pow(clamp(1.0 - cosTheta, 0.0, 1.0), 4.6) * smoothstep(1.0, 0.94, r);
    float innerMist = pow(clamp(1.0 - cosTheta, 0.0, 1.0), 1.8) * 0.14 * smoothstep(1.0, 0.80, r);
    float ringPulse = exp(-pow((r - 0.955) * 28.0, 2.0)) * 0.72 * breakup;
    float env = smoothstep(0.0, 0.05, ageNorm) * pow(1.0 - ageNorm, 1.65) * v_color.a * softFade;
    float alpha = clamp((rim * 0.36 + innerMist * 0.28 + ringPulse * 0.30) * env, 0.0, 0.30);
    if (alpha < 1e-3) discard;
    float flash = (1.0 - smoothstep(0.0, 0.20, ageNorm)) * clamp(extra, 0.0, 1.0);
    vec3 mistCol = mix(vec3(0.84, 0.90, 0.97) * (0.44 + 0.56 * daylight) * v_lighting.w,
                       vec3(1.0, 0.82, 0.52), flash * 0.48);
    vec3 lipGlow = mix(vec3(0.88, 0.93, 1.0) * (0.35 + 0.65 * daylight), vec3(1.0, 0.86, 0.62), flash);
    outColor = vec4(mistCol * alpha + lipGlow * ringPulse * env * 0.18, alpha);
}
