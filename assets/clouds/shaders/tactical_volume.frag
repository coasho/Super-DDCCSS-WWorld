#version 450

// VIS-M14 3-D Depth-Tested Tactical & EMSO Overlay Fragment Shader (v3 - Clean Precision Style).
// Restrained, non-flashy tactical drafting language: dark-haloed razor-sharp borders,
// range arcs, burn-through arcs, and quiet uniform translucent fills that remain effortlessly
// readable against both bright sunlit clouds and dark ocean — with zero flashy strobe waves,
// zero flickering static, and soft reverse-Z terrain occlusion.

layout(std140, set = 0, binding = 1) uniform TacticalFrame {
    vec4 projDepth; // x = p22, y = p32, z = 1/width, w = 1/height
    vec4 timeMeta;  // x = simTime, y = wallTime, z = daylight, w = unused
} frame;

layout(set = 0, binding = 2) uniform sampler2D sceneDepthTex;

layout(location = 0) in vec4 v_posR;
layout(location = 1) in vec4 v_normU;
layout(location = 2) in vec4 v_color;
layout(location = 3) in vec4 v_params;

layout(location = 0) out vec4 outColor;

float linearDepth(float z) {
    return frame.projDepth.y / max(z - frame.projDepth.x, 1e-7);
}

float aaLine(float val, float halfWidthPx) {
    float fw = max(fwidth(val), 1e-5);
    return 1.0 - smoothstep(0.0, fw * halfWidthPx, abs(val));
}

// Dark-haloed precision stroke: subtle charcoal casing around a crisp colored core line
// guarantees instant contrast over both bright clouds and dark sea without neon bloom.
void addFramedStroke(inout vec3 col, inout float alpha, float distField,
                     float corePx, float haloPx, vec3 lineCol, float lineAlpha) {
    float halo = aaLine(distField, haloPx);
    if (halo > 0.005) {
        float casingA = halo * lineAlpha * 0.62;
        col = mix(col, vec3(0.02, 0.04, 0.07), clamp(casingA, 0.0, 0.82));
        alpha = max(alpha, casingA);
    }
    float core = aaLine(distField, corePx);
    if (core > 0.005) {
        col = mix(col, lineCol, clamp(core, 0.0, 1.0));
        alpha = max(alpha, core * lineAlpha);
    }
}

void main() {
    float style = v_params.x;
    float aux1 = v_params.y;
    float aux2 = v_params.z;
    float scaleM = v_params.w;
    float rNorm = clamp(v_posR.w, 0.0, 1.0);
    float uCoord = v_normU.w;

    vec2 screenUv = gl_FragCoord.xy * frame.projDepth.zw;
    float sceneZ = texture(sceneDepthTex, screenUv).r;
    float fragDist = linearDepth(gl_FragCoord.z);
    float sceneDist = (sceneZ > 1e-7) ? linearDepth(sceneZ) : 1.0e8;
    float depthDiff = sceneDist - fragDist;
    if (depthDiff <= 0.0) discard;

    float softClip = smoothstep(0.0, clamp(fragDist * 0.002, 2.0, 18.0), depthDiff);

    vec3 viewDir = normalize(-v_posR.xyz);
    vec3 nrm = normalize(v_normU.xyz + vec3(1e-7));
    float ndv = abs(dot(nrm, viewDir));
    float fresnel = pow(clamp(1.0 - ndv, 0.0, 1.0), 1.8);

    vec3 baseCol = v_color.rgb;
    float baseAlpha = v_color.a;

    // -------------------------------------------------------------------------
    // Style 0: Radar Tactical Sector (Focused Aircraft Only)
    //   Clean flat subtle tint + dark-outlined borders & 10 km range arcs +
    //   crisp live sweep ray + razor-sharp dark-outlined gold burn-through arc.
    //   Suppressed outer zone uses quiet desaturated fill + static dashed border.
    // -------------------------------------------------------------------------
    if (style < 0.5) {
        float burnFrac = clamp(aux1, 0.04, 1.0);
        bool isJammed = (burnFrac < 0.985);
        bool inBlindZone = isJammed && (rNorm > burnFrac);
        float apexFade = smoothstep(0.0, 0.03, rNorm);

        float rangeKm = rNorm * max(scaleM, 1000.0) * 1e-3;
        float ringStep = (scaleM > 28000.0) ? 10.0 : 5.0;
        float ringDist = fract(rangeKm / ringStep + 0.5) - 0.5;
        float centerAzDist = abs(uCoord);

        vec3 col = baseCol;
        float alpha = 0.11 * apexFade;

        if (!inBlindZone) {
            // Subtle 10 km range arcs & center boresight tick
            float ringMask = smoothstep(0.06, 0.11, rNorm);
            addFramedStroke(col, alpha, ringDist, 1.25, 2.5, mix(baseCol, vec3(0.90, 0.97, 1.0), 0.38), 0.54 * ringMask * apexFade);
            addFramedStroke(col, alpha, centerAzDist, 1.0, 2.0, baseCol, 0.28 * apexFade);

            // Crisp dark-outlined left/right sector edges & far edge (when unjammed)
            addFramedStroke(col, alpha, 1.0 - abs(uCoord), 2.0, 4.2, mix(baseCol, vec3(0.94, 0.98, 1.0), 0.30), 0.92 * apexFade);
            if (!isJammed) {
                addFramedStroke(col, alpha, 1.0 - rNorm, 2.1, 4.4, mix(baseCol, vec3(0.94, 0.98, 1.0), 0.34), 0.94 * apexFade);
            }

            // Crisp thin instantaneous radar beam ray
            float beamDist = abs(uCoord - aux2);
            addFramedStroke(col, alpha, beamDist, 1.45, 3.0, mix(baseCol, vec3(0.96, 1.0, 1.0), 0.72), 0.86 * apexFade);
        } else {
            // Quiet suppressed outer zone: whisper-subtle cool slate tint + static dashed outer frame
            float normOut = clamp((rNorm - burnFrac) / max(1.0 - burnFrac, 0.06), 0.0, 1.0);
            float fadeOut = mix(1.0, 0.35, normOut);
            col = vec3(0.46, 0.52, 0.62);
            alpha = 0.032 * fadeOut * apexFade;
            float dashSide = step(0.42, fract(rNorm * 18.0));
            float dashArc = step(0.42, fract((uCoord * 0.5 + 0.5) * 14.0));
            addFramedStroke(col, alpha, 1.0 - abs(uCoord), 1.35, 2.7, vec3(0.68, 0.74, 0.84), 0.56 * dashSide * fadeOut * apexFade);
            addFramedStroke(col, alpha, 1.0 - rNorm, 1.35, 2.7, vec3(0.68, 0.74, 0.84), 0.56 * dashArc * fadeOut * apexFade);
        }

        // Unmistakable dark-outlined warm-gold Burn-Through arc boundary
        if (isJammed) {
            addFramedStroke(col, alpha, rNorm - burnFrac, 2.2, 4.4, vec3(1.00, 0.80, 0.16), 0.96 * apexFade);
        }

        alpha = clamp(alpha * baseAlpha * softClip, 0.0, 0.92);
        if (alpha < 0.004) discard;
        outColor = vec4(col, alpha);
        return;
    }

    // -------------------------------------------------------------------------
    // Style 1: EA Jamming Cone (Clean Geometric Violet / Amber Envelope)
    //   Zero flashy strobe waves: quiet translucent shell + crisp Fresnel rim +
    //   3 clean static dark-outlined range contour rings (33%, 66%, 98%).
    // -------------------------------------------------------------------------
    if (style < 1.5) {
        float faceWeight = gl_FrontFacing ? 1.0 : 0.15;
        float apexFade = smoothstep(0.0, 0.04, rNorm);
        float farFade = mix(1.0, 0.55, pow(rNorm, 1.3));
        float env = apexFade * farFade * softClip * faceWeight;

        vec3 col = baseCol;
        float alpha = (0.052 + fresnel * 0.30) * env;

        // 3 static clean range rings along the cone (at 0.33, 0.66, 0.98)
        float ring33 = abs(rNorm - 0.333);
        float ring66 = abs(rNorm - 0.666);
        float ringEnd = 1.0 - rNorm;
        float nearestRing = min(min(ring33, ring66), ringEnd);
        vec3 ringCol = mix(baseCol, vec3(0.98, 0.90, 1.0), 0.38);
        addFramedStroke(col, alpha, nearestRing, 1.45, 3.0, ringCol, 0.76 * env);

        alpha = clamp(alpha * baseAlpha, 0.0, 0.78);
        if (alpha < 0.004) discard;
        outColor = vec4(col, alpha);
        return;
    }

    // -------------------------------------------------------------------------
    // Style 2: ESM / RWR 3-D FIM CRLB Uncertainty Ellipsoid
    //   Quiet scientific glass shell + clean equator and 2 orthogonal meridians.
    // -------------------------------------------------------------------------
    if (style < 2.5) {
        float faceWeight = gl_FrontFacing ? 1.0 : 0.22;
        vec3 col = baseCol;
        float alpha = (0.075 + fresnel * 0.24) * faceWeight;
        float eqDist = abs(rNorm - 0.5);
        float merDist = abs(fract((uCoord * 0.5 + 0.5) * 2.0 + 0.5) - 0.5);
        addFramedStroke(col, alpha, eqDist, 1.6, 3.2, mix(baseCol, vec3(1.0, 0.96, 0.80), 0.45), 0.78 * faceWeight);
        addFramedStroke(col, alpha, merDist, 1.15, 2.3, baseCol, 0.46 * faceWeight);
        alpha = clamp(alpha * softClip * baseAlpha, 0.0, 0.84);
        if (alpha < 0.004) discard;
        outColor = vec4(col, alpha);
        return;
    }

    // -------------------------------------------------------------------------
    // Style 3: 5-D Weapon DLZ / WEZ Horizontal Ruler Fan
    //   aux1: 0 = R_max amber, 1 = NEZ emerald, 2 = R_min crimson
    // -------------------------------------------------------------------------
    if (style < 3.5) {
        float fillBase = (aux1 > 0.5 && aux1 < 1.5) ? 0.19 : (aux1 >= 1.5 ? 0.20 : 0.085);
        vec3 col = baseCol;
        float alpha = fillBase;
        float rimR = min(1.0 - rNorm, rNorm);
        float rimAz = 1.0 - abs(uCoord);
        vec3 borderCol = mix(baseCol, vec3(0.96, 1.0, 0.96), 0.32);
        addFramedStroke(col, alpha, min(rimR, rimAz), 1.65, 3.4, borderCol, 0.88);
        alpha = clamp(alpha * softClip * baseAlpha, 0.0, 0.86);
        if (alpha < 0.004) discard;
        outColor = vec4(col, alpha);
        return;
    }

    // -------------------------------------------------------------------------
    // Style 4: Link 16 / C2 / Mode 5 Clean Arc (Zero Flashing Strobe Packets)
    // -------------------------------------------------------------------------
    if (style < 4.5) {
        float ends = smoothstep(0.0, 0.03, rNorm) * smoothstep(1.0, 0.97, rNorm);
        vec3 col = vec3(0.02, 0.04, 0.07);
        float alpha = 0.0;
        addFramedStroke(col, alpha, abs(uCoord), 1.6, 3.4, mix(baseCol, vec3(0.95, 1.0, 0.98), 0.25), 0.88 * ends);
        alpha = clamp(alpha * softClip * baseAlpha, 0.0, 0.90);
        if (alpha < 0.004) discard;
        outColor = vec4(col, alpha);
        return;
    }

    // -------------------------------------------------------------------------
    // Style 5: SAR Spotlight / Weather Radar Swath Footprint
    // -------------------------------------------------------------------------
    if (style < 5.5) {
        float frameDist = min(1.0 - abs(uCoord), min(1.0 - rNorm, rNorm));
        if (aux1 > 0.5) {
            float rad = length(vec2(uCoord, (rNorm - 0.5) * 2.0));
            float cell = smoothstep(1.0, 0.20, rad);
            vec3 col = baseCol;
            float alpha = 0.26 * cell;
            addFramedStroke(col, alpha, frameDist, 1.5, 3.0, baseCol, 0.65);
            alpha = clamp(alpha * softClip * baseAlpha, 0.0, 0.72);
            if (alpha < 0.004) discard;
            outColor = vec4(col, alpha);
            return;
        }
        vec3 col = baseCol;
        float alpha = 0.095;
        addFramedStroke(col, alpha, frameDist, 1.75, 3.5, mix(baseCol, vec3(0.92, 1.0, 1.0), 0.35), 0.86);
        alpha = clamp(alpha * softClip * baseAlpha, 0.0, 0.84);
        if (alpha < 0.004) discard;
        outColor = vec4(col, alpha);
        return;
    }

    // -------------------------------------------------------------------------
    // Style 6: Crisp Dark-Outlined Solid or Dashed 3-D Line / Ring
    // -------------------------------------------------------------------------
    float dash = 1.0;
    if (aux1 > 0.5) {
        dash = step(0.38, fract(rNorm * aux1));
    }
    if (dash < 0.5) discard;
    vec3 col = vec3(0.02, 0.04, 0.07);
    float alpha = 0.0;
    addFramedStroke(col, alpha, abs(uCoord), 1.55, 3.2, mix(baseCol, vec3(0.96, 0.99, 1.0), 0.25), 0.90);
    alpha = clamp(alpha * softClip * baseAlpha, 0.0, 0.92);
    if (alpha < 0.004) discard;
    outColor = vec4(col, alpha);
}
