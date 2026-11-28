#version 450

// VIS-M14 3-D Depth-Tested Tactical & EMSO Overlay Fragment Shader (v3 - Clean Precision Style).
// Restrained, non-flashy tactical drafting language: dark-haloed razor-sharp borders,
// range arcs, burn-through arcs, and quiet uniform translucent fills that remain effortlessly
// readable against both bright sunlit clouds and dark ocean — with zero flashy strobe waves,
// zero flickering static, and soft reverse-Z terrain occlusion.

layout(std140, set = 0, binding = 1) uniform TacticalFrame {
    vec4 projDepth; // x = p22, y = p32, z = 1/width, w = 1/height
    vec4 timeMeta;  // x = simTime, y = wallTime, z = daylight, w = unused
    vec4 upDir;     // xyz = localUp(eyeEcef), w = unused
    vec4 keyLight;  // xyz = sculptural 3/4 directional key light (ECEF), w = unused
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
    float facing = dot(nrm, viewDir);
    float ndv = abs(facing);
    float fresnel = pow(clamp(1.0 - ndv, 0.0, 1.0), 1.8);
    // Boost stroke contrast slightly at grazing Chase angles so shallow planes never wash out.
    float grazingBoost = mix(1.0, 1.38, pow(clamp(1.0 - ndv, 0.0, 1.0), 1.6));

    // Directional 3-value / 5-tone translucent sketch-geometry shading ("亮面 / 灰面 / 明暗交界线 / 暗面 / 微反光")
    // aligned with scene sun azimuth at a sculptural 3/4 elevation so every 3-D volume exhibits
    // clean airy light/mid/dark face separation over both bright clouds and dark sea.
    vec3 upAxis = normalize(frame.upDir.xyz + vec3(0.0, 0.0, 1e-6));
    vec3 keyDir = normalize(frame.keyLight.xyz + vec3(0.0, 0.0, 1e-6));
    // Unflipped geometric outward normal projections: view-independent light vs. shadow sides.
    float worldUpDot = dot(nrm, upAxis);
    float keyDot = dot(nrm, keyDir);
    float sideLight = smoothstep(-0.72, 0.72, keyDot);     // 0 = dark face, 0.5 = mid-tone, 1 = bright lit face
    float sideShadow = 1.0 - sideLight;
    // Subtle plaster-sketch terminator (core shadow ridge around keyDot ~ -0.18) & faint backside bounce light
    float terminator = exp(-pow((keyDot + 0.18) / 0.26, 2.0));
    float bounceLift = smoothstep(-0.35, -0.98, keyDot) * 0.14;

    vec3 baseCol = v_color.rgb;
    float baseAlpha = v_color.a;
    vec3 shadowTint = mix(vec3(0.025, 0.04, 0.08), baseCol * 0.18, 0.30); // Cool charcoal dark face (暗面)
    vec3 midTint    = mix(vec3(0.32, 0.38, 0.48), baseCol, 0.56);         // Clean neutral-tinted mid face (灰面)
    vec3 litTint    = mix(baseCol, vec3(0.97, 0.99, 1.0), 0.58);          // Crisp pale sunlit face (亮面)

    // -------------------------------------------------------------------------
    // Style 0: Radar Tactical Sector (Focused Aircraft Only)
    //   Clean subtle shaded tint + dark-outlined borders & 10 km range arcs +
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

        // Subtle 3-D depth shade: slightly deeper smoked tint along edges and underside
        float topShade = mix(0.56, 0.94, 0.5 + 0.5 * clamp(facing, -1.0, 1.0));
        float edgeShade = mix(1.0, 0.72, pow(abs(uCoord), 2.2));
        vec3 col = mix(shadowTint, baseCol, topShade * edgeShade);
        float alpha = (0.092 + 0.036 * pow(abs(uCoord), 2.4)) * apexFade;

        if (!inBlindZone) {
            // Subtle 10 km range arcs & center boresight tick
            float ringMask = smoothstep(0.06, 0.11, rNorm);
            addFramedStroke(col, alpha, ringDist, 1.30, 2.6, mix(baseCol, vec3(0.90, 0.97, 1.0), 0.38), clamp(0.54 * grazingBoost, 0.0, 0.90) * ringMask * apexFade);
            addFramedStroke(col, alpha, centerAzDist, 1.0, 2.0, baseCol, 0.28 * apexFade);

            // Crisp dark-outlined left/right sector edges & far edge (when unjammed)
            addFramedStroke(col, alpha, 1.0 - abs(uCoord), 2.0, 4.2, mix(baseCol, vec3(0.94, 0.98, 1.0), 0.30), clamp(0.92 * grazingBoost, 0.0, 0.96) * apexFade);
            if (!isJammed) {
                addFramedStroke(col, alpha, 1.0 - rNorm, 2.1, 4.4, mix(baseCol, vec3(0.94, 0.98, 1.0), 0.34), clamp(0.94 * grazingBoost, 0.0, 0.98) * apexFade);
            }

            // Crisp thin instantaneous radar beam ray
            float beamDist = abs(uCoord - aux2);
            addFramedStroke(col, alpha, beamDist, 1.45, 3.0, mix(baseCol, vec3(0.96, 1.0, 1.0), 0.72), 0.86 * apexFade);
        } else {
            // Quiet suppressed outer zone: whisper-subtle cool slate tint + static dashed outer frame
            float normOut = clamp((rNorm - burnFrac) / max(1.0 - burnFrac, 0.06), 0.0, 1.0);
            float fadeOut = mix(1.0, 0.35, normOut);
            col = vec3(0.36, 0.42, 0.52);
            alpha = 0.034 * fadeOut * apexFade;
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
    //   Zero flashy strobe waves: softly hemisphere-shaded translucent shell +
    //   crisp Fresnel rim + 3 clean static dark-outlined range contour rings.
    // -------------------------------------------------------------------------
    if (style < 1.5) {
        float faceWeight = gl_FrontFacing ? 1.0 : 0.22;
        float apexFade = smoothstep(0.0, 0.04, rNorm);
        float farFade = mix(1.0, 0.55, pow(rNorm, 1.3));
        float env = apexFade * farFade * softClip * faceWeight;

        // Plaster-cone 5-tone translucent shading: bright highlight -> mid-tone -> core shadow terminator -> dark side + subtle bounce
        vec3 coneLit = mix(baseCol, vec3(0.98, 0.95, 1.0), 0.54);
        vec3 col = (sideLight >= 0.5)
            ? mix(midTint, coneLit, (sideLight - 0.5) * 2.0)
            : mix(shadowTint, midTint, sideLight * 2.0);
        col = mix(col, shadowTint * 0.85, terminator * 0.45);
        col = mix(col, midTint, bounceLift);
        float alpha = (0.055 + sideShadow * 0.115 + terminator * 0.045 - bounceLift * 0.03 + fresnel * mix(0.20, 0.28, sideShadow)) * env;

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
    //   5-tone translucent sphere/ellipsoid sketch shading + equator/meridians.
    // -------------------------------------------------------------------------
    if (style < 2.5) {
        float faceWeight = gl_FrontFacing ? 1.0 : 0.24;
        vec3 ellLit = mix(baseCol, vec3(1.0, 0.99, 0.94), 0.54);
        vec3 col = (sideLight >= 0.5)
            ? mix(midTint, ellLit, (sideLight - 0.5) * 2.0)
            : mix(shadowTint, midTint, sideLight * 2.0);
        col = mix(col, shadowTint * 0.85, terminator * 0.42);
        col = mix(col, midTint, bounceLift);
        float alpha = (0.068 + sideShadow * 0.105 + terminator * 0.042 + fresnel * 0.24) * faceWeight;
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
        float fillBase = (aux1 > 0.5 && aux1 < 1.5) ? 0.175 : (aux1 >= 1.5 ? 0.185 : 0.082);
        float topShade = mix(0.54, 0.96, 0.5 + 0.5 * clamp(facing, -1.0, 1.0));
        float edgeShade = mix(1.0, 0.72, pow(abs(uCoord), 2.4));
        vec3 col = mix(shadowTint, baseCol, topShade * edgeShade);
        float alpha = fillBase + 0.032 * pow(abs(uCoord), 2.6);
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
            vec3 col = mix(shadowTint, baseCol, mix(0.55, 0.98, cell));
            float alpha = 0.24 * cell;
            addFramedStroke(col, alpha, frameDist, 1.5, 3.0, baseCol, 0.65);
            alpha = clamp(alpha * softClip * baseAlpha, 0.0, 0.72);
            if (alpha < 0.004) discard;
            outColor = vec4(col, alpha);
            return;
        }
        vec3 col = mix(shadowTint, baseCol, mix(0.68, 0.95, 1.0 - 0.5 * abs(uCoord)));
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
    if (style < 6.5) {
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
        return;
    }

    // -------------------------------------------------------------------------
    // Style 7: Light Translucent 3-D Volumetric Shade Wall / Cap / Gate Curtain
    //   aux1 = burnFrac (1.0 when not burn-through clipped)
    //   aux2 = wallKind:
    //     0.0 = radar elevation side wall (rNorm=range, uCoord=-1..+1 height, 0=mid-plane)
    //     0.5 = radar top ceiling / shadowed bottom floor (rNorm=range, uCoord=-1..+1 az)
    //     1.0 = upright DLZ gate curtain / side wall / swath skirt (rNorm=span, uCoord=-1..+1 height)
    //     1.5 = DLZ two-tier side wall with exact deck crease at uCoord=0 (below=drop shadow, above=wall)
    //     2.0 = curved burn-through / range portal glass pane (rNorm=az, uCoord=-1..+1 height)
    // -------------------------------------------------------------------------
    float wallKind = aux2;
    // Keep opposite side walls visible enough through the volume so bright-side vs. dark-side
    // contrast reads immediately from every azimuth around 360 deg.
    float backDamp = (facing >= 0.0) ? 1.0 : 0.76;

    if (wallKind < 0.75) {
        // Radar 3-D corridor side walls (0.0) and differential ceiling/floor shells (0.5)
        float burnFrac = clamp(aux1, 0.04, 1.0);
        bool isJammed = (burnFrac < 0.985);
        bool inBlindZone = isJammed && (rNorm > burnFrac);
        float apexFade = smoothstep(0.0, 0.045, rNorm);
        float farTaper = mix(1.0, 0.78, rNorm);
        bool isRoof = (wallKind >= 0.25);

        vec3 col;
        float alpha;
        if (!inBlindZone) {
            if (!isRoof) {
                // 3-value translucent sketch side walls:
                //   Sunlit side wall -> crisp pale 亮/浅灰面
                //   Shadow side wall -> cool charcoal 暗面 (clearly darker, still airy & see-through)
                //   + subtle vertical depth ramp, mid-plane AO crease & matching 10 km vertical ribs
                float vFrac = clamp(uCoord * 0.5 + 0.5, 0.0, 1.0);
                float lowerDepth = pow(clamp(1.0 - vFrac, 0.0, 1.0), 1.45);
                float midCrease = exp(-abs(uCoord) * 6.0);
                float railRim = smoothstep(0.76, 1.0, abs(uCoord));
                float rangeKm = rNorm * max(scaleM, 1000.0) * 1e-3;
                float ringStep = (scaleM > 28000.0) ? 10.0 : 5.0;
                float ringDist = fract(rangeKm / ringStep + 0.5) - 0.5;
                float vRib = aaLine(ringDist, 1.35) * smoothstep(0.07, 0.13, rNorm);

                vec3 faceTone = (sideLight >= 0.5)
                    ? mix(midTint, litTint, (sideLight - 0.5) * 2.0)
                    : mix(shadowTint, midTint, sideLight * 2.0);
                col = mix(shadowTint, faceTone, mix(0.62, 1.0, vFrac));
                col = mix(col, shadowTint, midCrease * mix(0.18, 0.42, sideShadow));
                col = mix(col, mix(shadowTint, litTint, sideLight * 0.80), vRib * 0.55);

                // Noticeable yet light translucent value contrast:
                // Bright face: airy pale sheen (~0.07-0.11); Dark face: cool charcoal shadow (~0.17-0.25)
                float sideBaseA = mix(0.068, 0.175, sideShadow);
                float lowerA = lowerDepth * mix(0.025, 0.065, sideShadow);
                alpha = (sideBaseA + lowerA + fresnel * mix(0.042, 0.078, sideShadow)
                         + midCrease * mix(0.022, 0.045, sideShadow) + railRim * 0.028 + vRib * 0.082)
                      * apexFade * farTaper * backDamp;
            } else {
                // Top ceiling = airy pale 亮面; Bottom floor = cool charcoal 底面暗部
                float sideCrease = smoothstep(0.68, 1.0, abs(uCoord));
                bool isBottomFloor = (worldUpDot < 0.0);
                if (isBottomFloor) {
                    float floorBack = (facing >= 0.0) ? 1.0 : 0.78;
                    col = mix(shadowTint, midTint, 0.16);
                    col = mix(col, shadowTint, sideCrease * 0.35);
                    alpha = (0.098 + fresnel * 0.055 + sideCrease * 0.045) * apexFade * farTaper * floorBack;
                } else {
                    float roofBack = (facing >= 0.0) ? 1.0 : 0.38;
                    col = litTint;
                    alpha = (0.058 + fresnel * 0.038 + sideCrease * 0.018) * apexFade * farTaper * roofBack;
                }
            }
        } else {
            float normOut = clamp((rNorm - burnFrac) / max(1.0 - burnFrac, 0.06), 0.0, 1.0);
            float fadeOut = mix(1.0, 0.24, normOut);
            if (!isRoof) {
                float vFrac = clamp(uCoord * 0.5 + 0.5, 0.0, 1.0);
                float lowerDepth = pow(clamp(1.0 - vFrac, 0.0, 1.0), 1.45);
                col = mix(vec3(0.09, 0.13, 0.20), vec3(0.74, 0.80, 0.88), sideLight);
                alpha = (mix(0.026, 0.078, sideShadow) + lowerDepth * 0.024 + fresnel * 0.026) * fadeOut * apexFade * backDamp;
            } else {
                bool isBottomFloor = (worldUpDot < 0.0);
                col = isBottomFloor ? vec3(0.12, 0.16, 0.24) : vec3(0.76, 0.82, 0.90);
                alpha = (isBottomFloor ? 0.036 : 0.018) * fadeOut * apexFade * backDamp;
            }
        }

        alpha = clamp(alpha * baseAlpha * softClip, 0.0, 0.34);
        if (alpha < 0.003) discard;
        outColor = vec4(col, alpha);
        return;
    }

    if (wallKind < 1.25) {
        // Upright gate curtain, rotodome cylinder skirt, or swath box side wall
        // 3-value translucent sketch wrap + core terminator + grounded bottom shadow
        float vFrac = clamp(uCoord * 0.5 + 0.5, 0.0, 1.0);
        float baseShadow = pow(1.0 - vFrac, 1.50);
        float topRim = smoothstep(0.84, 1.0, vFrac);
        float hEdge = smoothstep(0.62, 1.0, abs(rNorm * 2.0 - 1.0));
        vec3 faceTone = (sideLight >= 0.5)
            ? mix(midTint, litTint, (sideLight - 0.5) * 2.0)
            : mix(shadowTint, midTint, sideLight * 2.0);
        faceTone = mix(faceTone, shadowTint, terminator * 0.30);
        vec3 col = mix(shadowTint, faceTone, mix(0.48, 1.0, vFrac));
        col = mix(col, shadowTint, baseShadow * 0.28 + hEdge * 0.16);
        float alpha = (mix(0.056, 0.155, sideShadow) + terminator * 0.032 + baseShadow * mix(0.048, 0.105, sideShadow)
                       + hEdge * 0.046 + topRim * 0.024 + fresnel * 0.046) * backDamp;
        alpha = clamp(alpha * baseAlpha * softClip, 0.0, 0.34);
        if (alpha < 0.003) discard;
        outColor = vec4(col, alpha);
        return;
    }

    if (wallKind < 1.75) {
        // Two-tier DLZ architectural side walls & curved gate curtains:
        //   Crisp translucent 亮面 vs. 灰面 vs. 暗面 contrast + exact deck crease at uCoord=0
        float deckCrease = exp(-abs(uCoord) * 7.0);
        float endShadow = smoothstep(0.74, 1.0, abs(rNorm * 2.0 - 1.0));
        vec3 faceTone = (sideLight >= 0.5)
            ? mix(midTint, litTint, (sideLight - 0.5) * 2.0)
            : mix(shadowTint, midTint, sideLight * 2.0);
        faceTone = mix(faceTone, shadowTint, terminator * 0.28);
        vec3 col;
        float alpha;
        if (uCoord < 0.0) {
            // Beneath the fan deck: cool translucent vertical drop-shadow skirt (暗部投影)
            float dropFade = smoothstep(-1.0, -0.06, uCoord);
            col = mix(shadowTint, faceTone * 0.65, 0.30 * sideLight * dropFade);
            alpha = (mix(0.046, 0.128, sideShadow) + dropFade * mix(0.042, 0.105, sideShadow)
                     + deckCrease * 0.042 + fresnel * 0.048) * backDamp;
        } else {
            // Above the fan deck: light sunlit face vs. cool shaded dark face
            float lowerWall = pow(clamp(1.0 - uCoord, 0.0, 1.0), 1.45);
            float topLip = smoothstep(0.84, 1.0, uCoord);
            col = mix(shadowTint, faceTone, mix(0.56, 1.0, uCoord));
            col = mix(col, shadowTint, deckCrease * mix(0.18, 0.40, sideShadow));
            alpha = (mix(0.064, 0.172, sideShadow) + terminator * 0.028 + lowerWall * mix(0.028, 0.072, sideShadow)
                     + deckCrease * mix(0.022, 0.046, sideShadow) + topLip * 0.028
                     + endShadow * 0.036 + fresnel * 0.048) * backDamp;
        }
        alpha = clamp(alpha * baseAlpha * softClip, 0.0, 0.34);
        if (alpha < 0.003) discard;
        outColor = vec4(col, alpha);
        return;
    }

    // Curved burn-through / active-range portal glass pane:
    // Translucent 3-value sweep across the curved arc (bright wing -> mid center -> shaded wing), keeping center clear
    float vFrac = clamp(uCoord * 0.5 + 0.5, 0.0, 1.0);
    float lowerDepth = pow(clamp(1.0 - vFrac, 0.0, 1.0), 1.55);
    float frameProx = max(smoothstep(0.56, 1.0, abs(uCoord)), smoothstep(0.64, 1.0, abs(rNorm * 2.0 - 1.0)));
    vec3 portalLit = mix(baseCol, vec3(1.0, 0.98, 0.90), 0.54);
    vec3 faceTone = (sideLight >= 0.5)
        ? mix(midTint, portalLit, (sideLight - 0.5) * 2.0)
        : mix(shadowTint, midTint, sideLight * 2.0);
    vec3 col = mix(shadowTint, faceTone, mix(0.45, 1.0, vFrac));
    col = mix(col, shadowTint, lowerDepth * 0.28 + terminator * 0.22);
    float alpha = (mix(0.032, 0.095, sideShadow) + terminator * 0.024 + lowerDepth * mix(0.038, 0.088, sideShadow)
                   + frameProx * 0.084 + fresnel * 0.044) * backDamp;
    alpha = clamp(alpha * baseAlpha * softClip, 0.0, 0.32);
    if (alpha < 0.003) discard;
    outColor = vec4(col, alpha);
}
