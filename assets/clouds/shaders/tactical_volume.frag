#version 450

// VIS-M14 3-D Depth-Tested Volumetric Tactical & EMSO Overlay Fragment Shader (v2).
// Uses standard alpha blending (SRC_ALPHA, ONE_MINUS_SRC_ALPHA) with rich saturated
// tactical hues so volumes remain crisp, high-contrast, and unmistakable even over
// bright sunlit clouds and ocean (zero white-out), with reverse-Z terrain occlusion.

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

void main() {
    float style = v_params.x;
    float aux1 = v_params.y;
    float aux2 = v_params.z;
    float scaleM = v_params.w;
    float rNorm = clamp(v_posR.w, 0.0, 1.0);
    float uCoord = v_normU.w;
    float t = frame.timeMeta.y;

    vec2 screenUv = gl_FragCoord.xy * frame.projDepth.zw;
    float sceneZ = texture(sceneDepthTex, screenUv).r;
    float fragDist = linearDepth(gl_FragCoord.z);
    float sceneDist = (sceneZ > 1e-7) ? linearDepth(sceneZ) : 1.0e8;
    float depthDiff = sceneDist - fragDist;
    if (depthDiff <= 0.0) discard;

    // Soft occlusion at terrain/airframe + subtle crisp terrain-intersection contour
    float softClip = smoothstep(0.0, clamp(fragDist * 0.002, 2.0, 20.0), depthDiff);
    float groundKiss = exp(-depthDiff * 0.09) * smoothstep(0.0, 2.5, depthDiff) * (1.0 - smoothstep(45.0, 90.0, depthDiff));

    vec3 viewDir = normalize(-v_posR.xyz);
    vec3 nrm = normalize(v_normU.xyz + vec3(1e-7));
    float ndv = abs(dot(nrm, viewDir));
    float fresnel = pow(clamp(1.0 - ndv, 0.0, 1.0), 1.85);

    vec3 baseCol = v_color.rgb;
    float baseAlpha = v_color.a;

    // -------------------------------------------------------------------------
    // Style 0: Radar Tactical Sector (Blue = Sapphire-Cyan, Red = Vivid Crimson)
    //   aux1 = burnThroughFrac (1.0 unjammed, < 0.98 jammed)
    //   aux2 = instantaneous beamAzNorm in [-1, 1]
    //   scaleM = displayed sector range (m)
    // -------------------------------------------------------------------------
    if (style < 0.5) {
        float burnFrac = clamp(aux1, 0.04, 1.0);
        bool isJammed = (burnFrac < 0.985);
        bool inBlindZone = isJammed && (rNorm > burnFrac);

        float apexFade = smoothstep(0.0, 0.035, rNorm);

        // Crisp concentric range rings every 5 km (<= 30 km) or 10 km (> 30 km)
        float rangeKm = rNorm * max(scaleM, 1000.0) * 1e-3;
        float ringStep = (scaleM > 32000.0) ? 10.0 : 5.0;
        float ringCoord = fract(rangeKm / ringStep + 0.5) - 0.5;
        float rangeRing = aaLine(ringCoord, 1.25) * smoothstep(0.05, 0.10, rNorm);

        // Subtle radial azimuth spokes every 15 deg (uCoord in [-1, 1])
        float azSpoke = aaLine(fract((uCoord * 0.5 + 0.5) * 4.0 + 0.5) - 0.5, 1.05) * 0.55;

        // Sharp anti-aliased outer perimeter edges (left/right sector walls + far arc)
        float edgeAz = aaLine(1.0 - abs(uCoord), 2.0);
        float edgeFar = aaLine(1.0 - rNorm, 2.2);
        float border = max(edgeAz, edgeFar);

        // Crisp live sweeping radar beam slice + smooth trailing phosphor wedge
        float beamDist = abs(uCoord - aux2);
        float beamLine = exp(-beamDist * beamDist * 260.0) * (inBlindZone ? 0.08 : 1.0);
        float beamTrail = exp(-beamDist * 11.0) * (inBlindZone ? 0.04 : 0.42);

        vec3 col = baseCol;
        float alpha = (0.13 + 0.14 * fresnel + rangeRing * 0.22 + azSpoke * 0.10 + beamTrail * 0.18) * apexFade;
        // Bright saturated rim & crisp white-tinted scan beam (never washed-out fog!)
        col = mix(col, mix(baseCol, vec3(0.92, 0.98, 1.0), 0.55), clamp(border * 0.72 + beamLine * 0.90 + rangeRing * 0.30, 0.0, 1.0));
        alpha = max(alpha, max(border * 0.72, beamLine * 0.80) * apexFade);

        if (inBlindZone) {
            // Suppressed outer zone: clearly separated dark-violet diagonal RF static hash fading softly away
            float normOut = clamp((rNorm - burnFrac) / max(1.0 - burnFrac, 0.06), 0.0, 1.0);
            float fadeOut = pow(1.0 - normOut, 1.25);
            float hatch = aaLine(fract(rNorm * 28.0 - uCoord * 10.0 - t * 1.4) - 0.5, 1.4);
            col = mix(vec3(0.28, 0.10, 0.42), baseCol * 0.45, 0.30);
            col = mix(col, vec3(0.74, 0.36, 0.96), hatch * 0.65);
            alpha = (0.075 + hatch * 0.16 + border * 0.28) * fadeOut * apexFade;
        }

        // Razor-sharp luminous Gold-Amber Burn-Through arc line on the sector deck!
        if (isJammed) {
            float btEdge = aaLine(rNorm - burnFrac, 2.4);
            float btHalo = exp(-pow((rNorm - burnFrac) * 65.0, 2.0)) * 0.48;
            float bt = clamp(btEdge + btHalo * 0.45, 0.0, 1.0);
            if (bt > 0.02) {
                vec3 btCol = mix(vec3(1.00, 0.76, 0.10), vec3(1.00, 0.98, 0.85), btEdge * 0.72);
                col = mix(col, btCol, clamp(bt * 1.1, 0.0, 1.0));
                alpha = max(alpha, bt * 0.86);
            }
        }

        // Crisp terrain-intersection highlight
        col = mix(col, vec3(1.0), groundKiss * 0.35);
        alpha = clamp((alpha + groundKiss * 0.20) * baseAlpha * softClip, 0.0, 0.86);
        if (alpha < 0.004) discard;
        outColor = vec4(col, alpha);
        return;
    }

    // -------------------------------------------------------------------------
    // Style 1: EA Jamming Volumetric RF Cone (Royal Violet + Fiery Amber Wavefronts)
    //   Back faces are kept ultra-subtle so the 3-D cone never double-stacks!
    // -------------------------------------------------------------------------
    if (style < 1.5) {
        float faceWeight = gl_FrontFacing ? 1.0 : 0.16;
        float apexFade = smoothstep(0.0, 0.04, rNorm);
        float farFade = pow(clamp(1.0 - rNorm, 0.0, 1.0), 1.35);

        // Crisp outward-racing RF interference wavefront rings (unmistakable animated EW signature!)
        float wavePhase = fract(rNorm * 10.5 - t * 1.9);
        float waveCrest = smoothstep(0.72, 0.90, wavePhase) * smoothstep(1.0, 0.90, wavePhase);
        float fineRipple = 0.5 + 0.5 * sin(rNorm * 64.0 + uCoord * 18.0 - t * 11.0);
        float silhouette = pow(clamp(1.0 - ndv, 0.0, 1.0), 1.95);

        vec3 deepBody = (aux2 > 0.5) ? vec3(0.84, 0.36, 0.02) : baseCol;
        vec3 hotCrest = (aux2 > 0.5) ? vec3(1.00, 0.86, 0.24) : vec3(1.00, 0.50, 0.08);
        vec3 col = mix(deepBody, hotCrest, clamp(waveCrest * 0.92 + fineRipple * 0.14, 0.0, 0.95));

        float env = apexFade * farFade * softClip * faceWeight;
        float alpha = (0.060 + silhouette * 0.28 + waveCrest * 0.38 * (0.45 + 0.55 * silhouette)) * env * baseAlpha;
        alpha = clamp(alpha + groundKiss * 0.22 * env, 0.0, 0.74);
        if (alpha < 0.004) discard;
        outColor = vec4(col, alpha);
        return;
    }

    // -------------------------------------------------------------------------
    // Style 2: ESM / RWR Tapered Gold Bearing Lobe OR 3-D FIM CRLB Ellipsoid
    //   aux1 > 0.5 => 3-D CRLB Ellipsoid; aux1 <= 0.5 => Tapered DF Lobe
    // -------------------------------------------------------------------------
    if (style < 2.5) {
        if (aux1 > 0.5) {
            float faceWeight = gl_FrontFacing ? 1.0 : 0.22;
            float latLine = aaLine(fract(rNorm * 4.0 + 0.5) - 0.5, 1.35);
            float lonLine = aaLine(fract(uCoord * 4.0 + 0.5) - 0.5, 1.35);
            float equator = aaLine(rNorm - 0.5, 2.0);
            float grid = max(max(latLine, lonLine) * 0.55, equator);
            vec3 col = mix(baseCol * 0.85, vec3(1.0, 0.96, 0.76), grid * 0.65);
            float alpha = clamp((0.12 + fresnel * 0.32 + grid * 0.42) * faceWeight * softClip * baseAlpha, 0.0, 0.82);
            if (alpha < 0.004) discard;
            outColor = vec4(col, alpha);
            return;
        }
        // Sleek tapered golden bearing wedge with crisp edges + animated directional ticks
        float across = 1.0 - abs(uCoord);
        float edge = aaLine(1.0 - abs(uCoord), 1.8);
        float centerLine = aaLine(uCoord, 1.2) * 0.55;
        float along = smoothstep(0.0, 0.04, rNorm) * pow(clamp(1.0 - rNorm, 0.0, 1.0), 1.15);
        float chevron = smoothstep(0.72, 0.92, fract(rNorm * 7.0 - abs(uCoord) * 0.6 - t * 1.4));
        vec3 col = mix(baseCol, vec3(1.0, 0.95, 0.68), clamp(edge * 0.5 + chevron * 0.65 + centerLine * 0.4, 0.0, 1.0));
        float alpha = clamp((0.10 * across + edge * 0.48 + chevron * 0.26 * across + centerLine * 0.25) * along * softClip * baseAlpha, 0.0, 0.72);
        if (alpha < 0.004) discard;
        outColor = vec4(col, alpha);
        return;
    }

    // -------------------------------------------------------------------------
    // Style 3: 5-D Weapon DLZ / WEZ Color-Zoned 3-D Tactical Fan Deck
    //   aux1: 0 = R_max outer amber, 1 = NEZ emerald kill zone, 2 = R_min crimson collar
    // -------------------------------------------------------------------------
    if (style < 3.5) {
        float rimR = max(aaLine(1.0 - rNorm, 2.2), aaLine(rNorm, 2.0));
        float rimAz = aaLine(1.0 - abs(uCoord), 2.0);
        float border = max(rimR, rimAz);
        float tick = aaLine(fract((uCoord * 0.5 + 0.5) * 4.0 + 0.5) - 0.5, 1.1) * 0.30;
        float fillBase = (aux1 > 0.5 && aux1 < 1.5) ? 0.28 : (aux1 >= 1.5 ? 0.26 : 0.12);
        vec3 col = mix(baseCol, vec3(0.96, 1.00, 0.96), border * 0.55);
        float alpha = clamp((fillBase + border * 0.56 + tick * 0.14) * softClip * baseAlpha, 0.0, 0.86);
        if (alpha < 0.004) discard;
        outColor = vec4(col, alpha);
        return;
    }

    // -------------------------------------------------------------------------
    // Style 4: Link 16 / DMS / C2 Consent Elevated Parabolic Energy Arc
    // -------------------------------------------------------------------------
    if (style < 4.5) {
        float core = exp(-uCoord * uCoord * 5.0);
        float pktPhase = fract(rNorm * 2.5 - t * 0.85);
        float packet = pow(clamp(1.0 - abs(pktPhase - 0.5) * 6.0, 0.0, 1.0), 2.2);
        float ends = smoothstep(0.0, 0.03, rNorm) * smoothstep(1.0, 0.97, rNorm);
        vec3 col = mix(baseCol, vec3(1.0, 1.0, 1.0), packet * 0.72);
        float alpha = clamp((0.36 + packet * 0.56) * core * ends * softClip * baseAlpha, 0.0, 0.92);
        if (alpha < 0.004) discard;
        outColor = vec4(col, alpha);
        return;
    }

    // -------------------------------------------------------------------------
    // Style 5: SAR Spotlight / Weather Radar Swath Footprint
    // -------------------------------------------------------------------------
    if (style < 5.5) {
        float frameEdge = max(aaLine(1.0 - abs(uCoord), 2.2), max(aaLine(1.0 - rNorm, 2.2), aaLine(rNorm, 2.2)));
        if (aux1 > 0.5) {
            float rad = length(vec2(uCoord, (rNorm - 0.5) * 2.0));
            float cell = smoothstep(1.02, 0.18, rad);
            float alpha = clamp((0.34 * cell + frameEdge * 0.36) * softClip * baseAlpha, 0.0, 0.74);
            if (alpha < 0.004) discard;
            outColor = vec4(baseCol, alpha);
            return;
        }
        float scanLine = aaLine(rNorm - fract(t * 0.35), 2.2);
        float grid = max(aaLine(fract(rNorm * 4.0) - 0.5, 1.1), aaLine(fract((uCoord * 0.5 + 0.5) * 4.0) - 0.5, 1.1)) * 0.25;
        vec3 col = mix(baseCol, vec3(0.92, 1.0, 1.0), max(frameEdge, scanLine) * 0.55);
        float alpha = clamp((0.14 + frameEdge * 0.58 + grid * 0.16 + scanLine * 0.45) * softClip * baseAlpha, 0.0, 0.82);
        if (alpha < 0.004) discard;
        outColor = vec4(col, alpha);
        return;
    }

    // -------------------------------------------------------------------------
    // Style 6: Crisp Soft-Antialiased 3-D Volumetric Beam / Ring / Dashed Track
    // -------------------------------------------------------------------------
    float core = exp(-uCoord * uCoord * 5.2);
    float dash = 1.0;
    if (aux1 > 0.5) {
        dash = smoothstep(0.30, 0.46, fract(rNorm * aux1 - t * aux2));
    }
    vec3 col = mix(baseCol, vec3(1.0, 1.0, 1.0), core * 0.42);
    float alpha = clamp(core * dash * softClip * baseAlpha * 0.88, 0.0, 0.92);
    if (alpha < 0.004) discard;
    outColor = vec4(col, alpha);
}
