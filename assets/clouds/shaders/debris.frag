#version 450

// Sunlit PBR metallic/charred shading + physical blackbody incandescence along
// fractured edges of tumbling 6-DOF debris chunks (VIS-M11).

layout(std140, set = 0, binding = 1) uniform Frame {
    vec4 sunDirLight; // xyz = sun dir ECEF, w = daylight (0..1)
    vec4 eyeRight;
    vec4 eyeUp;
    vec4 eyeMeta;     // xyz = eye up ECEF, w = simTime
} frame;

layout(location = 0) in vec3 v_posRelEye;
layout(location = 1) in vec3 v_normalEcef;
layout(location = 2) in vec3 v_baseColor;
layout(location = 3) in vec4 v_params; // x = tempK, y = scorch, z = edgeGlow, w = tone

layout(location = 0) out vec4 outColor;

vec3 blackbodyRadiance(float tempK) {
    // Physical blackbody ramp from 750 K (deep cherry ember) to 2200 K (hot orange-gold).
    float t = clamp((tempK - 750.0) / 1450.0, 0.0, 1.0);
    if (t <= 0.0) return vec3(0.0);
    vec3 c1 = vec3(0.82, 0.05, 0.004); // dark cherry ember
    vec3 c2 = vec3(1.00, 0.28, 0.02);  // fiery orange
    vec3 c3 = vec3(1.00, 0.68, 0.22);  // bright amber-gold
    vec3 col = t < 0.45 ? mix(c1, c2, t / 0.45) : mix(c2, c3, (t - 0.45) / 0.55);
    float intensity = pow(t, 2.6) * 1.65;
    return col * intensity;
}

void main() {
    vec3 N = normalize(v_normalEcef);
    if (!gl_FrontFacing) N = -N;
    vec3 V = normalize(-v_posRelEye);
    vec3 L = normalize(frame.sunDirLight.xyz);
    vec3 U = normalize(frame.eyeMeta.xyz);
    float daylight = clamp(frame.sunDirLight.w, 0.05, 1.0);

    float tempK = v_params.x;
    float scorch = clamp(v_params.y, 0.0, 1.0);
    float edgeGlow = clamp(v_params.z, 0.0, 1.0);
    float tone = clamp(v_params.w, 0.0, 1.0);

    // Blend painted aircraft aluminum toward matte soot/carbon charring on scorched & fractured zones.
    vec3 metal = v_baseColor * mix(0.45, 1.05, tone);
    vec3 charred = vec3(0.028, 0.026, 0.030);
    float charMix = clamp(scorch * 0.88 + edgeGlow * 0.45, 0.15, 0.94);
    vec3 albedo = mix(metal, charred, charMix);

    float ndl = max(dot(N, L), 0.0);
    float hemi = 0.5 + 0.5 * dot(N, U);
    vec3 skyAmb = mix(vec3(0.05, 0.06, 0.08), vec3(0.18, 0.24, 0.32), hemi) * daylight;
    vec3 sunCol = vec3(1.50, 1.42, 1.30) * daylight;

    // Sharp metallic glint on uncharred aircraft aluminum as shards tumble in sunlight.
    vec3 H = normalize(L + V);
    float specPow = mix(48.0, 10.0, charMix);
    float spec = pow(max(dot(N, H), 0.0), specPow) * mix(0.65, 0.05, charMix) * ndl;
    vec3 lit = albedo * (skyAmb + sunCol * ndl) + sunCol * spec;

    // Confine blackbody glow strictly to thin sheared fracture lips so solid chunks keep dark metal contrast.
    float rimMask = pow(edgeGlow, 3.6);
    float localTempK = tempK * mix(0.28, 1.02, rimMask);
    vec3 emissive = blackbodyRadiance(localTempK);
    vec3 color = lit + emissive;

    // Subtle aerial perspective fade at multi-kilometre ranges.
    float dist = length(v_posRelEye);
    float haze = 1.0 - exp(-dist / 24000.0);
    vec3 horizon = mix(vec3(0.10, 0.12, 0.16), vec3(0.52, 0.62, 0.76), daylight);
    color = mix(color, horizon, haze * 0.65);

    outColor = vec4(color, 1.0);
}
