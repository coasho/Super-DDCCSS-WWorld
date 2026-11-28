#version 450

// VIS-M13 Direct Exact-Pose Combat Ribbon & Core Vertex Shader:
//   Kind 0 = Gun tracer streak (velocity-aligned HDR beam) & subtle non-tracer ballistic slug
//   Kind 1 = Gun muzzle flash volumetric cone & star burst
//   Kind 2 = Pyrotechnic IR flare blazing magnesium core + 6-point optical diffraction star
//   Kind 3 = Expanding supersonic detonation Mach shockwave sphere (Fresnel shell)

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelView;
} pc;

struct CombatRibbon {
    vec4 posRadius;      // xyz = eye-relative ECEF (m), w = width / radius (m)
    vec4 dirLength;      // xyz = unit axis ECEF, w = length (m) or sphere radius (m)
    vec4 colorIntensity; // rgb = linear HDR color, a = intensity / opacity
    vec4 params;         // x = kind (0..3), y = ageNorm (0..1), z = seed, w = extra
};

layout(std430, set = 0, binding = 0) readonly buffer Ribbons {
    CombatRibbon ribbons[];
};

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

layout(location = 0) out vec2 v_uv;
layout(location = 1) out vec4 v_color;    // rgb = linear HDR color, a = intensity
layout(location = 2) out vec4 v_params;   // x = kind, y = ageNorm, z = seed, w = extra
layout(location = 3) out vec4 v_lighting; // x = distM, y = radiusM, z = softRangeM, w = sunDot

const vec2 kCorners[6] = vec2[](
    vec2(-1.0, -1.0), vec2( 1.0, -1.0), vec2( 1.0,  1.0),
    vec2(-1.0, -1.0), vec2( 1.0,  1.0), vec2(-1.0,  1.0)
);

void main() {
    CombatRibbon r = ribbons[gl_InstanceIndex];
    vec2 corner = kCorners[gl_VertexIndex];
    float width = r.posRadius.w;
    if (width <= 0.0 || r.colorIntensity.a <= 1e-4) {
        gl_Position = vec4(0.0, 0.0, -2.0, 1.0);
        v_uv = vec2(0.0);
        v_color = vec4(0.0);
        v_params = vec4(0.0);
        v_lighting = vec4(0.0);
        return;
    }

    vec3 center = r.posRadius.xyz;
    float distM = max(length(center), 1e-3);
    vec3 toEye = -center / distM;
    vec3 right = frame.eyeRight.xyz;
    vec3 up = frame.eyeUp.xyz;
    float kind = r.params.x;

    vec3 offset;
    float softRange = 0.35;
    vec4 outCol = r.colorIntensity;
    float lightingExtra = 0.5 + 0.5 * dot(normalize(toEye), normalize(frame.sunDirLight.xyz));
    float daylight = clamp(frame.sunDirLight.w, 0.08, 1.0);

    if (kind < 0.5) {
        // Kind 0: Gun tracer streak aligned strictly along projectile velocity vector.
        // Bounded sub-pixel visibility floor + realistic long-range extinction so 20 mm rounds vanish at synoptic range.
        vec3 axis = normalize(r.dirLength.xyz);
        vec3 sideRaw = cross(toEye, axis);
        vec3 side = (length(sideRaw) > 1e-3) ? normalize(sideRaw) : right;
        float halfLen = 0.5 * max(r.dirLength.w, 1.0);
        float tracerFade = 1.0 - smoothstep(4200.0, 9200.0, distM);
        float halfW = (tracerFade > 1e-4) ? max(width, min(distM * 0.00055, 1.35)) : 0.0;
        offset = axis * ((corner.x - 0.65) * halfLen) + side * (corner.y * halfW);
        outCol.a *= tracerFade;
        softRange = 0.25;
    } else if (kind < 1.5) {
        // Kind 1: Compact muzzle flash directional cone & petals anchored at gun barrel nozzle
        vec3 axis = normalize(r.dirLength.xyz);
        vec3 sideRaw = cross(toEye, axis);
        vec3 side = (length(sideRaw) > 1e-3) ? normalize(sideRaw) : right;
        float halfLen = 0.5 * max(r.dirLength.w, 0.8);
        float halfW = max(width, 0.30);
        offset = axis * ((corner.x + 1.0) * halfLen) + side * (corner.y * halfW);
        softRange = 0.20;
    } else if (kind < 2.5) {
        // Kind 2: Physically-scaled MTV pyrotechnic IR flare core (e.g. MJU-7/B ~0.35 m flame envelope).
        // Optical inverse-square irradiance + atmospheric extinction + day/night sky contrast:
        // crisp fiery core up close, delicate pinpoint spark at 2-6 km, completely invisible from regional/orbital views!
        float peakRefDist = mix(3200.0, 1150.0, daylight);
        float atmExt = exp(-distM * 0.00022);
        float irrad = (peakRefDist * peakRefDist) / (distM * distM + peakRefDist * peakRefDist) * atmExt;
        float visCutoff = 1.0 - smoothstep(mix(12000.0, 4800.0, daylight), mix(26000.0, 10500.0, daylight), distM);
        float apparentEnergy = irrad * visCutoff;
        float aaFloor = distM * 0.0011 * pow(max(apparentEnergy, 0.0), 0.38);
        float rad = (apparentEnergy > 1e-4) ? max(width, min(aaFloor, 2.2)) : 0.0;
        offset = right * (corner.x * rad) + up * (corner.y * rad);
        outCol.a *= clamp(apparentEnergy * 1.15, 0.0, 1.0);
        lightingExtra = apparentEnergy;
        softRange = 0.22;
    } else {
        // Kind 3: Expanding supersonic detonation shockwave sphere (camera-facing analytic sphere disc)
        float rad = max(r.dirLength.w, 1.0);
        offset = right * (corner.x * rad) + up * (corner.y * rad);
        softRange = max(rad * 0.14, 1.5);
        lightingExtra = 0.5 + 0.5 * dot(normalize(offset + toEye), normalize(frame.sunDirLight.xyz));
    }

    vec3 posRel = center + offset;
    v_uv = corner;
    v_color = outCol;
    v_params = r.params;
    v_lighting = vec4(distM, max(r.dirLength.w, width), softRange, lightingExtra);
    gl_Position = pc.projection * pc.modelView * vec4(posRel, 1.0);
}
