#version 450

// VIS-M13 Unified Indirect GPU Particle Vertex Shader:
// Renders both Bin 0 (alpha smoke / chaff haze / muzzle gas / ejecta / mist) and
// Bin 1 (additive HDR sparks / burning embers / chaff metallic dipole glints)
// from the GPU-compacted `alive[]` index buffer.

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelView;
} pc;

struct GpuParticle {
    vec4 posAge;     // xyz = origin-relative ECEF (m), w = ageS
    vec4 velLife;    // xyz = velocity ECEF (m/s), w = maxLifeS
    vec4 colorSize;  // rgb = linear HDR color, w = baseRadiusM
    vec4 params;     // x = kind, y = dragPerS, z = seed, w = extra
};

layout(std430, set = 0, binding = 0) readonly buffer ParticlePool {
    GpuParticle particles[];
};

layout(std430, set = 0, binding = 1) readonly buffer AliveIndices {
    uint alive[];
};

layout(std140, set = 0, binding = 2) uniform RenderFrame {
    vec4 sunDirLight;   // xyz = sun dir ECEF, w = daylight
    vec4 eyeRight;      // xyz = camera right ECEF, w = 0
    vec4 eyeUp;         // xyz = camera up ECEF, w = 0
    vec4 eyeMeta;       // xyz = local zenith ECEF, w = simTime
    vec4 originToEye;   // xyz = originEcef - eyeEcef, w = 0
    vec4 projDepth;     // P22, P32, 1/W, 1/H
    vec4 flowLaid;      // x0, y0, x1, y1 full-res Flow scissor rect
    vec4 flowMeta;      // x = half-res scale
} frame;

layout(location = 0) out vec2 v_uv;
layout(location = 1) out vec4 v_color;    // rgb = linear color, a = base opacity / intensity
layout(location = 2) out vec4 v_params;   // x = kind, y = ageNorm, z = seed, w = extra
layout(location = 3) out vec4 v_lighting; // x = sunDot, y = upDot, z = distM, w = softRangeM

const vec2 kCorners[6] = vec2[](
    vec2(-1.0, -1.0), vec2( 1.0, -1.0), vec2( 1.0,  1.0),
    vec2(-1.0, -1.0), vec2( 1.0,  1.0), vec2(-1.0,  1.0)
);

void main() {
    uint pIdx = alive[gl_InstanceIndex];
    GpuParticle p = particles[pIdx];
    vec2 corner = kCorners[gl_VertexIndex];

    float maxLife = max(p.velLife.w, 1e-3);
    float ageNorm = clamp(p.posAge.w / maxLife, 0.0, 1.0);
    float kind = p.params.x;
    float seed = p.params.z;
    float baseR = max(p.colorSize.w, 0.03);

    vec3 center = p.posAge.xyz + frame.originToEye.xyz;
    float centerDist = max(length(center), 1e-3);
    vec3 toEye = -center / centerDist;
    vec3 right = frame.eyeRight.xyz;
    vec3 up = frame.eyeUp.xyz;
    vec3 zenith = normalize(frame.eyeMeta.xyz);

    vec3 offset;
    float radius = baseR;
    float softRange = 0.45;
    float alpha = 1.0;

    if (kind > 0.5 && kind < 1.5) {
        // Kind 1: Sleek velocity-stretched incandescent spark / burning ember streak
        vec3 vel = p.velLife.xyz;
        vec3 projVel = vel - toEye * dot(vel, toEye);
        float pLen = length(projVel);
        vec3 dir = (pLen > 1e-2) ? (projVel / pLen) : up;
        vec3 perp = normalize(cross(toEye, dir));
        float sparkFade = 1.0 - smoothstep(2200.0, 6200.0, centerDist);
        float distFloor = clamp(centerDist * 0.00028, 0.0, 0.055) * sparkFade;
        radius = (sparkFade > 1e-4) ? (baseR * (1.0 - 0.38 * ageNorm) + distFloor) : 0.0;
        float stretch = clamp(p.params.w * (0.45 + 0.55 * clamp(pLen / 85.0, 0.20, 1.5)), 1.6, 5.4);
        offset = dir * (corner.x * radius * stretch) + perp * (corner.y * radius);
        softRange = 0.25;
        alpha = sparkFade;
    } else if (kind > 5.5) {
        // Kind 6: Delicate twinkling metallic chaff aluminium-foil dipole flecks (tiny sparkling diamonds)
        float ang = seed * 6.2831853 + p.posAge.w * (7.0 + 8.0 * fract(seed * 7.3));
        float ca = cos(ang), sa = sin(ang);
        float chaffFade = 1.0 - smoothstep(2500.0, 7500.0, centerDist);
        float distFloor = clamp(centerDist * 0.00035, 0.0, 0.075) * chaffFade;
        radius = (chaffFade > 1e-4) ? (baseR + distFloor) : 0.0;
        vec2 rot = vec2(corner.x * 1.45 * ca - corner.y * 0.65 * sa, corner.x * 1.45 * sa + corner.y * 0.65 * ca);
        offset = right * (rot.x * radius) + up * (rot.y * radius);
        softRange = 0.30;
        alpha = chaffFade;
    } else {
        // Alpha volumetric puffs (Kind 0 FlareSmoke, 2 ChaffHaze, 3 MuzzleSmoke, 4 SurfaceEjecta, 5 ShockMist)
        float growth = 2.6;
        if (kind < 0.5) growth = 3.6;       // Flare MgO turbulent white smoke expands naturally into a ~2 m plume
        else if (kind < 2.5) growth = 3.0;  // Chaff dipole cloud blooms softly
        else if (kind < 3.5) growth = 2.0;  // Muzzle gas / HEI puff
        else if (kind < 4.5) growth = 3.0;  // Soil / water ejecta plume
        else growth = 2.2;
        radius = baseR * (1.0 + growth * sqrt(ageNorm));
        float spin = (fract(seed * 13.7) - 0.5) * 3.2;
        float ang = seed * 6.2831853 + ageNorm * spin;
        float ca = cos(ang), sa = sin(ang);
        vec2 rot = vec2(corner.x * ca - corner.y * sa, corner.x * sa + corner.y * ca);
        offset = right * (rot.x * radius) + up * (rot.y * radius);
        softRange = max(radius * 0.72, 0.45);
        alpha = 1.0;
    }

    vec3 posRel = center + offset;
    v_uv = corner;
    v_color = vec4(p.colorSize.rgb, alpha);
    v_params = vec4(kind, ageNorm, seed, p.params.w);

    vec3 sun = normalize(frame.sunDirLight.xyz);
    vec3 sphereN = normalize(offset + toEye * max(radius * 0.85, 0.1));
    v_lighting = vec4(0.5 + 0.5 * dot(sphereN, sun), 0.5 + 0.5 * dot(sphereN, zenith), centerDist, softRange);
    gl_Position = pc.projection * pc.modelView * vec4(posRel, 1.0);
}
