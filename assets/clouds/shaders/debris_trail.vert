#version 450

// Instanced scene-depth soft-blended billboards & surface-aligned quads for:
//   kind 0 = continuous pitch-black volumetric smoke ribbons
//   kind 1 = incandescent white-orange velocity-stretched sparks (additive)
//   kind 2 = multi-layered 6-way sunlit hydrodynamic ocean water geyser
//   kind 3 = tight turbulent flame cores clinging to burning chunks (additive)
//   kind 4 = horizontal ocean surface churning white foam wake
//   kind 5 = billowing 6-way sunlit terrain soil/dust ejecta cloud
//   kind 6 = horizontal terrain-aligned charred ground skid furrow (VIS-M11 v2)

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelView;
} pc;

struct TrailQuad {
    vec4 posAndRadius;  // xyz = eye-relative ECEF (m), w = radius (m)
    vec4 velAndTemp;    // xyz = velocity/align dir ECEF (m/s), w = temperature (K) or secondary scale
    vec4 colorAlpha;    // rgb = base albedo/tint, a = opacity
    vec4 params;        // x = kind (0..6), y = ageNorm (0..1), z = seed/arcDist, w = stretch
};

layout(std430, set = 0, binding = 0) readonly buffer Quads {
    TrailQuad quads[];
};

layout(std140, set = 0, binding = 1) uniform Frame {
    vec4 sunDirLight; // xyz = sun dir ECEF, w = daylight (0..1)
    vec4 eyeRight;    // xyz = camera right ECEF
    vec4 eyeUp;       // xyz = camera up ECEF
    vec4 eyeMeta;     // xyz = local zenith ECEF, w = simTime
    vec4 projDepth;   // x = P[2][2], y = P[3][2], z = 1/width, w = 1/height
    vec4 flowLaid;    // x0, y0, x1, y1 full-res Flow scissor rect
    vec4 flowMeta;    // x = half-res scale
} frame;

layout(location = 0) out vec2 v_uv;
layout(location = 1) out vec4 v_colorAlpha;
layout(location = 2) out vec4 v_params;    // x = kind, y = ageNorm, z = seed/arcDist, w = tempK
layout(location = 3) out vec4 v_lighting;  // x = sunDot, y = upDot, z = distM, w = softRangeM

const vec2 kCorners[6] = vec2[](
    vec2(-1.0, -1.0), vec2( 1.0, -1.0), vec2( 1.0,  1.0),
    vec2(-1.0, -1.0), vec2( 1.0,  1.0), vec2(-1.0,  1.0)
);

void main() {
    TrailQuad q = quads[gl_InstanceIndex];
    vec2 corner = kCorners[gl_VertexIndex];
    float radius = q.posAndRadius.w;
    bool isFrontPass = (q.colorAlpha.a < 0.0);
    float absAlpha = abs(q.colorAlpha.a);
    if (radius <= 0.0 || absAlpha <= 1e-4) {
        gl_Position = vec4(0.0, 0.0, -2.0, 1.0);
        v_uv = vec2(0.0);
        v_colorAlpha = vec4(0.0);
        v_params = vec4(0.0);
        v_lighting = vec4(0.0);
        return;
    }

    vec3 center = q.posAndRadius.xyz;
    float centerDist = max(length(center), 1e-3);
    vec3 toEye = -center / centerDist;
    vec3 right = frame.eyeRight.xyz;
    vec3 up = frame.eyeUp.xyz;
    vec3 zenith = normalize(frame.eyeMeta.xyz);
    float kind = q.params.x;

    vec3 offset;
    float softRange = max(radius * 0.70, 0.45);
    if (kind > 0.5 && kind < 1.5) {
        // Kind 1 (SparkEmber): razor-thin glowing streak aligned along screen-projected velocity.
        vec3 vel = q.velAndTemp.xyz;
        vec3 projVel = vel - toEye * dot(vel, toEye);
        float pLen = length(projVel);
        vec3 dir = pLen > 1e-3 ? (projVel / pLen) : up;
        vec3 perp = normalize(cross(toEye, dir));
        float stretch = max(q.params.w, 2.5);
        offset = dir * (corner.x * radius * stretch) + perp * (corner.y * radius);
        softRange = 0.25;
    } else if (kind > 1.5 && kind < 2.5) {
        // Kind 2 (Grounded Hydrodynamic Ocean Water Curtain): strictly locked upright along local zenith!
        // Cylindrical Y-axis billboard — physically IMPOSSIBLE to ever flip 180 deg or tumble when falling!
        vec3 axisUp = zenith;
        vec3 sideRaw = cross(toEye, axisUp);
        vec3 axisSide = length(sideRaw) > 1e-3 ? normalize(sideRaw) : right;
        float halfH = radius * max(q.params.w, 0.9);
        offset = axisSide * (corner.x * radius) + axisUp * (corner.y * halfH);
        softRange = max(radius * 0.75, 1.8);
    } else if ((kind > 3.5 && kind < 4.5) || kind > 5.5) {
        // Kind 4 (Ocean FoamWake) or Kind 6 (Terrain ScorchMark): flat surface-aligned quad perpendicular to zenith!
        vec3 align = q.velAndTemp.xyz;
        vec3 horiz = align - zenith * dot(align, zenith);
        vec3 fwd = length(horiz) > 1e-3 ? normalize(horiz) : normalize(cross(zenith, abs(zenith.z) < 0.9 ? vec3(0, 0, 1) : vec3(1, 0, 0)));
        vec3 side = normalize(cross(zenith, fwd));
        if (kind > 5.5) {
            // ScorchMark: elongated skid furrow or irregular blast crater floor
            offset = fwd * (corner.x * radius * max(q.params.w, 1.0)) + side * (corner.y * radius);
            softRange = 1.6;
        } else {
            // FoamWake: randomly rotated circular/irregular seafoam splash ring (NEVER a straight arrow ribbon!)
            float ang = q.params.z * 6.2831853;
            float ca = cos(ang), sa = sin(ang);
            vec2 rot = vec2(corner.x * ca - corner.y * sa, corner.x * sa + corner.y * ca);
            offset = fwd * (rot.x * radius * max(q.params.w, 1.0)) + side * (rot.y * radius);
            softRange = 2.0;
        }
    } else {
        // Kind 0 (Billowing Smoke Puff) / Kind 3 (FlameCore) / Kind 5 (DustPlume):
        // freely tumbling camera-facing volumetric billow (ZERO rectangular stretched pipes or stacked bricks!).
        float spin = (kind < 0.5) ? q.params.w : 0.45;
        float ang = q.params.z * 6.2831853 + q.params.y * spin;
        float ca = cos(ang), sa = sin(ang);
        vec2 rot = vec2(corner.x * ca - corner.y * sa, corner.x * sa + corner.y * ca);
        offset = right * (rot.x * radius) + up * (rot.y * radius);
        softRange = (kind > 2.5 && kind < 3.5) ? 0.35 : max(radius * 0.85, 1.0);
    }

    vec3 posRel = center + offset;
    v_uv = corner;
    v_colorAlpha = vec4(q.colorAlpha.rgb, absAlpha);
    // Kind 2 (water curtain) passes its vertical aspect ratio in v_params.w so fragment noise stays isotropic (zero vertical hair streaks!).
    float paramW = (kind > 1.5 && kind < 2.5) ? max(q.params.w, 1.0) : q.velAndTemp.w;
    v_params = vec4(kind, q.params.y, q.params.z, paramW);
    vec3 sun = normalize(frame.sunDirLight.xyz);
    vec3 sphereN = normalize(offset + toEye * max(radius * 0.85, 0.1));
    v_lighting = vec4(0.5 + 0.5 * dot(sphereN, sun), 0.5 + 0.5 * dot(sphereN, zenith), centerDist, isFrontPass ? -softRange : softRange);
    gl_Position = pc.projection * pc.modelView * vec4(posRel, 1.0);
}
