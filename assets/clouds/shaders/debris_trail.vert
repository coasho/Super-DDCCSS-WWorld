#version 450

// Instanced soft billboards for glowing incandescent spark streaks, dark smoke
// ribbons trailing behind burning debris chunks, and hydrodynamic water splash
// plumes (VIS-M11). Uses premultiplied alpha output in a single draw call.

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelView;
} pc;

struct TrailQuad {
    vec4 posAndRadius;  // xyz = eye-relative ECEF (m), w = radius (m)
    vec4 velAndTemp;    // xyz = velocity/streak dir ECEF (m/s), w = temperature (K)
    vec4 colorAlpha;    // rgb = base albedo/tint, a = opacity
    vec4 params;        // x = kind (0=smoke, 1=spark, 2=splash, 3=flame), y = ageNorm (0..1), z = seed, w = stretch
};

layout(std430, set = 0, binding = 0) readonly buffer Quads {
    TrailQuad quads[];
};

layout(std140, set = 0, binding = 1) uniform Frame {
    vec4 sunDirLight; // xyz = sun dir ECEF, w = daylight (0..1)
    vec4 eyeRight;    // xyz = camera right ECEF
    vec4 eyeUp;       // xyz = camera up ECEF
    vec4 eyeMeta;     // xyz = local zenith ECEF, w = simTime
} frame;

layout(location = 0) out vec2 v_uv;
layout(location = 1) out vec4 v_colorAlpha;
layout(location = 2) out vec4 v_params;
layout(location = 3) out vec2 v_lighting; // x = sunDot, y = distM

const vec2 kCorners[6] = vec2[](
    vec2(-1.0, -1.0), vec2( 1.0, -1.0), vec2( 1.0,  1.0),
    vec2(-1.0, -1.0), vec2( 1.0,  1.0), vec2(-1.0,  1.0)
);

void main() {
    TrailQuad q = quads[gl_InstanceIndex];
    vec2 corner = kCorners[gl_VertexIndex];
    float radius = q.posAndRadius.w;
    if (radius <= 0.0 || q.colorAlpha.a <= 1e-4) {
        gl_Position = vec4(0.0, 0.0, -2.0, 1.0);
        v_uv = vec2(0.0);
        v_colorAlpha = vec4(0.0);
        v_params = vec4(0.0);
        v_lighting = vec2(0.0);
        return;
    }

    vec3 center = q.posAndRadius.xyz;
    vec3 toEye = normalize(-center);
    vec3 right = frame.eyeRight.xyz;
    vec3 up = frame.eyeUp.xyz;
    float kind = q.params.x;

    vec3 offset;
    if (kind > 0.5 && kind < 1.5) {
        // SparkEmber: align elongated hot streak along screen-projected velocity vector.
        vec3 vel = q.velAndTemp.xyz;
        vec3 projVel = vel - toEye * dot(vel, toEye);
        float pLen = length(projVel);
        vec3 dir = pLen > 1e-3 ? (projVel / pLen) : up;
        vec3 perp = normalize(cross(toEye, dir));
        float stretch = max(q.params.w, 1.5);
        offset = dir * (corner.x * radius * stretch) + perp * (corner.y * radius);
    } else if (kind < 0.5 && q.params.w > 1.05) {
        // Smoke ribbon puff: elongate young puffs along the debris trajectory so fast-falling
        // chunks leave a seamless continuous soot trail that relaxes into round billowing clouds.
        vec3 vel = q.velAndTemp.xyz;
        vec3 projVel = vel - toEye * dot(vel, toEye);
        float pLen = length(projVel);
        vec3 dir = pLen > 1e-3 ? (projVel / pLen) : right;
        vec3 perp = normalize(cross(toEye, dir));
        float stretch = mix(q.params.w, 1.0, smoothstep(0.0, 0.35, q.params.y));
        offset = dir * (corner.x * radius * stretch) + perp * (corner.y * radius);
    } else {
        // Splash / FlameCore / static smoke: rotated camera-facing soft volumetric puff.
        float ang = q.params.z * 6.2831853 + q.params.y * 0.6;
        float ca = cos(ang), sa = sin(ang);
        vec2 rot = vec2(corner.x * ca - corner.y * sa, corner.x * sa + corner.y * ca);
        offset = right * (rot.x * radius) + up * (rot.y * radius);
    }

    vec3 posRel = center + offset;
    v_uv = corner;
    v_colorAlpha = q.colorAlpha;
    v_params = vec4(kind, q.params.y, q.params.z, q.velAndTemp.w);
    vec3 sun = normalize(frame.sunDirLight.xyz);
    v_lighting = vec2(0.5 + 0.5 * dot(normalize(offset + toEye * radius), sun), length(center));
    gl_Position = pc.projection * pc.modelView * vec4(posRel, 1.0);
}
