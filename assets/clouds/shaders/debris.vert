#version 450

// Instanced 6-DOF tumbling aerospace debris chunks (VIS-M11).
// Vertex attributes come from procedural faceted shard meshes; per-instance
// eye-relative pose, quaternion orientation, scale, temperature, and charring
// are supplied by the dynamic DebrisField SSBO.

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelView;
} pc;

struct DebrisInstance {
    vec4 posAndTemp;     // xyz = eye-relative ECEF (m), w = temperature (K)
    vec4 quat;           // unit quaternion body->ECEF (x, y, z, w)
    vec4 scaleAndScorch; // xyz = half-extents (m), w = scorch (0..1)
    vec4 colorAndArch;   // rgb = base metal albedo, w = archetype id
};

layout(std430, set = 0, binding = 0) readonly buffer Instances {
    DebrisInstance instances[];
};

layout(std140, set = 0, binding = 1) uniform Frame {
    vec4 sunDirLight; // xyz = sun dir ECEF, w = daylight (0..1)
    vec4 eyeRight;
    vec4 eyeUp;
    vec4 eyeMeta;     // xyz = eye up ECEF, w = simTime
} frame;

layout(location = 0) in vec3 inPos;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inMeta; // x = fracture edge glow (0..1), y = interior/rib tone (0..1)

layout(location = 0) out vec3 v_posRelEye;
layout(location = 1) out vec3 v_normalEcef;
layout(location = 2) out vec3 v_baseColor;
layout(location = 3) out vec4 v_params; // x = tempK, y = scorch, z = edgeGlow, w = tone

vec3 quatRotate(vec4 q, vec3 v) {
    vec3 t = 2.0 * cross(q.xyz, v);
    return v + q.w * t + cross(q.xyz, t);
}

void main() {
    DebrisInstance inst = instances[gl_InstanceIndex];
    vec3 localPos = inPos * inst.scaleAndScorch.xyz;
    vec3 worldOffset = quatRotate(inst.quat, localPos);
    vec3 posRel = inst.posAndTemp.xyz + worldOffset;
    // Correct normal for non-uniform half-extents scaling before rotating.
    vec3 invScale = 1.0 / max(inst.scaleAndScorch.xyz, vec3(1e-3));
    vec3 nLocal = normalize(inNormal * invScale);
    vec3 nWorld = normalize(quatRotate(inst.quat, nLocal));

    v_posRelEye = posRel;
    v_normalEcef = nWorld;
    v_baseColor = inst.colorAndArch.rgb;
    v_params = vec4(inst.posAndTemp.w, inst.scaleAndScorch.w, inMeta.x, inMeta.y);
    gl_Position = pc.projection * pc.modelView * vec4(posRel, 1.0);
}
