#version 450

// VIS-M14 3-D Depth-Tested Volumetric Tactical & EMSO Overlay Vertex Shader.
// Pulls packed eye-relative ECEF vertices from a dynamic SSBO so coordinates keep
// full float32 precision anywhere around the WGS-84 globe and never jitter.

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelView;
} pc;

struct TacticalVertex {
    vec4 posR;   // xyz = eye-relative ECEF (m), w = radial / longitudinal coord rNorm in [0, 1]
    vec4 normU;  // xyz = outward surface normal (ECEF), w = lateral / angular coord u in [-1, 1]
    vec4 color;  // rgb = linear tactical color, a = base opacity / intensity
    vec4 params; // x = style (0..7), y = aux1, z = aux2, w = scaleM / extra
};

layout(std430, set = 0, binding = 0) readonly buffer Vertices {
    TacticalVertex verts[];
};

layout(location = 0) out vec4 v_posR;
layout(location = 1) out vec4 v_normU;
layout(location = 2) out vec4 v_color;
layout(location = 3) out vec4 v_params;

void main() {
    TacticalVertex v = verts[gl_VertexIndex];
    v_posR = v.posR;
    v_normU = v.normU;
    v_color = v.color;
    v_params = v.params;
    gl_Position = pc.projection * pc.modelView * vec4(v.posR.xyz, 1.0);
}
