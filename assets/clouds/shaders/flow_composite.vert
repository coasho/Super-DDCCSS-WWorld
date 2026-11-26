// fsim: fullscreen triangle for depth-aware bilateral upsampling of NVIDIA Flow volumes (VIS-M12).
#version 450
out gl_PerVertex { vec4 gl_Position; };
void main() {
  vec2 p = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
  gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
