#version 330

in vec4 position;
out vec2 texCoords;

void main() {
    gl_Position = vec4(position.xy, 0, 1);
    texCoords = position.zw;
}