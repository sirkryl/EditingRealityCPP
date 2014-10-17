#version 330

uniform sampler2D tex;
in vec2 texCoords;
out vec4 fragColor;
const vec4 color = vec4(1, 1, 1, 1);

void main() {
    fragColor = vec4(1, 1, 1, texture(tex, texCoords).r) * color;
}