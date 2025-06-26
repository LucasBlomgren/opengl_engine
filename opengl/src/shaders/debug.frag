#version 330 core

out vec4 FragColor;
in vec3 ourColor;
in vec3 FragPos;

uniform bool terrain;
uniform float minHeight;
uniform float maxHeight;

void main() 
{
    vec3 color = ourColor;

    if (terrain) {
        float t = (FragPos.y - minHeight) / (maxHeight - minHeight);
        t = clamp(t, 0.0, 1.0);
        color = vec3(0.0, t, 0.0);
    }

    FragColor = vec4(color, 1.0);
}