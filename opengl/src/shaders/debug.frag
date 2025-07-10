#version 330 core

out vec4 FragColor;
in vec3 ourColor;
in vec3 FragPos;

uniform float minHeight;
uniform float maxHeight;

void main() 
{
    vec3 color = ourColor;
    FragColor = vec4(color, 1.0);
}