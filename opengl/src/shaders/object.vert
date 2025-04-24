#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;
layout (location = 2) in vec3 aNormal;
layout (location = 3) in vec2 aTexCoord;

out vec2 TexCoord;
out vec3 ourColor;
out vec3 FragPos;
out vec3 Normal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

uniform vec3 uColor; 
uniform bool useUniformColor; 

void main() 
{
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;  

	gl_Position = projection * view * model * vec4(aPos, 1.0f);

	ourColor = useUniformColor ? uColor : aColor;
	TexCoord = vec2(aTexCoord.x, aTexCoord.y);
}