#version 330 core

layout (location = 0) in vec3 aPos; 
layout (location = 1) in vec3 aNormal; 
layout (location = 2) in vec2 aTexCoord; 
layout (location = 3) in mat4 instanceModel; 
layout (location = 7) in vec3 instanceColor;

// output to fragment shader
out vec2 vTexCoord;
out vec3 vColor;
out vec3 vWorldPos;
out vec3 vNormal;
out vec4 vPosLightSpace;

// uniforms
uniform mat4 projection;
uniform mat4 view;
uniform mat4 lightSpaceMatrix;

// main 
void main() 
{
	vWorldPos = vec3(instanceModel * vec4(aPos, 1.0));
	vNormal = mat3(transpose(inverse(instanceModel))) * aNormal;  
	vPosLightSpace = lightSpaceMatrix * vec4(vWorldPos, 1.0);
	
	gl_Position = projection * view * instanceModel * vec4(aPos, 1.0f);

	vColor = instanceColor;
	vTexCoord = aTexCoord;
}