#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;

// output to fragment shader
out vec2 vTexCoord;
out vec3 vColor;
out vec3 vWorldPos;
out vec3 vNormal;
out vec4 vPosLightSpace;

// uniforms
uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
uniform mat4 lightSpaceMatrix;

// color control
uniform vec3 uColor; 
uniform bool useRandomColor;

// simple hash function to generate pseudo-random numbers
float hash(float n) {
	return fract(sin(n) * 33249.5453123);
}

void main() 
{
	vWorldPos = vec3(model * vec4(aPos, 1.0));
	vNormal = mat3(transpose(inverse(model))) * aNormal;  
	vPosLightSpace = lightSpaceMatrix * vec4(vWorldPos, 1.0);
	gl_Position = projection * view * model * vec4(aPos, 1.0f);

	// Slå fram en färg baserat på gl_VertexID
	float id = float(gl_VertexID);
	vec3 randomColor = vec3(
		hash(id + 0.1),
		hash(id + 13.7),
		hash(id + 91.3)
	);

	// toggla mellan uniform-färg och random-färg:
	if (useRandomColor)
		vColor = randomColor;
	else
		vColor = uColor;

	vTexCoord = aTexCoord;
}