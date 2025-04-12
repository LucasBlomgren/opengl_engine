#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;
layout (location = 2) in vec3 aNormal;
layout (location = 3) in vec2 aTexCoord;

uniform vec3 uColor; // Add the uniform color
uniform bool useUniformColor; // Add a uniform to decide which color to use
uniform bool isContactPoint; 
uniform vec3 contactPointOffset;

uniform bool isLine;
uniform vec3 lineStart;
uniform vec3 lineEnd;

out vec2 TexCoord;
out vec3 ourColor;

out vec3 FragPos;
out vec3 Normal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {

    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;  

	if (isContactPoint) 
	{
		vec3 transformedPos = aPos + contactPointOffset;
		gl_Position = projection * view * vec4(transformedPos, 1.0f);
	} 
	else if (isLine) 
	{
		// Använda den korrekta punkten för varje vertex
		if (gl_VertexID == 0) 
			gl_Position = projection * view * vec4(lineStart, 1.0); // Första punkten i linjen
		else
			gl_Position = projection * view * vec4(lineEnd, 1.0);   // Andra punkten i linjen
	}
	else
	{
		gl_Position = projection * view * model * vec4(aPos, 1.0f);
	}

	ourColor = useUniformColor ? uColor : aColor;
	TexCoord = vec2(aTexCoord.x, aTexCoord.y);
}