#version 330 core


layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;
out vec3 ourColor;
out vec3 FragPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

uniform bool useUniformColor;
uniform vec3 uColor; 
uniform int objectType; 
uniform vec3 contactPointOffset;

uniform vec3 lineStart;
uniform vec3 lineEnd;

void main() 
{
    FragPos = vec3(model * vec4(aPos, 1.0));

	// regular object
	if (objectType == 0) {
		gl_Position = projection * view * vec4(aPos, 1.0f);
	}

	// contact point
	if (objectType == 1) {
		vec3 transformedPos = aPos + contactPointOffset;
		gl_Position = projection * view * vec4(transformedPos, 1.0f);
	} 

	// line 
	else if (objectType == 2) {
		if (gl_VertexID == 0) 
			gl_Position = projection * view * vec4(lineStart, 1.0);
		else
			gl_Position = projection * view * vec4(lineEnd, 1.0);
	}

	ourColor = useUniformColor ? uColor : aColor;
}