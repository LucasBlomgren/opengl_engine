#version 330 core


layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;
out vec3 ourColor;
out vec3 FragPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

struct debugSettings {
	int objectType; 

	bool useUniformColor;
	vec3 uColor;

	vec3 contactPointOffset;

	// line
	vec3 lineStart;
	vec3 lineEnd;

	// sphere
	vec3 uCenter;    
	vec3 uU;         
	vec3 uV;  
};

uniform debugSettings debug;

void main() 
{

	FragPos = vec3(model * vec4(aPos, 1.0));

	// default
	if (debug.objectType == 0) {
		gl_Position = projection * view * model * vec4(aPos, 1.0f);
	}

	// sphere
	if (debug.objectType == 1) {
		vec3 worldPos = debug.uCenter + aPos.x * debug.uU + aPos.y * debug.uV;
		gl_Position = projection * view * vec4(worldPos, 1.0);

	}

	// contact point
	if (debug.objectType == 2) {
		vec3 transformedPos = aPos + debug.contactPointOffset;
		gl_Position = projection * view * vec4(transformedPos, 1.0f);
	} 

	// line 
	else if (debug.objectType == 3) {
		if (gl_VertexID == 0) 
			gl_Position = projection * view * vec4(debug.lineStart, 1.0);
		else
			gl_Position = projection * view * vec4(debug.lineEnd, 1.0);
	}

	ourColor = debug.useUniformColor ? debug.uColor : aColor;
}