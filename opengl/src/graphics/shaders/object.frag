#version 330 core

out vec4 FragColor;

in vec3 ourColor;
in vec3 ourNormal;
in vec2 TexCoord;
in vec4 FragPosLightSpace;

in vec3 Normal;  
in vec3 FragPos;  

uniform bool useTexture;
uniform sampler2D texture1;
uniform vec3 viewPos;

// ---- lighting ----
struct DirLight {
	 vec3 direction;

	 vec3 ambient;
	 vec3 diffuse;
	 vec3 specular;
};

struct PointLight {
	 vec3 position;
	 vec3 color;
	 
	 float constant;
	 float linear;
	 float quadratic;
	
	 vec3 ambient;
	 vec3 diffuse;
	 vec3 specular;
};

#define MAX_POINT_LIGHTS 10
uniform int numPointLights;

uniform DirLight dirLight;
uniform PointLight pointLights[MAX_POINT_LIGHTS];

vec3 CalcDirLight(DirLight light, vec3 normal, vec3 viewDir, vec4 FragPosLightSpace);
vec3 CalcPointLight(PointLight light, vec3 normal, vec3 fragPos, vec3 viewDir);
float ShadowCalculation(vec4 fragPosLightSpace, vec3 normal, vec3 lightDir);

uniform sampler2DShadow shadowMap;

void main() 
{
	 // properties
	 vec3 norm = normalize(Normal);
	 vec3 viewDir = normalize(viewPos - FragPos);
	 
	 // phase 1: directional lighting
	 vec3 result = CalcDirLight(dirLight, norm, viewDir, FragPosLightSpace);

	 // phase 2: point lights
	 for (int i = 0; i < numPointLights; ++i)
		  result += CalcPointLight(pointLights[i], norm, FragPos, viewDir);

	 if (useTexture) {
		  FragColor = texture(texture1, TexCoord) * vec4(result, 1.0);
	 }
	 else {
		  FragColor = vec4(ourColor * result, 1.0);
	 }
}

// calculates the color when using a directional light.
vec3 CalcDirLight(DirLight light, vec3 normal, vec3 viewDir, vec4 FragPosLightSpace)
{
	 vec3 lightDir = normalize(-light.direction);
	 float diff = max(dot(normal, lightDir), 0.0);
	 vec3 reflectDir = reflect(-lightDir, normal);
	 float spec = pow(max(dot(viewDir, reflectDir), 0.0), 16.0); // hårdkodad shininess

	 vec3 ambient  = light.ambient;
	 vec3 diffuse  = light.diffuse * diff;
	 vec3 specular = light.specular * spec;

	 float shadow = ShadowCalculation(FragPosLightSpace, normal, lightDir);     

	 return ambient + shadow * (diffuse + specular);
}

// calculates the color when using a point light.
vec3 CalcPointLight(PointLight light, vec3 normal, vec3 fragPos, vec3 viewDir)
{
	 vec3 lightDir = normalize(light.position - fragPos);
	 float diff = max(dot(normal, lightDir), 0.0);
	 vec3 reflectDir = reflect(-lightDir, normal);
	 float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);

	 float distance = length(light.position - fragPos);
	 float attenuation = 1.0 / (light.constant + light.linear * distance + light.quadratic * (distance * distance));

	 vec3 ambient  = light.ambient;
	 vec3 diffuse  = light.diffuse  * diff;
	 vec3 specular = light.specular * spec;

	 ambient  *= attenuation;
	 diffuse  *= attenuation;
	 specular *= attenuation;

	 return (ambient + diffuse + specular) * light.color;
}

float ShadowCalculation(vec4 fragPosLightSpace, vec3 normal, vec3 lightDir)
{
    // perform perspective divide
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    // transform to [0,1] range
    projCoords = projCoords * 0.5 + 0.5;
    float currentDepth = projCoords.z;

    float shadow = texture(shadowMap, projCoords);
    return shadow; // eller shadow beroende på hur du vill tolka: 1.0 = i skugga
}  