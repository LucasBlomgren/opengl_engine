#version 330 core

out vec4 FragColor;

in vec3 ourColor;
in vec3 ourNormal;
in vec2 TexCoord;

in vec3 Normal;  
in vec3 FragPos;  

uniform bool useTexture;
uniform sampler2D texture1;

uniform vec3 lightPos; 
uniform vec3 viewPos; 
uniform vec3 lightColor;

void main() {

    // ambient
    float ambientStrength = 0.0000000000000000000000000001;
    vec3 ambient = vec3(ambientStrength, ambientStrength, ambientStrength);

    //float ambientStrength = 0.3;
    //vec3 ambient = ambientStrength * lightColor;
  	
    // diffuse 
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;
    
    // specular
    float specularStrength = 0.5;
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);  
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    vec3 specular = specularStrength * spec * lightColor;  

    // attenuation
    float lightConstant  = 1.0;
    float lightLinear    = 0.0145;
    float lightQuadratic = 0.00275;

    float distance    = length(lightPos - FragPos);
    float attenuation = 1.0 / (lightConstant + lightLinear * distance + lightQuadratic * (distance * distance));  
    //attenuation = max(attenuation, 0.05);
    ambient  *= attenuation; 
    diffuse  *= attenuation;
    specular *= attenuation;   
        
    vec3 result = (ambient + diffuse + specular) * ourColor;

    result = ourColor;

    if (useTexture) {
        FragColor = texture(texture1, TexCoord) * vec4(result, 1.0);
    }
    else {
        FragColor = vec4(ourColor, 1.0);
    }
}