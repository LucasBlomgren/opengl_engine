#version 330 core

out vec4 FragColor;

uniform vec2 iResolution; 
const float FOV = 80.0; // Field of view in degrees
uniform vec3 SunVec3;
uniform mat4 ViewMatrix;


/*

	A fork of "Still non-accurate atmosphere by robobo1221" by kerbonaut
  link: https://www.shadertoy.com/view/llffzM
*/

const float pi = 3.14159265359;

const float zenithOffset = 0.0;
const float multiScatterPhase = 0.01;
const float density = 0.7;

const vec3 skyColor = vec3(0.39, 0.57, 1.0); //Make sure one of the conponents is never 0.0


float greatCircleDist(vec2 p, vec2 lp)
{
	float phi_1 = p.y;
	float phi_2 = lp.y;
	float delta_lambda = p.x-lp.x;
	return acos(sin(phi_1)*sin(phi_2) + cos(phi_1)*cos(phi_2)*cos(delta_lambda));
}

float  zenithDensity(float x)
{
	
	return density / pow(max(x - zenithOffset, 0.35e-2), 0.75);
}

vec3 getSkyAbsorption(vec3 x, float y){
	
	vec3 absorption = x * -y;
		 absorption = exp2(absorption) * 2.0;
	
	return absorption;
}

float getSunPoint(vec2 p, vec2 lp){
	float dist = greatCircleDist(p, lp)/pi*2.;
	return smoothstep(0.03, 0.026, dist) * 50.0;
}


float getRayleigMultiplier(vec2 p, vec2 lp)
{
	float dist = greatCircleDist(p, lp)/pi*5.;
	return 1.0 + pow(1.0 - clamp(dist, 0.0, 1.0), 2.0) * pi * 0.5;
}

float getMie(vec2 p, vec2 lp){
	float dist = greatCircleDist(p, lp)/pi*2.;
	float disk = clamp(1.0 - pow(dist, 0.1), 0.0, 1.0);
	
	return disk*disk*(3.0 - 2.0 * disk) * 2.0 * pi;
}

vec3 getAtmosphericScattering(vec2 p, vec2 lp)
{
	
	float zenith = zenithDensity(p.y);
	float sunPointDistMult =  clamp(length(max(lp.y + multiScatterPhase - zenithOffset, 0.0)), 0.0, 1.0);
	
	float rayleighMult = getRayleigMultiplier(p, lp);
	
	vec3 absorption = getSkyAbsorption(skyColor, zenith);
	vec3 sunAbsorption = getSkyAbsorption(skyColor, zenithDensity(lp.y + multiScatterPhase));
	vec3 sky = skyColor * zenith * rayleighMult;
	vec3 sun = getSunPoint(p, lp) * absorption;
	vec3 mie = getMie(p, lp) * sunAbsorption;
	
	vec3 totalSky = mix(sky * absorption, sky / (sky + 0.5), sunPointDistMult);
		 totalSky += sun + mie;
		 totalSky *= sunAbsorption * 0.5 + 0.5 * length(sunAbsorption);
	
	return totalSky;
}

vec3 jodieReinhardTonemap(vec3 c)
{
	float l = dot(c, vec3(0.2126, 0.7152, 0.0722));
	vec3 tc = c / (c + 1.0);

	return mix(c / (l + 1.0), tc, tc);
}

vec2 screen2world(vec2 pos)
{
	return (pos / iResolution.xy - .5) * vec2(2., 1.) * pi;
}


// get the world space ray for the pixel
vec3 GetPixelRay(vec2 uv) 
{
	vec2 ndc = (uv * 2.0 - 1.0); 
	mat4 invView = ViewMatrix;
	vec3 CameraRight = normalize(vec3(invView[0][0], invView[1][0], invView[2][0]));
	vec3 CameraUp = normalize(vec3(invView[0][1], invView[1][1], invView[2][1]));
	vec3 CameraForward = normalize(vec3(invView[0][2], invView[1][2], invView[2][2]));
	
	// Calculate the ray direction in world space
	float aspectRatio = (iResolution.x / iResolution.y);
	float tanFOV = tan(radians(FOV) * 0.5);
	vec3 rayDir = -CameraRight * ndc.x * tanFOV * aspectRatio + -CameraUp * ndc.y * tanFOV;

	return normalize(rayDir + CameraForward); // Add camera forward direction
}


void main() {
	vec2 uv = gl_FragCoord.xy / iResolution.xy;
	vec3 rayDir = GetPixelRay(uv);
	vec3 sunDir = normalize(SunVec3);

	// convert ray direction to 2D coordinates for the atmospheric scattering function
	vec2 rayDir2D = vec2(atan(rayDir.x, rayDir.z), asin(-rayDir.y));

	vec2 sunDir2D = vec2(atan(-sunDir.x, -sunDir.z), asin(sunDir.y) + 0.10);
	
	vec3 color = getAtmosphericScattering(rayDir2D, sunDir2D);
	color = jodieReinhardTonemap(color);
	color = pow(color, vec3(2.2)); //Back to linear
	
	FragColor = vec4(color, 1.0 );

}
