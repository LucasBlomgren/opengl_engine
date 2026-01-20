#pragma once

class ShaderManager {
public:
	ShaderManager() {
		shaders["default"]					= Shader("src/graphics/shaders/default.vert", "src/graphics/shaders/default.frag");
		shaders["default_instanced"]		= Shader("src/graphics/shaders/default_instanced.vert", "src/graphics/shaders/default.frag");
		shaders["default"].instancedVariant = &shaders["default_instanced"];

		shaders["shadow"]					= Shader("src/graphics/shaders/shadow.vert", "src/graphics/shaders/shadow.frag");
		shaders["shadow_instanced"]			= Shader("src/graphics/shaders/shadow_instanced.vert", "src/graphics/shaders/shadow.frag");
		shaders["shadow"].instancedVariant	= &shaders["shadow_instanced"];

		shaders["debug"]					= Shader("src/graphics/shaders/debug.vert", "src/graphics/shaders/debug.frag");
		shaders["skybox"]					= Shader("src/graphics/shaders/skybox.vert", "src/graphics/shaders/skybox.frag");

		shaders["SkyShader"]				= Shader("src/graphics/shaders/SkyShader.vert", "src/graphics/shaders/SkyShader.frag");
	};

	Shader* getShader(const std::string& name);

private:
	static std::unordered_map<std::string, Shader> shaders;
};