#pragma once
const float epsilon = 1e-6;

std::pair<float, float> ProjectVertices(const std::array<glm::vec3, 8>& vertices, const glm::vec3& axis)
{
	float min = std::numeric_limits<float>::max();
	float max = std::numeric_limits<float>::lowest();

	for (int i = 0; i < vertices.size(); i++)
	{
		const glm::vec3& v = vertices[i];
		float proj = glm::dot(v, axis);

		if (proj < min) { min = proj; }
		if (proj > max) { max = proj; }
	}

	return std::make_pair(min, max);
}

bool IntersectPolygons(Mesh& objA, Mesh& objB, glm::vec3& normal, float& depth, int& collisionNormalOwner)
{
    OOBB& boxA = objA.OOBB;
    OOBB& boxB = objB.OOBB;

	const std::array<glm::vec3, 8>& verticesA = boxA.transformedVertices;
	const std::array<glm::vec3, 8>& verticesB = boxB.transformedVertices;
	const std::array<glm::vec3, 3>& normalsA = boxA.normals;
	const std::array<glm::vec3, 3>& normalsB = boxB.normals;

	for (const glm::vec3& faceNormal : normalsA)
	{
		auto [minA, maxA] = ProjectVertices(verticesA, faceNormal);
		auto [minB, maxB] = ProjectVertices(verticesB, faceNormal);

		if (minA >= maxB or minB >= maxA)
		{
			return false;
		}

		float axisDepth = std::min(maxB - minA, maxA - minB);

		if (axisDepth < depth)
		{
			depth = axisDepth;
			normal = faceNormal;

			collisionNormalOwner = 0;
		}
	}

	for (const glm::vec3& faceNormal : normalsB)
	{
		auto [minA, maxA] = ProjectVertices(verticesA, faceNormal);
		auto [minB, maxB] = ProjectVertices(verticesB, faceNormal);

		if (minA >= maxB or minB >= maxA)
		{
			return false;
		}

		float axisDepth = std::min(maxB - minA, maxA - minB);

		if (axisDepth < depth)
		{
			depth = axisDepth;
			normal = faceNormal;

			collisionNormalOwner = 1;
		}
	}

	for (const glm::vec3& faceNormalA : normalsA)
		for (const glm::vec3& faceNormalB : normalsB)
		{
			glm::vec3 axis = glm::cross(faceNormalA, faceNormalB);
			if (glm::length2(axis) < epsilon)
			{
				continue; // Skip degenerate axis
			}
			axis = glm::normalize(axis);

			auto [minA, maxA] = ProjectVertices(verticesA, axis);
			auto [minB, maxB] = ProjectVertices(verticesB, axis);

			if (minA >= maxB or minB >= maxA)
			{
				return false;
			}				

			float axisDepth = std::min(maxB - minA, maxA - minB);

			if (axisDepth < depth)
			{
				depth = axisDepth;
				normal = axis;

				collisionNormalOwner = 0;
			}
		}

	return true;
}