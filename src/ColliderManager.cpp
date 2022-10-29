#include<json/json.h>
#include"Mesh.h"
#include "glm/gtx/string_cast.hpp"
struct Collider {
	virtual glm::vec3 FindFurthestPoint(glm::vec3 direction) const = 0;
};
struct MeshCollider : Collider {
	public:
		std::vector<glm::vec3> object_veritces;
		// This method will be different for every mesh
		glm::vec3 FindFurthestPoint(glm::vec3 direction) const override {
			glm::vec3 maxPoint ;
			float maxDistance = -FLT_MAX;

			for (glm::vec3 vertex : object_veritces) {
				float distance = glm::dot(vertex, direction);
				if (distance > maxDistance) {
					maxDistance = distance;
					maxPoint = vertex;
				}
			}

			return maxPoint;
		}
};
// Takes one point from the first collider and take a second point from the other collider to try to create a simplex
glm::vec3 SupportMethod(const Collider* colA,const Collider* colB, glm::vec3 direction) {
	return colA->FindFurthestPoint(direction) - colB->FindFurthestPoint(-direction);
}

struct simplex {
private:
	std::array<glm::vec3, 4> simplex_points;
	unsigned simplex_size;
public:
	// Initialize a basic simplex
	simplex()
		: simplex_points({ glm::vec3(420,69,-420) })
		, simplex_size(0)
	{}


	simplex& operator=(std::initializer_list<glm::vec3> list) {
		for (auto v = list.begin(); v != list.end(); v++) {
			simplex_points[std::distance(list.begin(), v)] = *v;
		}
		simplex_size = list.size();

		return *this;
	}
	unsigned thisSize() {
		return simplex_size;
	}

	void push_front(glm::vec3 point) {
		simplex_points = { point, simplex_points[0], simplex_points[1], simplex_points[2] };
		simplex_size = std::min(simplex_size + 1, 4u);
	}

	glm::vec3& operator[] (unsigned i) { return simplex_points[i]; }
	unsigned size() const { return simplex_size; };

	auto begin() const { return simplex_points.begin(); }
	auto end()	 const { return simplex_points.end() - (4 - simplex_size); }
};
class ColliderManager {
public:
	static bool GJK(const Collider* colA, const Collider* colB)
	{
		// first simplex point
		glm::vec3 support = SupportMethod(colA, colB, glm::vec3(-1, 0, 0));

		simplex points;
		points.push_front(support);

		// new direction faces the origin
		glm::vec3 direction = -support;

		while (true) {
			// all points we'll be declared "support"
			support = SupportMethod(colA, colB, direction);

			if (glm::dot(support, direction) <= 0) {
				// collision failed
				return false;
			}

			points.push_front(support);
			// this will declare if this collision is true
			if (NextSimplex(points, direction)) {
				return true;
			}
		}
	}

	static bool NextSimplex(simplex& points, glm::vec3& direction) {
		switch (points.size()) {
		case 2: return Line(points, direction);
		case 3: return Triangle(points, direction);
		case 4: return Tetrahedron(points, direction);
		}

		// this point will never be reached
		return false;
	}
	static bool IsSameDirection(const glm::vec3& direction, const glm::vec3& ao)
	{
		return glm::dot(direction, ao) > 0;
	}
	static glm::vec3 TripleCross(glm::vec3 a, glm::vec3 b, glm::vec3 c) {
		return glm::cross(glm::cross(a, b), c);
	}
	static bool Line(simplex& points, glm::vec3& direction) {
		glm::vec3 a = points[0];
		glm::vec3 b = points[1];

		glm::vec3 ab = b - a;
		glm::vec3 ao = -a;

		if (IsSameDirection(ab, ao)) {
			direction = TripleCross(ab, ao, ab);
		}
		else {
			points = { a };
			direction = ao;
		}

		return false;
	}
	static bool Triangle(simplex& points, glm::vec3& direction) {
		glm::vec3 a = points[0];
		glm::vec3 b = points[1];
		glm::vec3 c = points[2];

		glm::vec3 ab = b - a;
		glm::vec3 ac = c - a;
		glm::vec3 ao = -a;

		glm::vec3 abc = glm::cross(ab, ac);

		if (IsSameDirection(glm::cross(abc, ac), ao)) {
			if (IsSameDirection(ab, ao)) {
				points = { a, c };
				direction = TripleCross(ab, ao, ab);
			}
			else {
				return Line(points = { a,b }, direction);
			}
		}
		else {
			if (IsSameDirection(glm::cross(ab, abc), ao)) {
				return Line(points = { a, b }, direction);
			}
			else {
				if (IsSameDirection(abc, ao)) {
					direction = abc;
				}
				else {
					points = { a, c, b };
					direction = -abc;
				}
			}
		}
		return false;
	}
	static bool Tetrahedron(simplex& points, glm::vec3& direction)
	{
		glm::vec3 a = points[0];
		glm::vec3 b = points[1];
		glm::vec3 c = points[2];
		glm::vec3 d = points[3];

		glm::vec3 ab = b - a;
		glm::vec3 ac = c - a;
		glm::vec3 ad = d - a;
		glm::vec3 ao = -a;

		glm::vec3 abc = glm::cross(ab, ac);
		glm::vec3 acd = glm::cross(ac, ad);
		glm::vec3 adb = glm::cross(ad, ab);

		if (IsSameDirection(abc, ao)) {
			return Triangle(points = { a, b, c }, direction);
		}

		if (IsSameDirection(acd, ao)) {
			return Triangle(points = { a, c, d }, direction);
		}

		if (IsSameDirection(adb, ao)) {
			return Triangle(points = { a, d, b }, direction);
		}

		return true;
	}

};
