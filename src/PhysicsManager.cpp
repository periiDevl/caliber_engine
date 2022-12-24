#include "PhysicsManager.h"

#pragma region ObjectData

struct GameObject {
};
struct Component {
	bool IsActive;
};
struct Collider : Component {
	glm::vec3 Offset;
	float Radius;

	std::vector<glm::vec3> object_veritces;

	glm::vec3 FindFurthestPoint(glm::vec3 position, glm::vec3 direction) {
		glm::vec3 maxPoint;
		if (Radius != 0) {
			maxPoint = ((position + Offset) + (direction * Radius));
			return maxPoint;
		}
		else {

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
	}

};
struct CollisionObject {
public:
	Collider* collider;
	bool isTrigger;
	bool isStatic;
	bool isDynamic;
};
struct Rigidbody : Component {
	float Mass;

	glm::vec3 Position;
	glm::vec3 Velocity;
	glm::vec3 Force;

	bool IsTrigger;
	bool IsStatic;
	float Restitution;
	Collider* Collider;
};
#pragma endregion ObjectData

// COLLIDER MANAGER
#pragma region ColliderManager

inline Rigidbody CreateSphereBody(glm::vec3 position, float radius, glm::vec3 offset) {
	Rigidbody body;
	body.Collider = new Collider();
	body.Collider->IsActive = true;
	body.Collider->Radius = radius;
	body.Collider->Offset = offset;

	body.IsActive = true;
	body.Mass = 10;
	body.Position = position;
	body.Velocity = glm::vec3(0, 0, 0);
	return body;
}

// Takes one point from the first collider and take a second point from the other collider to try to create a simplex
inline glm::vec3 SupportMethod(const Rigidbody* colA, const Rigidbody* colB, glm::vec3 direction) {
	return colA->Collider->FindFurthestPoint(colA->Position, direction) - colB->Collider->FindFurthestPoint(colB->Position, -direction);
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

struct CollisionPoints {
	Rigidbody* objA;
	Rigidbody* objB;
	glm::vec3 A; // Furthest point of A into B
	glm::vec3 B; // Furthest point of B into A
	glm::vec3 Normal; // B – A normalized
	float Depth;    // Length of B – A
};
class ColliderManager {
public:
	static std::pair<bool, simplex> GJK(const Rigidbody* colA, const Rigidbody* colB)
	{
		if (colA->Collider->Radius != 0 && colB->Collider->Radius != 0) {
			simplex points;
			points.push_front(glm::vec3(0, 0, 0));
			return { glm::distance(colA->Position, colB->Position) <= colA->Collider->Radius + colB->Collider->Radius, points };
		}

		// first simplex point
		glm::vec3 support = SupportMethod(colA, colB, glm::vec3(-1, 0, 0));
		simplex points;
		points.push_front(support);

		// new direction faces the origin
		glm::vec3 direction = -support;

		while (true) {
			// all points we'll be declared "support"
			support = SupportMethod(colA, colB, glm::normalize(direction));

			if (glm::dot(support, direction) <= 0) {
				// collision failed
				return { false, points };
			}
			points.push_front(support);
			// this will declare if this collision is true
			if (NextSimplex(points, direction)) {
				return { true, points };
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

	static CollisionPoints  EPA(const simplex& simplex, const Rigidbody* colliderA, const Rigidbody* colliderB) {
		if (colliderA->Collider->Radius != 0 && colliderB->Collider->Radius != 0) {
			CollisionPoints sphereVsphere;
			glm::vec3 pointA = colliderA->Collider->FindFurthestPoint(colliderA->Position, glm::normalize(colliderB->Position - colliderA->Position));
			glm::vec3 pointB = colliderB->Collider->FindFurthestPoint(colliderB->Position, glm::normalize(-(colliderB->Position - colliderA->Position)));
			sphereVsphere.Normal = -glm::normalize(pointA - pointB);
			sphereVsphere.Depth = glm::distance(pointA, pointB);

			sphereVsphere.objA = (Rigidbody*)colliderA;
			sphereVsphere.objB = (Rigidbody*)colliderB;

			return sphereVsphere;
		}

		std::vector<glm::vec3> polytope(simplex.begin(), simplex.end());
		std::vector<size_t> faces = {
			0, 1, 2,
			0, 3, 1,
			0, 2, 3,
			1, 3, 2
		};

		auto [normals, minFace] = GetFaceNormals(polytope, faces);

		glm::vec3 minNormal;
		float minDistance = FLT_MAX;

		while (minDistance == FLT_MAX) {
			minNormal = glm::vec3(normals[minFace].x, normals[minFace].y, normals[minFace].z);
			minDistance = normals[minFace].w;

			glm::vec3 support = SupportMethod(colliderA, colliderB, glm::normalize(minNormal));
			float sDistance = glm::dot(minNormal, support);
			if (abs(sDistance - minDistance) > 0.001f) {
				minDistance = FLT_MAX;

				std::vector<std::pair<size_t, size_t>> uniqueEdges;
				for (size_t i = 0; i < normals.size(); i++) {
					if (IsSameDirection(normals[i], support)) {
						size_t f = i * 3;

						AddIfUniqueEdge(uniqueEdges, faces, f, f + 1);
						AddIfUniqueEdge(uniqueEdges, faces, f + 1, f + 2);
						AddIfUniqueEdge(uniqueEdges, faces, f + 2, f);

						faces[f + 2] = faces.back(); faces.pop_back();
						faces[f + 1] = faces.back(); faces.pop_back();
						faces[f] = faces.back(); faces.pop_back();

						normals[i] = normals.back(); normals.pop_back();

						i--;
					}
				}
				std::vector<size_t> newFaces;
				for (auto [edgeIndex1, edgeIndex2] : uniqueEdges) {
					newFaces.push_back(edgeIndex1);
					newFaces.push_back(edgeIndex2);
					newFaces.push_back(polytope.size());
				}
				polytope.push_back(support);

				auto [newNormals, newMinFace] = GetFaceNormals(polytope, newFaces);
				float oldMinDistance = FLT_MAX;
				for (size_t i = 0; i < normals.size(); i++) {
					if (normals[i].w < oldMinDistance) {
						oldMinDistance = normals[i].w;
						minFace = i;
					}
				}
				if (newNormals[newMinFace].w < oldMinDistance) {
					minFace = newMinFace + normals.size();
				}

				faces.insert(faces.end(), newFaces.begin(), newFaces.end());
				normals.insert(normals.end(), newNormals.begin(), newNormals.end());
			}
		}
		CollisionPoints points;

		points.Normal = minNormal;
		points.Depth = minDistance + 0.0001f;
		points.objA = (Rigidbody*)colliderA;
		points.objB = (Rigidbody*)colliderB;
		return points;
	}
	static std::pair<std::vector<glm::vec4>, size_t> GetFaceNormals(const std::vector<glm::vec3>& polytope, const std::vector<size_t>& faces)
	{
		std::vector<glm::vec4> normals;
		size_t minTriangle = 0;
		float  minDistance = FLT_MAX;

		for (size_t i = 0; i < faces.size(); i += 3) {
			glm::vec3 a = polytope[faces[i]];
			glm::vec3 b = polytope[faces[i + 1]];
			glm::vec3 c = polytope[faces[i + 2]];

			glm::vec3 normal = glm::normalize(glm::cross((b - a), (c - a)));
			float distance = glm::dot(normal, (a));

			if (distance < 0) {
				normal *= -1;
				distance *= -1;
			}

			normals.emplace_back(normal, distance);

			if (distance < minDistance) {
				minTriangle = i / 3;
				minDistance = distance;
			}
		}

		return { normals, minTriangle };
	}
	static void AddIfUniqueEdge(std::vector<std::pair<size_t, size_t>>& edges, const std::vector<size_t>& faces, size_t a, size_t b)
	{
		auto reverse = std::find(               //      0--<--3
			edges.begin(),                     //     / \ B /   A: 2-0
			edges.end(),                       //    / A \ /    B: 0-2
			std::make_pair(faces[b], faces[a]) //   1-->--2
		);

		if (reverse != edges.end()) {
			edges.erase(reverse);
		}

		else {
			edges.emplace_back(faces[a], faces[b]);
		}
	};
};

#pragma endregion ColliderManager

// BASIC OBJCETS

// PHYSICS

class Solver {
public:
	virtual void Solve(
		std::vector<CollisionPoints>& collisions,
		float dt) = 0;
};
class PositionSolver : Solver {
	void Solve(std::vector<CollisionPoints>& collisions, float dt) {
		for (CollisionPoints collision : collisions) {
			collision.objA->Position += collision.Depth * collision.Normal;
			collision.objB->Position -= collision.Depth * collision.Normal;
		}
	}
};
class PhysicalWorld {
private:
	std::list<Rigidbody*> Objects;
	std::list<Solver*> Solvers;
	glm::vec3 Gravity = glm::vec3(0, -1.35981f, 0);

public:
	void AddObject(Rigidbody* object) {
		Objects.push_back(object);
	}
	void RemoveObject(Rigidbody* object) {
		Objects.remove(object);
	}

	void AddSolver(Solver* solver) {
		Solvers.push_back(solver);
	}
	void RemoveSolver(Solver* solver) {
		Solvers.remove(solver);
	}
	void Step(float dt, GLFWwindow* window) {
		for (Rigidbody* obj : Objects) {
			glm::vec3 gravityF = Gravity * obj->Mass * dt;
			obj->Force += gravityF;
			glm::vec3 forceAdded = obj->Force / obj->Mass;
			obj->Velocity += forceAdded;

			glm::vec3 pos = obj->Position;

			if (!obj->IsStatic)
				pos += obj->Velocity * dt;

			obj->Position = pos;
			obj->Force = glm::vec3(0, 0, 0);
		}
		ResolveCollisions(dt, window);
	}


	void ResolveCollisions(float dt, GLFWwindow* window)
	{
		std::vector<CollisionPoints> collisions;
		for (Rigidbody* a : Objects) {
			if (!a->IsActive)
				continue;
			for (Rigidbody* b : Objects) {
				if (!b->IsActive)
					continue;
				if (a == b) break;

				if (!a->Collider || !b->Collider)
				{
					continue;
				}
				auto [value_, simplex_] = ColliderManager::GJK(a, b);

				if (value_) {
					CollisionPoints points = ColliderManager::EPA(simplex_, a, b);

					collisions.emplace_back(points);
				}
			}
		}



		for (CollisionPoints collision : collisions) {
			glm::vec3 relativeVelocity = collision.objB->Velocity - collision.objA->Velocity;

			float e = std::min(collision.objA->Restitution, collision.objB->Restitution);

			float j = glm::dot(relativeVelocity, collision.Normal);

			std::cout << "energy jouls:" << j << std::endl;

			collision.objA->Position += collision.Normal * collision.Depth / 2.0f;
			collision.objB->Position -= collision.Normal * collision.Depth / 2.0f;

			std::cout << "obj a velocity:" << glm::to_string(collision.objA->Velocity) << std::endl;
			std::cout << "obj b velocity:" << glm::to_string(collision.objB->Velocity) << std::endl;

			collision.objA->Velocity += j * collision.Normal;
			collision.objB->Velocity += -j * collision.Normal;
		}
	}
};
