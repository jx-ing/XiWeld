#pragma once

#include <array>
#include <vector>

namespace coll
{
	class CollisionObject;

	/// \brief 多边行碰撞对象的 碰撞检测函数. return 是否碰撞.
	bool arePolygonsColliding(const CollisionObject& obj1, const CollisionObject& obj2);


	// 三维向量结构体
	struct Vector3d {
		double x, y, z;

		Vector3d(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
		Vector3d(float x, float y, float z) : x(double(x)), y(double(y)), z(double(z)) {}

		// 向量减法
		Vector3d operator-(const Vector3d& other) const {
			return Vector3d(x - other.x, y - other.y, z - other.z);
		}

		// 向量叉积
		Vector3d cross(const Vector3d& other) const {
			return Vector3d(
				y * other.z - z * other.y,
				z * other.x - x * other.z,
				x * other.y - y * other.x
			);
		}

		// 向量点积
		double dot(const Vector3d& other) const {
			return x * other.x + y * other.y + z * other.z;
		}
	};

	// 三角形结构体
	struct Triangle {
		Vector3d v0, v1, v2;

		Triangle(Vector3d v0, Vector3d v1, Vector3d v2) : v0(v0), v1(v1), v2(v2) {}

		// 计算三角形法向量
		Vector3d normal() const {
			Vector3d edge1 = v1 - v0;
			Vector3d edge2 = v2 - v0;
			return edge1.cross(edge2);
		}
	};

	// 三维点结构体
	using Point3D = Vector3d;

	// 包围盒结构体
	struct BoundingBox {
		double minX, maxX;
		double minY, maxY;
		double minZ, maxZ;
	};


	/**
	 * \brief 碰撞对象.
	 */
	class CollisionObject
	{
	private:
		std::vector<Point3D> points;
		BoundingBox bounds;
		std::vector<std::vector<Point3D>> triangles;

	public:

		/**
		 * \brief 使用多边形顶点构建碰撞对象.
		 *
		 * \param inputPoints 在同一平面的至少点的数组.
		 * \note 确保 inputPoints 顺序相连的多边行不自交.
		 */
		CollisionObject(const std::vector<Point3D>& inputPoints) : points(inputPoints) {
			preprocess();
		}
		CollisionObject(const std::vector<std::array<double, 3U>>& inputPoints) {
			for (const auto& p : inputPoints) {
				points.push_back({ p[0], p[1], p[2] });
			}
			preprocess();
		}

		void preprocess();

		const BoundingBox& getBounds() const {
			return bounds;
		}

		const std::vector<std::vector<Point3D>>& getTriangles() const {
			return triangles;
		}

		const std::vector<Point3D>& getPoints() const {
			return points;
		}
	};

}