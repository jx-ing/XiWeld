#include "stdafx.h"
#include "Apps/Welding/PolygonCollision.h"

#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>

namespace coll {

	// 计算三角形在轴上的投影范围
	void getProjection(const Triangle& tri, const Vector3d& axis, double& min, double& max) {
		double p0 = axis.dot(tri.v0);
		double p1 = axis.dot(tri.v1);
		double p2 = axis.dot(tri.v2);

		min = std::min(p0, std::min(p1, p2));
		max = std::max(p0, std::max(p1, p2));
	}

	// 检查两个范围是否重叠
	bool rangesOverlap(double min1, double max1, double min2, double max2) {
		return max1 >= min2 && max2 >= min1;
	}

	// 使用分离轴定理检测两个三角形是否碰撞
	bool trianglesCollide(const Triangle& tri1, const Triangle& tri2) {
		// 可能的分离轴
		Vector3d axes[17];

		// 三角形1的法线
		axes[0] = tri1.normal();

		// 三角形2的法线
		axes[1] = tri2.normal();

		// 两个三角形边
		Vector3d edge1[3] = { tri1.v1 - tri1.v0, tri1.v2 - tri1.v1, tri1.v0 - tri1.v2 };
		Vector3d edge2[3] = { tri2.v1 - tri2.v0, tri2.v2 - tri2.v1, tri2.v0 - tri2.v2 };
		
		// 两个三角形边的叉积
		int index = 2;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				axes[index++] = edge1[i].cross(edge2[j]);
			}
		}

		// 两个三角形边的法向
		for (int i = 0; i < 3; i++) {
			axes[index++] = edge1[i].cross(axes[0]);
		}
		for (int i = 0; i < 3; i++) {
			axes[index++] = edge2[i].cross(axes[1]);
		}

		size_t n = 11;

		// 检查所有可能的分离轴
		for (int i = 0; i < n; i++) {
			Vector3d axis = axes[i];

			// 跳过零向量
			if (axis.x == 0 && axis.y == 0 && axis.z == 0) {
				continue;
			}

			double min1, max1, min2, max2;
			getProjection(tri1, axis, min1, max1);
			getProjection(tri2, axis, min2, max2);

			// 两面平行判断
			if (i == 0) {
				if (fabs(min1 - max1) < 1e-7 && fabs(min2 - max2) < 1e-7 && fabs(min1 - min2) < 1e-7) {
					n = 17;
				}
			}

			if (!rangesOverlap(min1, max1, min2, max2)) {
				return false; // 找到分离轴，不碰撞
			}
		}

		return true; // 所有轴上都重叠，碰撞
	}

	// 计算包围盒
	BoundingBox calculateBoundingBox(const std::vector<Point3D>& points) {
		if (points.empty()) {
			return { 0, 0, 0, 0, 0, 0 };
		}

		// 初始化最小最大值
		double minX = points[0].x, maxX = points[0].x;
		double minY = points[0].y, maxY = points[0].y;
		double minZ = points[0].z, maxZ = points[0].z;

		// 遍历所有点，找出最小最大值
		for (const auto& point : points) {
			minX = std::min(minX, point.x);
			maxX = std::max(maxX, point.x);
			minY = std::min(minY, point.y);
			maxY = std::max(maxY, point.y);
			minZ = std::min(minZ, point.z);
			maxZ = std::max(maxZ, point.z);
		}

		return { minX, maxX, minY, maxY, minZ, maxZ };
	}

	// 判断两个包围盒是否碰撞
	bool isBoundingBoxCollision(const BoundingBox& box1, const BoundingBox& box2) {
		// 检查x轴方向是否有重叠
		bool xOverlap = (box1.minX <= box2.maxX) && (box1.maxX >= box2.minX);
		// 检查y轴方向是否有重叠
		bool yOverlap = (box1.minY <= box2.maxY) && (box1.maxY >= box2.minY);
		// 检查z轴方向是否有重叠
		bool zOverlap = (box1.minZ <= box2.maxZ) && (box1.maxZ >= box2.minZ);

		// 三个方向都有重叠才表示包围盒碰撞
		return xOverlap && yOverlap && zOverlap;
	}

	// 判断点是否在三角形内部
	bool isPointInTriangle(const Point3D& p, const Point3D& a, const Point3D& b, const Point3D& c) {
		// 计算向量
		Point3D v0 = { c.x - a.x, c.y - a.y, c.z - a.z };
		Point3D v1 = { b.x - a.x, b.y - a.y, b.z - a.z };
		Point3D v2 = { p.x - a.x, p.y - a.y, p.z - a.z };

		// 计算点积
		double dot00 = v0.x * v0.x + v0.y * v0.y + v0.z * v0.z;
		double dot01 = v0.x * v1.x + v0.y * v1.y + v0.z * v1.z;
		double dot02 = v0.x * v2.x + v0.y * v2.y + v0.z * v2.z;
		double dot11 = v1.x * v1.x + v1.y * v1.y + v1.z * v1.z;
		double dot12 = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;

		// 计算重心坐标
		double invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
		double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
		double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

		// 检查点是否在三角形内
		return (u >= 0) && (v >= 0) && (u + v < 1);
	}

	// 判断三角形是否是"耳朵"
	bool isEar(const Point3D& a, const Point3D& b, const Point3D& c, const std::vector<Point3D>& polygon) {
		// 检查三角形内部是否包含其他顶点
		for (const auto& p : polygon) {
			if (&p == &a || &p == &b || &p == &c) { continue; }
			if (isPointInTriangle(p, a, b, c)) {
				return false;
			}
		}
		return true;
	}

	// 将多边形划分为三角形
	std::vector<std::vector<Point3D>> triangulatePolygon(const std::vector<Point3D>& polygon) {
		std::vector<std::vector<Point3D>> triangles;
		if (polygon.size() < 3) return triangles;

		std::vector<Point3D> workingPolygon = polygon;

		while (workingPolygon.size() > 3) {
			bool earFound = false;

			for (size_t i = 0; i < workingPolygon.size(); ++i) {
				size_t prev = (i == 0) ? workingPolygon.size() - 1 : i - 1;
				size_t next = (i + 1) % workingPolygon.size();

				const Point3D& a = workingPolygon[prev];
				const Point3D& b = workingPolygon[i];
				const Point3D& c = workingPolygon[next];

				if (isEar(a, b, c, workingPolygon)) {
					// 找到耳朵，切割三角形
					triangles.push_back({ a, b, c });
					workingPolygon.erase(workingPolygon.begin() + i);
					earFound = true;
					break;
				}
			}

			if (!earFound) {
				// 如果没有找到耳朵，可能是凹多边形，需要特殊处理
				// 这里简单处理为取前三个点构成三角形
				triangles.push_back({ workingPolygon[0], workingPolygon[1], workingPolygon[2] });
				workingPolygon.erase(workingPolygon.begin(), workingPolygon.begin() + 2);
			}
		}

		// 添加最后一个三角形
		if (workingPolygon.size() == 3) {
			triangles.push_back({ workingPolygon[0], workingPolygon[1], workingPolygon[2] });
		}

		return triangles;
	}

	void CollisionObject::preprocess() {
		if (points.empty()) {
			return;
		}

		bounds = calculateBoundingBox(points);
		triangles = triangulatePolygon(points);
	}


	bool arePolygonsColliding(const CollisionObject& obj1, const CollisionObject& obj2) {
		if (!isBoundingBoxCollision(obj1.getBounds(), obj2.getBounds())) {
			return false;
		}

		const auto& triangles1 = obj1.getTriangles();
		const auto& triangles2 = obj2.getTriangles();

		for (const auto& tri1 : triangles1) {
			for (const auto& tri2 : triangles2) {
				Triangle t1(Vector3d(tri1[0].x, tri1[0].y, tri1[0].z), Vector3d(tri1[1].x, tri1[1].y, tri1[1].z), Vector3d(tri1[2].x, tri1[2].y, tri1[2].z));
				Triangle t2(Vector3d(tri2[0].x, tri2[0].y, tri2[0].z), Vector3d(tri2[1].x, tri2[1].y, tri2[1].z), Vector3d(tri2[2].x, tri2[2].y, tri2[2].z));
				if (trianglesCollide(t1, t2)) {
					return true;
				}
			}
		}
		return false;
	}
}