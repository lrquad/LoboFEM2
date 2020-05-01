#include "computeTriangle.h"

void Lobo::computeTriangleNorm(Eigen::Vector3d &v1, Eigen::Vector3d &v2, Eigen::Vector3d &v3, Eigen::Vector3d &normal)
{
	normal = (v2 - v1).cross(v3 - v1);
	normal.normalize();
}

double Lobo::computeTriangleArea(Eigen::Vector3d n0, Eigen::Vector3d n1, Eigen::Vector3d n2)
{
	n1 = n1 - n0;
	n2 = n2 - n0;

	n0 = n1.cross(n2);

	double area = 0.5 * n0.norm();
	return area;
}

double Lobo::computeDistancePointToTriangle(Eigen::Vector3d &v1, Eigen::Vector3d &v2, Eigen::Vector3d &v3, Eigen::Vector3d &normal, Eigen::Vector3d &p)
{
	Eigen::Vector3d pv1 = (p - v1);
	return (p - v1).norm()*pv1.dot(normal) / (pv1.norm());
}

bool Lobo::checkPointTriangleProjection(Eigen::Vector3d &v1, Eigen::Vector3d &v2, Eigen::Vector3d &v3,  const Eigen::Vector3d &p)
{
	Eigen::Vector3d u = v2 - v1;
	Eigen::Vector3d v = v3 - v1;
	Eigen::Vector3d n = u.cross(v);
	Eigen::Vector3d w = p - v1;

	double gamma = (u.cross(w)).dot(n) / n.dot(n);
	double beta = w.cross(v).dot(n) / n.dot(n);
	double alpha = 1 - gamma - beta;

	return ((0 <= alpha) && (alpha <= 1) &&
		(0 <= beta) && (beta <= 1) &&
		(0 <= gamma) && (gamma <= 1));
}

double Lobo::computeTetVolumeABS(Eigen::Vector3d &a, Eigen::Vector3d &b, Eigen::Vector3d &c, Eigen::Vector3d &d)
{
	Eigen::Matrix3d D;
	D.col(0) = b - a;
	D.col(1) = c - a;
	D.col(2) = d - a;
	return D.determinant()/6.0;
	//return (1.0 / 6 * fabs((a - d).dot((b - d).cross(c - d))));
}