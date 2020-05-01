#pragma once
#include <Eigen/Dense>

namespace Lobo
{

void computeSVD(Eigen::Matrix3d &F, Eigen::Matrix3d &U, Eigen::Matrix3d &V, Eigen::Matrix3d &singularF, double singularValue_eps, int modifiedSVD);

void eigen_sym(Eigen::Matrix3d &M, Eigen::Vector3d &eig_val, Eigen::Vector3d (&eig_vec)[3]);

void findOrthonormalVector(Eigen::Vector3d &v1, Eigen::Vector3d &v2);
} // namespace Lobo