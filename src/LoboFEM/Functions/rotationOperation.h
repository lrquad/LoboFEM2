#pragma once

#include <Eigen/Dense>


namespace lobo {


	template<typename T>
	Eigen::Matrix<T,3,3> rollPitchYawRotation(T roll, T pitch, T yaw)
	{
		Eigen::Matrix<T, 3, 3> rotation;
		rotation.setZero();

		rotation.data()[0] = std::cos(yaw)*std::cos(pitch);
		rotation.data()[1] = std::sin(yaw)*std::cos(pitch);
		rotation.data()[2] = -std::sin(pitch);


		rotation.data()[3] = std::cos(yaw)*std::sin(pitch)*std::sin(roll) - std::sin(yaw)*std::cos(roll);
		rotation.data()[4] = std::sin(yaw)*std::sin(pitch)*std::sin(roll) + std::cos(yaw)*std::cos(roll);
		rotation.data()[5] = std::cos(pitch)*std::sin(roll);


		rotation.data()[6] = std::cos(yaw)*std::sin(pitch)*std::cos(roll) + std::sin(yaw)*std::sin(roll);
		rotation.data()[7] = std::sin(yaw)*std::sin(pitch)*std::cos(roll) - std::cos(yaw)*std::sin(roll);
		rotation.data()[8] = std::cos(pitch)*std::cos(roll);

		return rotation;
	}

	template<typename T>
	Eigen::Matrix<T, 3, 3> rotationAxis(T angle,std::vector<T> &axis)
	{
		Eigen::Matrix<T, 3, 3> rotation;
		Eigen::Matrix<T, 3, 1> axis_v;
		axis_v[0] = axis[0];
		axis_v[1] = axis[1];
		axis_v[2] = axis[2];
		axis_v.normalize();

		rotation = Eigen::AngleAxis<T>(angle, axis_v).toRotationMatrix();
		return rotation;
	}

}
