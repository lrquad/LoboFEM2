#pragma once
#include <Eigen/Dense>
#include "glm/glm.hpp"
#include <glm/matrix.hpp>

//thanks to  podgorskiy
namespace Lobo
{

template <typename T, int m, int n>
inline glm::mat<m, n, float, glm::qualifier::highp> E_2_GLM(const Eigen::Matrix<T, m, n> &em)
{
    glm::mat<m, n, float, glm::qualifier::highp> mat;
    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            mat[j][i] = em(i, j);
        }
    }
    return mat;
}

template <typename T, int m>
inline glm::vec<m, float, glm::qualifier::highp> E_2_GLM(const Eigen::Matrix<T, m, 1> &em)
{
    glm::vec<m, float, glm::qualifier::highp> v;
    for (int i = 0; i < m; ++i)
    {
        v[i] = em(i);
    }
    return v;
}

template <typename T, int m, int n>
inline Eigen::Matrix<T, m, n> GLM_2_E(const glm::mat<m, n, float, glm::qualifier::highp> &glmm)
{
    Eigen::Matrix<T, m, n> mat;

    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            mat.data()[j * m + i] = glmm[j][i];
        }
    }
    return mat;
}

template <typename T, int m>
inline Eigen::Matrix<T, m, 1> GLM_2_E(const glm::vec<m, float, glm::qualifier::highp> &glmm)
{

    Eigen::Matrix<T, m, 1> v;
    for (int i = 0; i < m; ++i)
    {
        v.data()[i] = glmm[i];
    }
    return v;
}

template<typename T>
inline Eigen::Matrix<T, -1, -1> eigen_vec_2_mat(const Eigen::Matrix<T, -1, 1> &inputmat,int row,int col)
{
    Eigen::Matrix<T, -1, -1> mat(row,col);
    for (int i = 0; i < row; ++i)
    {
        for (int j = 0; j < col; ++j)
        {
            mat.data()[j * row + i] = inputmat.data()[i*col+j];
        }
    }
    return mat;
}

} // namespace Lobo
