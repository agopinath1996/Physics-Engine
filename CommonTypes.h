#pragma once
#include "eigen-3.4.0/Eigen/Dense"

/*
This header file includes all the common datatypes and their shorthands used within this project
*/
namespace MathCommon
{
    template<typename T>
    using Matrix3 = Eigen::Matrix<T,3,3>;

    template<typename T>
    using Matrix4 = Eigen::Matrix<T,4,4>;

    template<typename T>
    using Vector3 = Eigen::Matrix<T,3,1>; //Column vector

    template<typename T>
    using Vector4 = Eigen::Matrix<T,4,1>;

    template<typename T>
    using MatrixDynamic = Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic>;

    template<typename T>
    constexpr Eigen::Matrix<T,3,3> Identity3 = Eigen::Matrix<T,3,3>::Identity();

    template<typename T>
    constexpr Eigen::Matrix<T,3,3> Identity4= Eigen::Matrix<T,4,4>::Identity();

    template<typename T>
    struct Ray3
    {
        Vector3<T> pos;
        Vector3<T> dir;
    };
}