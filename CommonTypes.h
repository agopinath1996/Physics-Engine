#pragma once
#include "eigen-3.4.0/Eigen/Dense"
/*
This header file includes all the common datatypes and their shorthands used within this project
*/
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
const Eigen::Matrix<T,3,3> Identity3 = Eigen::Matrix<T,3,3>::Identity();

template<typename T>
const Eigen::Matrix<T,4,4> Identity4 = Eigen::Matrix<T,4,4>::Identity();

