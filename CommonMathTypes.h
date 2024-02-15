#pragma once
#include "eigen-3.4.0/Eigen/Dense"

/*
This header file includes all the common datatypes and their shorthands used within this project
*/
namespace MathCommon
{
    template<typename T>
    extern const T EPSILON = T(0.0001);

    // Too manu digits 
    template<typename T>
    extern const T PI = T(3.141592653589793238462643383279502884);

    template<typename T>
    extern const T INF = Eigen::Infinity;

    template<typename T>
    using Matrix3 = Eigen::Matrix<T,3,3>;

    template<typename T>
    using Matrix4 = Eigen::Matrix<T,4,4>;

    // 3D vector
    template<typename T>
    using Vector3 = Eigen::Matrix<T,3,1>; //Column vector

    // 4D vector
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

    template<typename T>
    struct Plane3D
    {
        /*Potential for improvement*/
        Vector3<T> point;
        Vector3<T> normal;
    };

    // Breaking code style for readablility here
    template<typename Type>
    struct Basis3D
    {
        Vector3<Type> X;
        Vector3<Type> Y;
        Vector3<Type> Z;
        Vector3<Type> T;

        Basis3D() = default;
        Basis3D(Vector3<Type> x, Vector3<Type> y, Vector3<Type> z) : X(x), Y(y), Z(z), T(Type(0.0),Type(0.0),Type(0.0))
        {}
    };

    template<typename T>
    using Surface3D = Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic>;

    template<typename T>
    MathCommon::Basis3D<T> BasisFromDirection(const MathCommon::Vector3<T>& direction)
    {
        // crreate a basis using the least significant component of a direction
        //Amazing code originally in python but templated here for C++, Erin Catto @ https://box2d.org/

        MathCommon::Vector3<T> t1,t2;

        if (direction[0] >= T(0.57735))
            t1<<direction[1], -direction[0], T(0.0);
        else
            t1<<T(0.0), direction[2], -direction[1];
    
        //least significant component of a direction
        t1.normalize();


        t2 = (direction.cross(t1)).normalize();
        return MathCommon::Basis3D<T>(direction,t1,t2);
    }
}