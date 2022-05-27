// page3.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

//https://gitlab.com/libeigen/eigen.git
#include <F:/eigen/Eigen/Core>
#include <F:/eigen/Eigen/Dense>
# include <F:/eigen/Eigen/Geometry>

#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;
using namespace Eigen;

//Matrix3f A;
//Matrix4d B;
//Matrix <short , 5, 5> M1;

void Fun1() {
    cout << " Eigen version : " << EIGEN_MAJOR_VERSION << "."
    << EIGEN_MINOR_VERSION << endl;
}

//////////////Initialization
void Fun2() {
    Matrix3f A;

    // Initialize A
    A << 1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f;
    cout << A << endl;
}

//Eigen stores matrices in column-major order by default
void Fun3() {
    Matrix4d B;
    // Initialize B by accessing individual elements
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j<4; ++j) {
            B(i, j) = i+j;
        }
    }
    cout << B << endl;
}

void Fun4() {
    //Matrix3f A = Matrix3f::Random();
    //Matrix4d A = Matrix4d::Identity();
    //Matrix3f A = Matrix3f::Zero();
    //Matrix3f A = Matrix3f::Ones();
    Matrix4d A = Matrix4d::Constant(4.5);

    cout << A << endl;
}

////////Matrix Operations
void Fun10() {
    Matrix4f M1 = Matrix4f::Identity();
    Matrix4f M2 = Matrix4f::Constant(2.2f);

    cout << M1 + M2 << endl << endl;
    cout << M1 * M2 << endl << endl;
    cout << M2 - Matrix4f::Ones() * 2.2 << endl << endl;

    cout << (M2 - Matrix4f::Ones() * 2.2 == Matrix4f::Zero()) << endl << endl;      //1 //float
}

void Fun11() {
    Matrix4f M1 = Matrix4f::Random();

    cout << M1 << endl << endl;
    cout << M1.transpose() << endl << endl;     //转置
    cout << M1.inverse() << endl << endl;       //逆
}

void Page5() {
    //array()是元素一一乘/比较，不是矩阵乘/比较
    Matrix4f M1 = Matrix4f::Random();
    cout << M1 << endl << endl;

    cout << "Square each element of the matrix, 一一乘除，不是矩阵乘" << endl;
    cout << M1.array().square() << endl << endl;
    //cout << M1 << endl <<endl; //M1 is unchanged

    cout << "Multiply two matrices element - wise" << endl;
    cout << M1.array() * Matrix4f::Identity().array() << endl;

    cout << "All relational operators can be applied element - wise" << endl;
    Matrix4f M2 = Matrix4f::Random();
    cout << M2 << endl << endl;
    cout << (M1.array() <= M2.array()) << endl << endl;
    cout << (M1.array() > M2.array()) << endl;
}

void Page6() {
    //typedef Matrix <float, 3, 1> Vector3f;
    //typedef Matrix <double, 4, 1> Vector4d;
    /*
    Vector3f v;
    v << 1.0f, 2.0f, 3.0f;
    cout << v(2) << endl;   //3.0f

    Vector3f w(1.0f, 2.0f, 3.0f);
    cout << w << endl;*/

    // Utility functions
    Vector3f v1 = Vector3f::Ones();
    //cout << v1 << endl << endl;
    Vector3f v2 = Vector3f::Zero();
    Vector4d v3 = Vector4d::Random();
    Vector4d v4 = Vector4d::Constant(1.8);
    //cout << v4 << endl << endl;

    //cout << v1 + v2 << endl << endl;
    /*cout << v4 << endl << endl;
    cout << v3 << endl << endl;
    cout << v4 - v3 << endl;*/

    //cout << v4 * 2 << endl; //[3.6 3.6 3.6]T

    // Equality
    // Again , equality and inequality are the only relational
    // operators that work with vectors
    cout << (Vector2f::Ones() * 3 == Vector2f::Constant(3)) << endl;    //only 1
}

void Page6B() {
    Vector4f v5 = Vector4f(1.0f, 2.0f, 3.0f, 4.0f);
    Matrix4f m4 = Matrix4f::Random();
    cout << m4 << endl << endl;

    cout << m4 * v5 << endl;        //真正矩阵 * 向量
    //cout << v5 * m4 << endl;    //error C2338: INVALID_MATRIX_PRODUCT
}

void Page7() {
    Vector3f v1 = Vector3f::Random();
    cout << v1 << endl << endl;
    Vector3f v2 = Vector3f::Random();
    cout << v2 << endl << endl;

    //cout << v1 * v2.transpose() << endl; //3 x 3

    cout << v1.dot(v2) << endl << endl;
    cout << v1.normalized() << endl << endl;
    cout << v1.cross(v2) << endl;
}

void Page7A() {
    // Convert a vector to and from homogenous coordinates  //齐次坐标  //齐次点(a,b,1) 齐次向量(a,b,0)
    //齐次坐标就是用N+1维来代表N维坐标
    Vector3f s = Vector3f::Random();
    cout << s << endl << endl;
    Vector4f q = s.homogeneous();
    cout << q << endl << endl;
    cout << (s == q.hnormalized()) << endl;
}

void Page7B() {
    Vector3f v1 = Vector3f::Random();
    cout << v1 << endl << endl;

    Vector3f v2 = Vector3f::Random();
    cout << v2 << endl << endl;
    
    cout << v1.array() * v2.array() << endl << endl;    //元素一一乘，不是矩阵乘
    cout << v1.array().sin() << endl;
}


void Page8() {
    float arrVertices[] = { -1.0, -1.0, -1.0,
                            1.0 , -1.0, -1.0,
                            1.0 , 1.0 , -1.0,
                            -1.0, 1.0 , -1.0,
                            -1.0, -1.0, 1.0 ,
                            1.0 , -1.0, 1.0 ,
                            1.0 , 1.0 , 1.0 ,
                            -1.0, 1.0 , 1.0 };
    MatrixXf mVertices = Map < Matrix <float, 3, 8> >(arrVertices);
    //Affine: 仿射
    //U = TRSI
    Transform <float, 3, Affine > t = Transform <float, 3, Affine >::Identity();    //t.matrix()
    cout << "t.scale(0.8f)" << endl;
    t.scale(0.8f);
    //cout << t << endl;
    cout << t.matrix() << endl; //齐次

    cout << "\nt.rotate()" << endl;
    t.rotate( AngleAxisf(0.25f * M_PI, Vector3f::UnitX()) );
    cout << t.matrix() << endl;

    cout << "\nt.translate()" << endl;
    t.translate( Vector3f(1.5, 10.2, -5.1) );
    cout << t.matrix() << endl << endl;

    cout << t * mVertices.colwise().homogeneous() << endl << endl;
    cout << t * (mVertices.colwise().homogeneous()) << endl << endl;
}


//https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
int main()
{
    //Fun1();
    //Fun2();    
    //Fun3();
    //Fun4();
    //Fun10();
    //Fun11();

    //Page5();
    //Page6();
    //Page6B();
    //Page7();
    //Page7A();
    //Page7B();
    Page8();

    return 0;
}
