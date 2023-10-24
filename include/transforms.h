#ifndef TRANSFORMS_H_
#define TRANSFORMS_H_

#include <Eigen/Dense>
#include <cmath>


struct Translation {

    double tx;
    double ty;
    double tz;

    Translation(float x, float y, float z) {
        tx = x;
        ty = y;
        tz = z;
    }

    Eigen::Matrix4d get_matrix() {
        Eigen::Matrix4d mat;
        mat << 1, 0, 0, tx,
               0, 1, 0, ty,
               0, 0, 1, tz,
               0, 0, 0, 1;
        return mat;
    }

};

struct Scale {

    double sx;
    double sy;
    double sz;

    Scale(double x, double y, double z) {
        sx = x;
        sy = y;
        sz = z;
    }

    Eigen::Matrix4d get_matrix() {
        Eigen::Matrix4d mat;
        mat << sx, 0, 0, 0,
               0, sy, 0, 0,
               0, 0, sz, 0,
               0, 0, 0, 1;
        return mat;
    }

};

struct Rotation {

    double ux;
    double uy;
    double uz;
    double theta;

    Rotation(double x, double y, double z, double angle) {
        ux = x;
        uy = y;
        uz = z;
        Eigen::Vector3d axis;
        axis << ux,
                uy,
                uz;
        ux /= axis.norm();
        uy /= axis.norm();
        uz /= axis.norm();
        theta = angle;
    }

    Eigen::Matrix<double, 4, 4> get_matrix() {
        Eigen::Matrix<double, 4, 4> mat;
        double m00 = ux * ux + (1 - ux * ux) * cos(theta);
        double m01 = ux * uy * (1 - cos(theta)) - uz * sin(theta);
        double m02 = ux * uz * (1 - cos(theta)) + uy * sin(theta);

        double m10 = uy * ux * (1 - cos(theta)) + uz * sin(theta);
        double m11 = uy * uy + (1 - uy * uy) * cos(theta);
        double m12 = uy * uz * (1 - cos(theta)) - ux * sin(theta);

        double m20 = uz * ux * (1 - cos(theta)) - uy * sin(theta);
        double m21 = uz * uy * (1 - cos(theta)) + ux * sin(theta);
        double m22 = uz * uz + (1 - uz * uz) * cos(theta);

        mat << m00, m01, m02, 0,
               m10, m11, m12, 0,
               m20, m21, m22, 0,
               0, 0, 0, 1;
        return mat;
    }

};

#endif