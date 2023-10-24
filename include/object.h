#ifndef OBJECT_H_
#define OBJECT_H_

#include "image.h"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

typedef Eigen::Vector3d Color;
typedef Eigen::Vector3d Vertex;
typedef Eigen::Vector3d Normal;

struct Material {

    Color ambient;
    Color diffuse;
    Color specular;
    double shininess;
};

struct Face {

    int v1;
    int v2;
    int v3;
    
    int vn1;
    int vn2;
    int vn3;
};

struct Light {
    Eigen::Vector3d position;
    double k;
    Color color;

    void read_file(std::string line) {
        std::istringstream iss(line);
        std::string l;
        iss >> l >> position(0) >> position(1) >> position(2);
        iss >> l >> color(0) >> color(1) >> color(2);
        iss >> l >> k;
    }
};

class Object {
public:

    std::vector<Vertex> vertices;
    std::vector<Face> faces;
    std::vector<Normal> normals;

    Material material;

    Object() {};

    Object(const char* filename) {
        read_file(filename);
    }

    void print_data() {
        for (size_t i = 1; i < vertices.size(); i++)
            std::cout << "v " << vertices[i](0) << " " << vertices[i](1) << " " << vertices[i](2) 
                      << " " << " " << std::endl;
        for (size_t i = 1; i < normals.size(); i++)
            std::cout << "vn " << normals[i](0) << " " << normals[i](1) << " " 
                      << normals[i](2) << " " << std::endl;
        for (size_t i = 0; i < faces.size(); i++)
            std::cout << "f " << faces[i].v1 << " " << faces[i].v2 << " " << faces[i].v3 << std::endl;
        std::cout << std::endl;
    }

    void read_file(const char* filename) {
        Vertex v_0{0, 0, 0};
        Normal vn_0{0, 0, 0};
        vertices.push_back(v_0);
        normals.push_back(vn_0);

        std::ifstream fin(filename);
        std::string line;
        if (fin.is_open()) {
            while (std::getline(fin, line)) {
                std::istringstream iss(line);
                std::string token;

                iss >> token;

                if (token == "v") {
                    double x, y, z;
                    iss >> x >> y >> z;
                    Vertex v{x, y, z};
                    vertices.push_back(v);
                } else if (token == "vn") {
                    double x, y, z;
                    iss >> x >> y >> z;
                    Normal vn{x, y, z};
                    normals.push_back(vn);
                } else if (token == "f") {
                    Face f;
                    std::string vvn1, vvn2, vvn3;
                    iss >> vvn1 >> vvn2 >> vvn3;
                    f.v1 = std::stoi(vvn1.substr(0, vvn1.find("//")));
                    f.v2 = std::stoi(vvn2.substr(0, vvn2.find("//")));
                    f.v3 = std::stoi(vvn3.substr(0, vvn3.find("//")));
                    f.vn1 = std::stoi(vvn1.substr(vvn1.find("//") + 2));
                    f.vn2 = std::stoi(vvn2.substr(vvn2.find("//") + 2));
                    f.vn3 = std::stoi(vvn3.substr(vvn3.find("//") + 2));
                    faces.push_back(f);
                }
            }
        }
    }
};

class Camera {
public:

    Vertex position;
    Vertex axis;
    double theta, near, far, left, right, top, bottom;

    Camera(std::ifstream &fin) {
        read_file(fin);
    }

    Eigen::Matrix4d get_matrix() {
        Translation t(position(0), position(1), position(2));
        Rotation r(axis(0), axis(1), axis(2), theta);
        Eigen::Matrix<double, 4, 4> C = t.get_matrix() * r.get_matrix();
        return C;
    }

    Eigen::Matrix4d get_perspective_matrix() {
        Eigen::Matrix<double, 4, 4> P;
        P << 2 * near / (right - left), 0, (right + left) / (right - left), 0,
             0, 2 * near / (top - bottom), (top + bottom) / (top - bottom), 0,
             0, 0, -(far + near) / (far - near), -2 * far * near / (far - near), 
             0, 0, -1, 0;
        return P;
    }

    void read_file(std::ifstream &fin) {
        std::string line;
        std::getline(fin, line);
        std::getline(fin, line);
        std::istringstream iss1(line);
        std::string pos;
        double x, y, z;
        iss1 >> pos >> x >> y >> z;
        position = {x, y, z};

        std::getline(fin, line);
        std::string ori;
        std::istringstream iss2(line);
        iss2 >> ori >> x >> y >> z >> theta;
        axis = {x, y, z};

        std::getline(fin, line);
        std::string n;
        std::istringstream iss3(line);
        iss3 >> n >> near;

        std::getline(fin, line);
        std::string f;
        std::istringstream iss4(line);
        iss4 >> f >> far;

        std::getline(fin, line);
        std::string l;
        std::istringstream iss5(line);
        iss5 >> l >> left;

        std::getline(fin, line);
        std::string r;
        std::istringstream iss6(line);
        iss6 >> r >> right;

        std::getline(fin, line);
        std::string t;
        std::istringstream iss7(line);
        iss7 >> t >> top;

        std::getline(fin, line);
        std::string b;
        std::istringstream iss8(line);
        iss8 >> b >> bottom;

        std::getline(fin, line);
    }

};

#endif