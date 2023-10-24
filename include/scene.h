#ifndef SCENE_H_
#define SCENE_H_

#include "rasterize.h"
#include <map>

class Scene {
public:

    Eigen::Matrix4d perspective_transform;
    std::vector<Object> objects;
    std::vector<Light> lights;
    Eigen::Matrix4d camera_transform;
    Eigen::Vector3d camera_position;

    Scene (const char* filename) {
        std::ifstream fin(filename);
        Camera c(fin);
        camera_transform = c.get_matrix();
        perspective_transform = c.get_perspective_matrix();

        std::string line;
        std::getline(fin, line);

        while (!line.empty()) {
            Light l;
            l.read_file(line);
            lights.push_back(l);
            std::getline(fin, line);
        }

        std::getline(fin, line);

        std::map<std::string, Object> obj_map;

        while (std::getline(fin, line) && line != "") {

            std::istringstream iss(line);
            std::string obj_name, obj_file;

            iss >> obj_name >> obj_file;
            
            Object o(obj_file.c_str());
            obj_map.insert({obj_name, o});
        }

        while (std::getline(fin, line)) {

            std::istringstream iss(line);
            std::string obj_name;

            iss >> obj_name;

            Eigen::Matrix4d curr_mat = Eigen::MatrixXd::Identity(4, 4);
            Material m;

            while (std::getline(fin, line) && line != "") {
                std::istringstream iss2(line);
                std::string token;

                iss2 >> token;

                if (token == "t") {
                    double tx, ty, tz;
                    iss2 >> tx >> ty >> tz;
                    Translation t(tx, ty, tz);
                    curr_mat = t.get_matrix() * curr_mat;
                } else if (token == "s") {
                    double sx, sy, sz;
                    iss2 >> sx >> sy >> sz;
                    Scale s(sx, sy, sz);
                    curr_mat = s.get_matrix() * curr_mat;
                } else if (token == "r") {
                    double rx, ry, rz, theta;
                    iss2 >> rx >> ry >> rz >> theta;
                    Rotation r(rx, ry, rz, theta);
                    curr_mat = r.get_matrix() * curr_mat;
                } else if (token == "ambient") {
                    iss2 >> m.ambient(0) >> m.ambient(1) >> m.ambient(2);
                } else if (token == "diffuse") {
                    iss2 >> m.diffuse(0) >> m.diffuse(1) >> m.diffuse(2);
                } else if (token == "specular") {
                    iss2 >> m.specular(0) >> m.specular(1) >> m.specular(2);
                } else if (token == "shininess") {
                    iss2 >> m.shininess;
                }
            }

            Object obj = obj_map[obj_name];
            obj.material = m;

            for (size_t i = 1; i < obj.vertices.size(); i++) {
                Vertex v = obj.vertices[i];
                Eigen::Vector4d vec{v(0), v(1), v(2), 1};
                vec = curr_mat * vec;
                obj.vertices[i] = Vertex{vec(0), vec(1), vec(2)};
            }

            Eigen::Matrix3d normal_transform = curr_mat.block<3, 3>(0, 0);
            normal_transform = normal_transform.inverse().transpose();

            for (size_t i = 1; i < obj.normals.size(); i++) {
                Normal vn = obj.normals[i];
                vn = normal_transform * vn;
                vn.normalize();
                obj.normals[i] = Vertex{vn(0), vn(1), vn(2)};
            }

            objects.push_back(obj);
            std::cout << std::endl;
        }
    }

    void shaded_renderer(int mode, Image &image) {
        Eigen::Matrix4d ndc_transform = perspective_transform * camera_transform.inverse();

        for (Object &obj : objects) {
            shade_object(obj, lights, camera_position, ndc_transform, image, mode);
        }
    }
};

#endif