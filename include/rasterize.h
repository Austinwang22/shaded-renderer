#include "object.h"

struct VNC {
    Vertex vertex;
    Normal normal;
    Color color;
};

bool is_valid_NDC(Vertex &v) {
    return (v(0) >= -1.0 && v(0) < 1.0 && v(1) >= -1.0 && v(1) < 1.0
            && v(2) >= -1.0 && v(2) < 1.0);
}

Vertex world_to_NDC(Vertex &v, Eigen::Matrix4d &M) {
    Eigen::Vector4d vec{v(0), v(1), v(2), 1};
    vec = M * vec;

    return Eigen::Vector3d{vec(0) / vec(3), vec(1) / vec(3), vec(2) / vec(3)};
}

Eigen::Vector2i NDC_to_screen(Vertex &v, int xres, int yres) {
    int x = int((v(0) + 1.0) / 2.0 * xres);
    int y = yres - 1 - int((v(1) + 1.0) / 2.0 * yres);

    return Eigen::Vector2i{x, y};
}

double get_alpha(Eigen::Vector2i v_a, Eigen::Vector2i v_b, Eigen::Vector2i v_c, int vx, int vy) {
    int num = (v_b(1) - v_c(1)) * vx + (v_c(0) - v_b(0)) * vy + v_b(0) * v_c(1) - v_c(0) * v_b(1);
    int denom = (v_b(1) - v_c(1)) * v_a(0) + (v_c(0) - v_b(0)) * v_a(1) + v_b(0) * v_c(1) - v_c(0) * v_b(1);

    return (double) num / denom;
}

double get_beta(Eigen::Vector2i v_a, Eigen::Vector2i v_b, Eigen::Vector2i v_c, int vx, int vy) {
    int num = (v_a(1) - v_c(1)) * vx + (v_c(0) - v_a(0)) * vy + v_a(0) * v_c(1) - v_c(0) * v_a(1);
    int denom = (v_a(1) - v_c(1)) * v_b(0) + (v_c(0) - v_a(0)) * v_b(1) + v_a(0) * v_c(1) - v_c(0) * v_a(1);

    return (double) num / denom;
}

double get_gamma(Eigen::Vector2i v_a, Eigen::Vector2i v_b, Eigen::Vector2i v_c, int vx, int vy) {
    return 1.0 - get_alpha(v_a, v_b, v_c, vx, vy) - get_beta(v_a, v_b, v_c, vx, vy);
}

Color get_lighting(Vertex &v, Normal &n, Material &m, std::vector<Light> lights, Eigen::Vector3d &e) {
    Color diffuse_sum{0, 0, 0};
    Color specular_sum{0, 0, 0};
    Eigen::Vector3d e_direction = (e - v).normalized();

    for (const Light &l : lights) {
        double d = (v - l.position).norm();
        Color l_c = l.color / (1 + l.k * pow(d, 2));

        Eigen::Vector3d l_direction = (l.position - v).normalized();

        Color l_diffuse = l_c * std::max(0.0, n.dot(l_direction));
        diffuse_sum += l_diffuse;

        Color l_specular = l_c
            * pow(std::max(0.0, n.dot((e_direction + l_direction).normalized())),
            m.shininess);
        specular_sum += l_specular;
    }

    Color color = diffuse_sum.cwiseProduct(m.diffuse) + specular_sum.cwiseProduct(m.specular) + m.ambient;
    double r = std::min(1.0, color(0));
    double g = std::min(1.0, color(1));
    double b = std::min(1.0, color(2));

    return Color{r, g, b};
}

void rasterize_colored_triangle(VNC &a, VNC &b, VNC &c,
                                Eigen::Matrix4d &ndc_transform, Image &image) {
    Eigen::Vector3d ndc_1 = world_to_NDC(a.vertex, ndc_transform);
    Eigen::Vector3d ndc_2 = world_to_NDC(b.vertex, ndc_transform);
    Eigen::Vector3d ndc_3 = world_to_NDC(c.vertex, ndc_transform);
    Eigen::Vector3d cross = (ndc_3 - ndc_2).cross(ndc_1 - ndc_2);
    if (cross(2) < 0.0)
        return;
    Eigen::Vector2i coord1 = NDC_to_screen(ndc_1, image.xres, image.yres);
    Eigen::Vector2i coord2 = NDC_to_screen(ndc_2, image.xres, image.yres);
    Eigen::Vector2i coord3 = NDC_to_screen(ndc_3, image.xres, image.yres);

    int x_min = std::min({coord1(0), coord2(0), coord3(0)});
    int x_max = std::max({coord1(0), coord2(0), coord3(0)});
    int y_min = std::min({coord1(1), coord2(1), coord3(1)});
    int y_max = std::max({coord1(1), coord2(1), coord3(1)});

    for (int x = x_min; x <= x_max; x++) {
        for (int y = y_min; y <= y_max; y++) {
            double alpha = get_alpha(coord1, coord2, coord3, x, y);
            double beta = get_beta(coord1, coord2, coord3, x, y);
            double gamma = get_gamma(coord1, coord2, coord3, x, y);

            if (alpha >= 0.0 && alpha <= 1.0 && beta >= 0.0 && beta <= 1.0
                && gamma >= 0.0 && gamma <= 1.0) {
                Eigen::Vector3d ndc = alpha * ndc_1 + beta * ndc_2 + gamma * ndc_3;
                if (is_valid_NDC(ndc) && ndc(2) < image.buffer[y][x]) {
                    image.buffer[y][x] = ndc(2);

                    double c_r = alpha * (a.color)(0) + beta * (b.color)(0) + gamma * (c.color)(0);
                    double c_g = alpha * (a.color)(1) + beta * (b.color)(1) + gamma * (c.color)(1);
                    double c_b = alpha * (a.color)(2) + beta * (b.color)(2) + gamma * (c.color)(2);

                    image.pixels[y][x] = Color{c_r, c_g, c_b};
                }
            }
        }
    }
}

void phong(VNC &a, VNC &b, VNC &c,
                    Material &m, std::vector<Light> &lights, Eigen::Vector3d &e,
                    Eigen::Matrix4d &ndc_transform, Image &image) {
    Eigen::Vector3d ndc_1 = world_to_NDC(a.vertex, ndc_transform);
    Eigen::Vector3d ndc_2 = world_to_NDC(b.vertex, ndc_transform);
    Eigen::Vector3d ndc_3 = world_to_NDC(c.vertex, ndc_transform);

    Eigen::Vector3d cross = (ndc_3 - ndc_2).cross(ndc_1 - ndc_2);

    if (cross(2) < 0)
        return;

    Eigen::Vector2i coord1 = NDC_to_screen(ndc_1, image.xres, image.yres);
    Eigen::Vector2i coord2 = NDC_to_screen(ndc_2, image.xres, image.yres);
    Eigen::Vector2i coord3 = NDC_to_screen(ndc_3, image.xres, image.yres);

    int x_min = std::min({coord1(0), coord2(0), coord3(0)});
    int x_max = std::max({coord1(0), coord2(0), coord3(0)});
    int y_min = std::min({coord1(1), coord2(1), coord3(1)});
    int y_max = std::max({coord1(1), coord2(1), coord3(1)});

    for (int x = x_min; x <= x_max; x++) {
        for (int y = y_min; y <= y_max; y++) {
            float alpha = get_alpha(coord1, coord2, coord3, x, y);
            float beta = get_beta(coord1, coord2, coord3, x, y);
            float gamma = get_gamma(coord1, coord2, coord3, x, y);

            if (alpha >= 0 && alpha <= 1 && beta >= 0 && beta <= 1
                && gamma >= 0 && gamma <= 1) {
                Eigen::Vector3d ndc = alpha * ndc_1 + beta * ndc_2 + gamma * ndc_3;

                if (is_valid_NDC(ndc) && ndc(2) < image.buffer[y][x]) {
                    image.buffer[y][x] = (float) ndc(2);
                    Eigen::Vector3d n = alpha * a.normal + beta * b.normal + gamma * c.normal;
                    Eigen::Vector3d v = alpha * a.vertex + beta * b.vertex + gamma * c.vertex;
                    image.pixels[y][x] = get_lighting(v, n, m, lights, e);
                }
            }
        }
    }
}

void gouraud(VNC &a, VNC &b, VNC &c,
                        Material &mat, std::vector<Light> &lights, Eigen::Vector3d &e,
                        Eigen::Matrix4d &ndc_transform, Image &image) {
    a.color = get_lighting(a.vertex, a.normal, mat, lights, e);
    b.color = get_lighting(b.vertex, b.normal, mat, lights, e);
    c.color = get_lighting(c.vertex, c.normal, mat, lights, e);

    rasterize_colored_triangle(a, b, c, ndc_transform, image);
}

void shade_object(Object &obj, std::vector<Light> &lights, Eigen::Vector3d &camera_position,
                      Eigen::Matrix4d &ndc_transform, Image &image, int mode) {
    for (const Face &f : obj.faces) {
        Vertex v1 = obj.vertices[f.v1];
        Vertex v2 = obj.vertices[f.v2];
        Vertex v3 = obj.vertices[f.v3];

        Normal vn1 = obj.normals[f.vn1];
        Normal vn2 = obj.normals[f.vn2];
        Normal vn3 = obj.normals[f.vn3];

        VNC a = {v1, vn1, Color{0, 0, 0}};
        VNC b = {v2, vn2, Color{0, 0, 0}};
        VNC c = {v3, vn3, Color{0, 0, 0}};

        if (mode == 0) {
            gouraud(a, b, c, obj.material, lights, camera_position, ndc_transform, image);
        }
        else if (mode == 1) {
            phong(a, b, c, obj.material, lights, camera_position, ndc_transform, image);
        }
    }
}