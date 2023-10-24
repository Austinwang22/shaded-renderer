#ifndef IMAGE_H_
#define IMAGE_H_

#include "color.h"

#include <fstream>
#include <iostream>

class Image {
public:

    int xres, yres;
    Color **pixels;
    double **buffer;

    Image(int xres, int yres) {
        this->xres = xres;
        this->yres = yres;

        pixels = new Color*[yres];
        buffer = new double*[yres];

        for (size_t i = 0; i < yres; i++) {
            pixels[i] = new Color[xres];
            buffer[i] = new double[xres];
            for (size_t j = 0; j < xres; j++) {
                pixels[i][j] = Color{0, 0, 0};
                buffer[i][j] = (double) MAXFLOAT;
            }
        }
    }

    ~Image() {
        for (size_t i = 0; i < yres; i++) {
            delete [] buffer[i];
            delete [] pixels[i];
        }
        delete [] buffer;
        delete [] pixels;
    }

    void write_file(std::string filename) {
        std::ofstream image(filename, std::ios::binary);
        image << "P3\n";
        image << yres << " " << xres << "\n";
        image << "255\n";

        for (int i = 0; i < yres; i++) {
            for (int j = 0; j < xres; j++) {
                image << (int) (pixels[i][j](0) * 255) << " " << (int) (pixels[i][j](1) * 255) 
                      << " " << (int) (pixels[i][j](2) * 255) << "\n";
                std::cout << (int) (pixels[i][j](0) * 255) << " " << (int) (pixels[i][j](1) * 255) 
                      << " " << (int) (pixels[i][j](2) * 255) << std::endl;
            }   
        }

        image.close();
    }
};

#endif