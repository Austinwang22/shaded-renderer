#include <include/scene.h>

int main(int argc, char *argv[]) {
    if (argc != 5) {
        std::cout << "Needs the filename, xres, yres, and mode..." << std::endl;
        return 1;
    }

    char *filename = argv[1];
    int xres = atoi(argv[2]);
    int yres = atoi(argv[3]);
    int mode = atoi(argv[4]);

    Image im(xres, yres);
    Scene scene(filename);

    scene.shaded_renderer(mode, im);

    im.write_file("image.ppm");

    return 0;
}