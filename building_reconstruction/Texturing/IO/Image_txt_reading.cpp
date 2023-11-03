#include "Image_txt_reading.h"


std::ifstream& urban_rec::image_txt_read::go_to_line(std::ifstream& file, unsigned int num) {
    file.seekg(std::ios::beg);
    for (int i = 0; i < num - 1; ++i) {
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    return (file);
}

