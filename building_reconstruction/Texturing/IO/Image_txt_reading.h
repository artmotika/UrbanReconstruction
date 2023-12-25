#ifndef URBANRECONSTRUCTION_IMAGE_TXT_READING_H
#define URBANRECONSTRUCTION_IMAGE_TXT_READING_H

#include <fstream>
#include <iostream>
#include <sstream>

namespace urban_rec
{
    namespace image_txt_read {
        std::ifstream& go_to_line(std::ifstream& file, unsigned int num);
    }
}


#endif //URBANRECONSTRUCTION_IMAGE_TXT_READING_H
