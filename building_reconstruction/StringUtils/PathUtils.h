#ifndef URBANRECONSTRUCTION_PATHUTILS_H
#define URBANRECONSTRUCTION_PATHUTILS_H

#include<string>
#include <sstream>
#include <fstream>

namespace path_utils {
    int getIndexBeforeChar(std::string path, char sign);

    std::string getFileName(std::string file_path);
}
#endif //URBANRECONSTRUCTION_PATHUTILS_H
