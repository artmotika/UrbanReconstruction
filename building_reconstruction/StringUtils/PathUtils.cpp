#include "PathUtils.h"

namespace path_utils {
    int getIndexBeforeChar(std::string path, char sign) {
        unsigned int path_len = path.size();
        for (int i = path_len - 1; i >= 0; i--) {
            if (path[i] == sign) return i;
        }
        std::ostringstream oss;
        oss << "path argument in Panorama2cubemap::getIndexBeforeChar(std::string path, char sign) doesn't have a " << sign
            << " :  " << path << "!\n";
        throw std::invalid_argument(oss.str());
    }

    std::string getFileName(std::string file_path) {
        int slash_index = getIndexBeforeChar(file_path, '/');
        int dot_index = getIndexBeforeChar(file_path, '.');
        int str_length = dot_index - slash_index - 1;
        return file_path.substr(slash_index + 1, str_length);
    }
}