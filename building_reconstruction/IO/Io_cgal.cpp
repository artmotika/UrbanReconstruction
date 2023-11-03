#include "Io_cgal.h"

namespace Io_cgal {
    void readFileToPointSet(std::string filename, Point_set *points_start) {
        CGAL::Timer t;
        t.start();
        std::ifstream stream (filename, std::ios_base::binary);
        if (!stream)
        {
            std::cerr << "Error: cannot read file " << filename << std::endl;
            throw "error to read in readFileToPointSet(filename)";
        }
        stream >> *points_start;
        std::cout << "Read " << points_start->size () << " point(s)" << ". Time: " << t.time() << " sec." << std::endl;
        if (points_start->empty())
            throw "error to read in readFileToPointSet(filename), file is empty";
    }
    void saveFileFromPointSet(std::string filename, Point_set points_start) {
        CGAL::Timer t;
        t.start();
        std::ofstream f (filename);
        f << points_start;
        f.close ();
        std::cout << "Write " << points_start.size () << " point(s)" << ". Time: " << t.time() << " sec." << std::endl;
    }
    void readFileToPointVector(std::string filename, Point_vector *points) {
        CGAL::Timer t;
        t.start();
        // Load point set from a file.
        const std::string input_file(filename);
        std::ifstream input_stream(input_file.c_str());
        if (input_stream.fail()) {
            std::cerr << "Failed open file \'" << input_file << "\'" << std::endl;
            throw "error to read in readFileToPointVector(filename)";
        }
        input_stream.close();
        std::cout << "Loading point cloud: " << input_file << "...";

        if (!CGAL::IO::read_points(input_file.c_str(), std::back_inserter(*points),
                                   CGAL::parameters::point_map(Point_map()).normal_map(Normal_map()))) {
            std::cerr << "Error: cannot read file " << input_file << std::endl;
            throw "error to read in readFileToPointVector(filename)";
        }
        else
            std::cout << " Done. " << points->size() << " points. Time: "
                      << t.time() << " sec." << std::endl;
    }
    void saveFileFromPointVector(std::string filename, Surface_mesh model) {
        CGAL::Timer t;
        t.start();
        std::cout << "Saving...";
        const std::string& output_file(filename);
        if (CGAL::IO::write_PLY(output_file, model, CGAL::parameters::use_binary_mode(false)))
            std::cout << " Done. Saved to " << output_file << ". Time: " << t.time() << " sec." << std::endl;
        else {
            std::cerr << " Failed saving file." << std::endl;
            throw "error to save in saveFileFromPointVector(filename)";
        }
    }
}