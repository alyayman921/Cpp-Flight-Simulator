#include <fstream>
#include <string>

// Function to read controls from a simple text file
bool readControlsFromFile(const std::string& filename, Eigen::Matrix<float, 4, 1>& controls) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open controls file: " << filename << std::endl;
        return false;
    }
    
    float da, de, dth, dr;
    if (file >> da >> de >> dth >> dr) {
        controls << da, de, dth, dr;
        file.close();
        return true;
    } else {
        std::cerr << "Error: Invalid format in controls file. Expected 4 floats." << std::endl;
        file.close();
        return false;
    }
}
