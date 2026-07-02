#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <Eigen/Core>
#include <Eigen/Dense>

class DataLogger {
private:
    std::ofstream file;
    bool isOpen;
    std::string filename;
    int iterationCount;
    
    // Helper to create timestamp for unique filenames
    std::string getTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
        return ss.str();
    }

public:
    DataLogger() : isOpen(false), iterationCount(0) {}
    
    ~DataLogger() {
        close();
    }
    
    // Initialize logger with a name and optional timestamp
    bool init(const std::string& name, bool addTimestamp = true) {
        // Create logs directory if it doesn't exist
        std::filesystem::create_directories("logs");
        
        // Build filename
        std::stringstream ss;
        ss << "logs/" << name;
        if (addTimestamp) {
            ss << "_" << getTimestamp();
        }
        ss << ".csv";
        
        filename = ss.str();
        
        file.open(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open log file: " << filename << std::endl;
            isOpen = false;
            return false;
        }
        
        isOpen = true;
        iterationCount = 0;
        std::cout << "Logging to: " << filename << std::endl;
        return true;
    }
    
    // Log Eigen vector (dynamic size)
    template<typename Derived>
    bool logVector(const Eigen::MatrixBase<Derived>& vec) {
        if (!isOpen) {
            std::cerr << "Error: Logger not initialized for " << filename << std::endl;
            return false;
        }
        
        for (int i = 0; i < vec.size(); i++) {
            file << vec(i);
            if (i < vec.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
        iterationCount++;
        return true;
    }
    
    // Log multiple vectors as columns (for debugging)
    template<typename Derived1, typename Derived2>
    bool logMultiple(const Eigen::MatrixBase<Derived1>& vec1, 
                     const Eigen::MatrixBase<Derived2>& vec2) {
        if (!isOpen) {
            std::cerr << "Error: Logger not initialized for " << filename << std::endl;
            return false;
        }
        
        // Log vec1
        for (int i = 0; i < vec1.size(); i++) {
            file << vec1(i);
            file << ",";
        }
        
        // Log vec2
        for (int i = 0; i < vec2.size(); i++) {
            file << vec2(i);
            if (i < vec2.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
        iterationCount++;
        return true;
    }
    
    // Log with time column
    template<typename Derived>
    bool logWithTime(float time, const Eigen::MatrixBase<Derived>& vec) {
        if (!isOpen) {
            std::cerr << "Error: Logger not initialized for " << filename << std::endl;
            return false;
        }
        
        file << time << ",";
        for (int i = 0; i < vec.size(); i++) {
            file << vec(i);
            if (i < vec.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
        iterationCount++;
        return true;
    }
    
    // Log raw float array
    bool logArray(const float* data, int size) {
        if (!isOpen) {
            std::cerr << "Error: Logger not initialized for " << filename << std::endl;
            return false;
        }
        
        for (int i = 0; i < size; i++) {
            file << data[i];
            if (i < size - 1) {
                file << ",";
            }
        }
        file << "\n";
        iterationCount++;
        return true;
    }
    
    // Add header to CSV
    template<typename Derived>
    bool writeHeader(const Eigen::MatrixBase<Derived>& vec, const std::string& prefix = "") {
        if (!isOpen) {
            std::cerr << "Error: Logger not initialized for " << filename << std::endl;
            return false;
        }
        
        for (int i = 0; i < vec.size(); i++) {
            if (!prefix.empty()) {
                file << prefix << "_";
            }
            file << i;
            if (i < vec.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
        return true;
    }
    
    bool writeHeader(const std::vector<std::string>& names) {
        if (!isOpen) {
            std::cerr << "Error: Logger not initialized for " << filename << std::endl;
            return false;
        }
        
        for (size_t i = 0; i < names.size(); i++) {
            file << names[i];
            if (i < names.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
        return true;
    }
    
    void close() {
        if (isOpen && file.is_open()) {
            file.close();
            isOpen = false;
            std::cout << "Closed log: " << filename << " (" << iterationCount << " entries)" << std::endl;
        }
    }
    // Add these to the DataLogger class in logger.h
bool logStates(float time, const Eigen::Matrix<float,9,1>& states) {
    if (!isOpen || !file.is_open()) {
        return false;
    }
    
    file << time << ","
         << states(0) << "," << states(1) << "," << states(2) << ","
         << states(3) << "," << states(4) << "," << states(5) << ","
         << states(6) << "," << states(7) << "," << states(8) << "\n";
    iterationCount++;
    return true;
}

bool logForces(float time, const Eigen::Vector3f& F_aero, 
               const Eigen::Vector3f& F_grav, 
               const Eigen::Vector3f& F_total) {
    if (!isOpen || !file.is_open()) {
        return false;
    }
    
    file << time << ","
         << F_aero(0) << "," << F_aero(1) << "," << F_aero(2) << ","
         << F_grav(0) << "," << F_grav(1) << "," << F_grav(2) << ","
         << F_total(0) << "," << F_total(1) << "," << F_total(2) << "\n";
    iterationCount++;
    return true;
}
    
    bool isActive() const { return isOpen; }
    int getIterationCount() const { return iterationCount; }
    std::string getFilename() const { return filename; }
};