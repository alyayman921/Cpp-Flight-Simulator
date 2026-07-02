#include <iostream>
#include <sys/stat.h>
#include "readxslx.h"
#include "derivatives.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "RBDEqns.h"
#include "rk4.h"
#include "controls.h"

const char filename[] = "meta/C5A.xlsx";
Eigen::Matrix<float, 4, 1> Controls;
const float dt = 0.01;
const float tfinal = 200;

bool fileExists(const char* path) {
    struct stat buffer;
    return (stat(path, &buffer) == 0);
}

int main() {
    if (!fileExists(filename)) {
        std::cerr << "Error: aircraft data file not found at '" << filename << "'" << std::endl;
        std::cerr << "Run this from the directory containing meta/C5A.xlsx" << std::endl;
        return 1;
    }

    if (!readControlsFromFile("controls.txt", Controls)) {
        std::cerr << "Failed to read controls. Using defaults..." << std::endl;
        Controls << 0.0, 0.0, 1000.0, 0.05;
    }   
    std::cout << "Controls loaded: " << Controls.transpose() << std::endl;

    // Read aircraft data
    raw_data raw = readxlsx(filename);
    aircraft_data c5a = sorting(raw);

    RBDsolve RBD(c5a, Controls);

    // Initial state vector: [v(3), omega(3), euler(3)]
    Eigen::Matrix<float, 9, 1> initial_state;
    initial_state << c5a.V0(0), c5a.V0(1), c5a.V0(2),
                      c5a.omega0(0), c5a.omega0(1), c5a.omega0(2),
                      c5a.euler0(0), c5a.euler0(1), c5a.euler0(2);
    
    // Setup and run RK4 integration
    rk4 rk4Solver(dt, tfinal);
    Eigen::Matrix<float, 9, 1>* results = rk4Solver.rk4_solver(RBD, initial_state);

    int N_steps = (int)(tfinal / dt);
    Eigen::Matrix<float, 9, 1> final_state = results[N_steps];
    
    std::cout << "\n=== Final State (t=" << tfinal << "s) ===" << std::endl;
    std::cout << "Velocity (v_x, v_y, v_z): "
              << final_state(0) << ", " << final_state(1) << ", " << final_state(2) << std::endl;
    std::cout << "Angular velocity (p, q, r): "
              << final_state(3) << ", " << final_state(4) << ", " << final_state(5) << std::endl;
    std::cout << "Euler angles (phi, theta, psi): "
              << final_state(6) << ", " << final_state(7) << ", " << final_state(8) << std::endl;

    if (!final_state.allFinite()) {
        std::cerr << "\nWarning: final state contains NaN/Inf - simulation diverged." << std::endl;
    }

    rk4Solver.free_results();
    return 0;
}