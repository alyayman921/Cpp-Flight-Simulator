#include <iostream>
#include <sys/stat.h>
#include "readxslx.h"
#include "derivatives.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "RBDEqns.h"
#include "rk4.h"

const char filename[] = "meta/C5A.xlsx";
float da = 0.0, de = 0, dth = 1000, dr = 0;  // Control inputs: aileron, elevator, throttle, rudder
Eigen::Matrix<float, 4, 1> Controls;
const float dt = 0.2;
const float tfinal = 20;

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

    // Initialize controls
    Controls << da, de, dth, dr;

    // Read aircraft data
    raw_data raw = readxlsx(filename);
    aircraft_data c5a = sorting(raw);

    std::cout << "Trim velocity (V0): " << c5a.V0.transpose() << std::endl;
    std::cout << "Trim Euler angles (euler0): " << c5a.euler0.transpose() << std::endl;

    RBDsolve RBD(c5a, Controls);
    std::cout << "Fg0 : " << RBD.F_g0 << std::endl;
    std::cout << "Fg : " << RBD.F_g << std::endl;


    // Initial state vector: [v(3), omega(3), euler(3)]
    Eigen::Matrix<float, 9, 1> initial_state;
    initial_state << c5a.V0(0), c5a.V0(1), c5a.V0(2),             // velocities
                      c5a.omega0(0), c5a.omega0(1), c5a.omega0(2), // angular velocities
                      c5a.euler0(0), c5a.euler0(1), c5a.euler0(2); // Euler angles
    // Setup and run RK4 integration
    rk4 rk4Solver(dt, tfinal);
    
    // Verify
    //Eigen::Matrix<float,9,1> check={2,1,0,0,0,0,0,0,0};
    //Eigen::Matrix<float, 9, 1>* results = rk4Solver.rk4_solver(RBD, check); // must change rk4 to solve rbd.verify, const float dt = 0.2; const float tfinal = 20;
    // results must be near 1.03608, 5.31897 at time 20

    // Solve
    //Eigen::Matrix<float, 9, 1>* results = rk4Solver.rk4_solver(RBD, initial_state);

    int N_steps = (int)(tfinal / dt);

    // Print a short trace through the simulation (every 20s) so the
    // trajectory can be sanity-checked, not just the final state.
    std::cout << "\n=== Trajectory trace ===" << std::endl;
    std::cout << "  t(s)    v_x      v_y      v_z      p        q        r        phi      theta    psi" << std::endl;
    int print_stride = (int)(20.0f / dt);
    for (int step = 0; step <= N_steps; step += print_stride) {
        Eigen::Matrix<float, 9, 1> s = results[step];
        printf("  %5.1f  %7.3f  %7.3f  %7.3f  %7.4f  %7.4f  %7.4f  %7.4f  %7.4f  %7.4f\n",
               step * dt, s(0), s(1), s(2), s(3), s(4), s(5), s(6), s(7), s(8));
    }

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
    std::cout << "Fg : " << RBD.F_g << std::endl;

    rk4Solver.free_results();
    return 0;
}