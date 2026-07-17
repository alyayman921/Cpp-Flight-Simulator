#include "Autopilot.h"

bool fileExists(const char* path) {
    struct stat buffer;
    return (stat(path, &buffer) == 0);
}

int main(int argc, char* argv[]) {
    /*
    for (int i=0;i<argc;i++){
        printf("%s\n",argv[i]);
    }
    */

    if (argc >= 2){
        std::string arg = argv[1];
        if (arg == "--help") {
            std::cout<< "\nFlight Simulator for Lockheed Martin's C5A Aircraft\n";
            std::cout<< "Run the Simulator with the supported input arguments like \"./Autopilot arg\"\n\n";
            std::cout<<"Arguments    Usage\n--------------------------------------------------\n";
            std::cout<<"manual       read the control commands from the textfile controls.txt\n";
            std::cout<<"loop         prevent the program from exiting after solving\n";
            return 0;
        }
        if (arg == "loop") {
            loop = true;
        }
        if (arg == "manual") {
            Autopiloted = false;
        }
    }
    if (!fileExists(filename)){
        std::cerr << "Error: aircraft data file not found at '" << filename << "'" << std::endl;
        std::cerr << "Run this from the directory containing meta/C5A.xlsx" << std::endl;
        return 1;
    }

    // Variables to be read from controls file
    double dt = 0.01;      // Default values
    double tfinal = 100.0;

    if (!readControlsFromFile("controls.txt", Controls, dt, tfinal)) {
        std::cerr << "Failed to read controls. Using defaults..." << std::endl;
        Controls << 0.0, 0.0, 1000.0, 0.05;
        dt = 0.01;
        tfinal = 100.0;
    }   
    std::cout << "Controls loaded: " << Controls.transpose() << std::endl;
    std::cout << "Timestep: " << dt << " s" << std::endl;
    std::cout << "Final time: " << tfinal << " s" << std::endl;
    std::cout << "Number of steps: " << (int)(tfinal / dt) << std::endl;

    // Read aircraft data
    raw_data raw = readxlsx(filename);
    aircraft_data c5a = sorting(raw);

    // Initialize the controller and keep the control vector the same across files
    controller c(&Controls,dt,set_pitch,Autopiloted);
    RBDsolve RBD(c5a, &Controls,Autopiloted);
    //std::cout<<"Controller Didn't Crash it"<<std::endl;


    // Initial state vector: [uvw, pqr, euler(3)]
    Eigen::Matrix<double, 9, 1> initial_state;
    initial_state << c5a.V0(0), c5a.V0(1), c5a.V0(2),
                      c5a.omega0(0), c5a.omega0(1), c5a.omega0(2),
                      c5a.euler0(0), c5a.euler0(1), c5a.euler0(2);
    

    // Setup and run RK4 integration
    rk4 rk4Solver(dt, tfinal);
    rk4Solver.resultsPointer(c);
    //std::cout<<"rk4 init Didn't Crash it"<<std::endl;

    Eigen::Matrix<double, 9, 1>* results = rk4Solver.rk4_solver(RBD,c, initial_state);
    //std::cout<<"surely this Didn't Crash it"<<std::endl;

    int N_steps = (int)(tfinal / dt);
    Eigen::Matrix<double, 9, 1> final_state = results[N_steps];

    std::cout << "\n=== Final State (t=" << tfinal << "s) ===" << std::endl;
    std::cout << "Velocity (v_x, v_y, v_z): "
              << final_state(0) << ", " << final_state(1) << ", " << final_state(2) << std::endl;
    std::cout << "Angular velocity (p, q, r): "
              << final_state(3) << ", " << final_state(4) << ", " << final_state(5) << std::endl;
    std::cout << "Euler angles (phi, theta, psi): "
              << final_state(6)*rad2deg << ", " << final_state(7)*rad2deg << ", " << final_state(8)*rad2deg << std::endl;

    if (!final_state.allFinite()) {
        std::cerr << "\nWarning: final state contains NaN/Inf - simulation diverged." << std::endl;
    }

    rk4Solver.free_results();
    if (loop){
        std::cout<<"Press any key to exit\n";
        std::cin >> dt;
    }
    return 0;
}