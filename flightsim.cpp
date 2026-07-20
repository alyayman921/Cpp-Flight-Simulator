#include "src/flightsim.h"

int main(int argc, char* argv[]) {

    // Read aircraft data
    raw_data raw = readxlsx(filename);
    aircraft_data c5a = sorting(raw);

    // Input handling
    if (argc >= 2){
        std::string arg = argv[1];
        if (arg == "--help") {
            std::cout<< "\nFlight Simulator for Lockheed Martin's C5A Aircraft\n";
            std::cout<< "Run the Simulator with the supported input arguments like \"./FlightSim --arg\"\n\n";
            std::cout<<"Arguments    Usage\n";
            std::cout<<"---------    ---------------------------------------------------------\n";
            std::cout<<"--help       print this message\n";
            std::cout<<"--loop       prevent the program from exiting after solving\n";
            std::cout<<"--manual     read the control commands from the textfile controls.txt\n";
            std::cout<<"--pitch      overrides altitude loop straight to pitch control\n";
            return 0;
        }
        for(int i=0;i<argc;i++){
            arg=argv[i];
            if (arg == "--loop") {
                std::cout<<"Prevent from Exit : True\n";
                loop = true;
            }
            if (arg == "--manual") {
                std::cout<<"Applying Control actions directly from text file\n";
                Autopiloted = false;
            }
            if (arg == "--pitch") {
                std::cout<<"Skipping altitude loop\n";
                alt_override = true;
            }

        }
    }
        if(alt_override){
        std::cout<<"Pitch Autopilot change (Degrees): ";
        std::cin >> set_pitch; set_pitch=set_pitch*deg2rad+c5a.euler0(1);
        }else{
        std::cout<<"Altitude Change (ft): ";
        std::cin >> set_alt ;set_alt -= c5a.z0;
        }
        std::cout<<"Velocity Autopilot change (ft/s): ";
        std::cin >> set_vel ;set_vel += c5a.V0(0);

    // Variables to be read from controls file
    double dt = 0.01;      // Default values
    double tfinal = 100.0;

    if (!readControlsFromFile("controls.txt", Controls, dt, tfinal)) {
        std::cerr << "Failed to read controls. Using defaults..." << std::endl;
        Controls << 0.0, 0.0, 1000.0, 0.05;
        dt = 0.01;
        tfinal = 100.0;
    }   
    if (!Autopiloted){std::cout << "Controls loaded: " << Controls.transpose() << std::endl;}
    std::cout << "Timestep: " << dt << " s" << std::endl;
    std::cout << "Final time: " << tfinal << " s" << std::endl;
    std::cout << "Number of steps: " << (int)(tfinal / dt) << std::endl;

    // Initialize the controller and keep the control vector the same across files
    controller c(&Controls, &str_h,dt,set_pitch, set_vel,set_alt,set_heading,alt_override,Autopiloted);
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
    Eigen::Matrix<double, 9, 1>* results = rk4Solver.rk4_solver(RBD,c, str_h, initial_state);
    int N_steps = (int)(tfinal / dt);
    Eigen::Matrix<double, 9, 1> final_state = results[N_steps];

    std::cout << "\n=== Final State (t=" << tfinal << "s) ===" << std::endl;
    std::cout << "Velocity ft/s (v_x, v_y, v_z): "
              << final_state(0) << ", " << final_state(1) << ", " << final_state(2) << std::endl;
    //std::cout << "Angular velocity rad/s (p, q, r): "
    //          << final_state(3) << ", " << final_state(4) << ", " << final_state(5) << std::endl;
    std::cout << "Euler angles in Degrees (phi, theta, psi): "
              << (float)final_state(6)*rad2deg << ", " << (float)final_state(7)*rad2deg << ", " << (float)final_state(8)*rad2deg << std::endl;
    std::cout << "Final Altitude: "<<str_h.h<<"\n"; 
    if (!final_state.allFinite()) {
        std::cerr << "\nWarning: final state contains NaN/Inf - simulation diverged." << std::endl;
    }

    rk4Solver.free_results();
    if (loop){
        std::cout<<"Press Ctrl+C to Exit\n";
        std::cin >> dt;
    }
    return 0;
}