#include "Task.hpp"
#include "CoppeliaOutLoop.hpp"

int main(int argc, const char** argv){

    std::cout << "\nPANDA WIN SOFTWARE\nTest Franka Emika Panda on Windows " << std::endl;

    // Initialize options
    bool tracking = true;
    bool torque = true;
    bool position = true;
    bool orientation = true;
    bool simulation = false; // TODO

    // Check parameters
    //std::cout << "argc " << argc << std::endl;
    if (argc > 1){
        for (auto i = int{0}; i < argc; ++i) {
            if (std::string(argv[i]) == "-p"){ orientation = false;}
            if (std::string(argv[i]) == "-o"){ position = false;}
            if (std::string(argv[i]) == "-r"){ tracking = false;}            
            if (std::string(argv[i]) == "-k"){ torque = false;}
            // if (std::string(argv[i]) == "-sim"){ simulation = true;}
            if (std::string(argv[i]) == "-t"){ tracking = true;}
            if (std::string(argv[i]) == "-d"){ torque = true;}
            if (std::string(argv[i]) == "--help"){ 
                printHelp();
                return 0;
            }
        }
    }
    
    // Print options
    printporks(position, orientation, tracking, torque, simulation);

    int actionInput = -1;

    while (actionInput != 0){

        // Ask the user to select the desired action
		actionInput = Menu();

        if (actionInput == 1){

            std::cout << "Be sure LED are BLUE and press enter to go to initial configuration" << std::endl;
            system("pause");

            // Instantiate FrankaProxy and go to initial configuration
            FrankaProxy *robot = new FrankaProxy;
            robot->moveJoint();

            // Clean up
            std::cout << "Task completed" << std::endl;
            delete robot;
        }

        if (actionInput == 2) {

            std::cout << "Be sure LED are BLUE and press enter to start Cartesian task" << std::endl;
            system("pause");

            // Start task
            Task* task = new Task;
            task->setup(tracking, torque, position, orientation);
            task->start();

            // Clean up
            std::cout << "Task completed" << std::endl;
            delete task;
        }

        if (actionInput == 3){

            // Initialize Coppelia to track robot out of control loop
            CoppeliaOutLoop* csout = new CoppeliaOutLoop;
            csout->init();
            std::thread CoppeliaThread;
            csout->startWhiteTrack();
            CoppeliaThread = std::thread(&CoppeliaOutLoop::mainLoop, csout);

            std::cout << "Be sure LED are white and move the robot to the desired position.\nPress any key to get transformation" << std::endl;
            system("pause");

            // Stop white led tracking
            csout->stopWhiteTrack();
            CoppeliaThread.join();

            // Init robot
            FrankaProxy *robot = new FrankaProxy;
            robot->panda = new franka::Robot("172.16.0.2");

            // Read current 0_T_ee
            franka::RobotState current_state = robot->panda->readOnce();
            Eigen::Map<const Eigen::Matrix4d> T0ee_init(current_state.O_T_EE.data());
            std::cout << "0_T_ee = \n " << T0ee_init << std::endl;

            // Ask if save 0_T_ee
            char save;
            std::cout << "\nSave transformation? (y/n)" << std::endl;
            std::cin >> save;
            if (save == 'y'){                
                // Save current transformation
                saveT(current_state.O_T_EE);
                std::cout << "\nTransformation saved" << std::endl;
            }
            else{
                std::cout << "\nTransformation not saved" << std::endl;
            }
            
            // Clean up
            std::cout << "Task completed" << std::endl;
            delete csout;
            delete robot;
        }
   
    }
    std::cout << "Leaving..." << std::endl;
    return 0;
}