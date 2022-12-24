// Project header files
#include "Task.hpp"

Task::Task(){
    this->ctrl = nullptr;
    this->robot = nullptr;
    this->robotProxy = nullptr;
}

Task::~Task(){}

void Task::mainLoop() {

    // Loop unitl a key is pressed
    while (!_kbhit()){      
    }

    // Set robot running on false to stop thread
    this->robotProxy->setRunning(false);
    dynamic_cast<FrankaProxy*>(robotProxy)->setCallbackEnabled(false);

    // Exit controller loop
    this->ctrl->exit();
}

void Task::setup(bool tracking, bool torque, bool position, bool orientation){

    // Init robot
    std::cout << "\n[INIT ROBOT]\nCreating the Franka robot instances ... " << std::endl;
    this->robotProxy = new FrankaProxy;
    this->robotProxy->init();
    this->robot = new PANDARobot("panda");

    // Set control level at robotProxy
    if (torque == true ){ this->robotProxy->setControlLevel(ROBOT_CONTROL_TYPE::TORQUE_CONTROL); }
    else{ this->robotProxy->setControlLevel(ROBOT_CONTROL_TYPE::KINEMATIC_CONTROL); }

    // Init controller
    std::cout << "\n[INIT CONTROLLER]\nCreating the controller instances ... " << std::endl;
    this->ctrl = new Controller;
    this->ctrl->setRobotProxy(this->robotProxy);
    this->ctrl->setRobotInterface(this->robot);
    this->ctrl->init();

    // Load desired 0_T_ee
    this->ctrl->setT_des(); 

    // Set initial 0_T_ee
    franka::RobotState current_state = this->robotProxy->panda->readOnce();
    Eigen::Map<const Eigen::Matrix4d> T0ee_init(current_state.O_T_EE.data());
    this->ctrl->setT_init(T0ee_init);

    // Set task options
    this->ctrl->setTaskOption(position, orientation, tracking);

    // Generate trajectory offline for regulation or tracking (deprecated, better to generate online in Controller::MainLoop)
    //this->ctrl->generateTrajectoryOffline();

    // Set control level at Controller
    this->ctrl->setControlType(torque);
}

void Task::start(){

    // TODO: Coppelia simulation without real robot 
    /*if (this->simulation == true){
        std::thread simThread;
        simThread = std::thread(&Task::simLoop, this);

        simThread.join();
    }
    else{
    */

    // Launch threads
    std::thread controlThread, robotThread, mainThread;

    std::cout << "\n[THREADS]" << std::endl; 
    // Robot
    robotThread = std::thread(&FrankaProxy::mainLoop, this->robotProxy);

    // Controller
    controlThread = std::thread(&Controller::mainLoop, this->ctrl);

    // Main loop
    mainThread = std::thread(&Task::mainLoop, this);

    // Join threads
    robotThread.join();
    controlThread.join();
    mainThread.join();

    //}
}

