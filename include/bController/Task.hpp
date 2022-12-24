#ifndef TASK_HPP_
#define TASK_HPP_

// Project Header files
#include "Controller.hpp"
#include "FrankaProxy.hpp"
#include "PANDARobot.hpp"
#include "RobotInterface.hpp"
#include "RobotProxy.hpp"

// System Header files
#include <conio.h>
#include <string>
#include <thread>


class Task{

public:

	Task();

	~Task();

	void mainLoop();

	/**
	 * @brief Prepare data for the desired task. initialize robot, controller and reference trajectory
	*/
	void setup(bool tracking, bool torque, bool position, bool orientation);

	/**
	 * @brief Start the desired task
	*/
	void start();

protected:

	FrankaProxy *robotProxy;				//!< Dynamic instance of FrankaProxy class
	Controller *ctrl;						//!< Dynamic instance of Controller class
	RobotInterface* robot;					//!< Dynamic instance of RobotInterface class
};

#endif //TASK_HPP_