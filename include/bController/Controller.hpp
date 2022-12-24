#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

// Eigen Header files
#include <Eigen/Dense>

// Project Header files
#include "FrankaProxy.hpp"
#include "ini.h"
#include "VREPProxy.hpp"

// System Header files
#include <string.h>

enum CTRL_STATE {UNDEFINED_STATE = -1, REQUESTED_STATE, SET_STATE, RUNNING_STATE, STOPPING_STATE, STOPPED_STATE, WAITING_STATE};
/*struct trapezoidalData{
	const double v_max = 0.1;
	const double a_max = 0.1;
};*/

class Controller {

public:

	Controller();

	~Controller();

	void init();

	// Control laws
	// Kinematic 
	Eigen::VectorXd controlLaw_Admittance();

	// Dynamic
	Eigen::VectorXd Controller::controlLaw_Impedance();
	Eigen::VectorXd controlLaw_PIDxGravity();

	/**
	* @brief Exit function
	* Set the exit flag on true to exit mainLoop
	*/
	inline void exit() { this->exit_loop = true; }

	/**
	 * @brief assign the next step of a trapezoidal trajectory
	 * @param t time variable
	 * @param L trajectory length
	 * @return delta steps to compute next position, velocity and acceleration 
	 * 
	*/
	std::vector<double> evalTrapezoidalTrajectory(double t, double L);

	/**
	 * @brief TEMP generate a series of reference 0_T_ee dividing the interval between the initial and desired transformation by the number of points N = duration/time-step
	*/
	void generateTrajectoryOffline();

	/**
	 * @brief At each iteration generate the next reference point of the trajectory, according to the path planning strategy specified inside it
	*/
	void generateTrajectoryOnline();

	/**
	 * @brief initialize Coppelia instance
	*/
	void initCoppelia();
	
	/**
	* @brief Check function
	* Check if the main loop routine must running
	* @return the main loop routine flag
	*/
	inline bool ok() { return !this->exit_loop; }

	void mainLoop();

	/**
	* @brief Update the state of the robot from data acquired from the corresponding proxy
	* @param online: online processing flag (default true)
	*/
	void updateRobot(const bool& online = true);

	// Maybe better in FrankaProxy
	Eigen::VectorXd torqueSaturation(Eigen::VectorXd tau);
	Eigen::Vector7d velocitySaturation(Eigen::Vector7d q_dot);

	/**
	 * @brief Update the reference 0_T_ee for the current iteration
	 * @param counter: for counting the number of iterations
	*/
	void updateT_ref(int counter);

	/**
	* @brief Wait function
	* Waits for the user input requesting the task start
	*/
	void waitForStartRequest();

	// Get functions
	Eigen::Vector3d getP_des();
	Eigen::Matrix3d getR_des();
	Eigen::Vector3d getP_ref();
	Eigen::Matrix3d getR_ref();
	inline Eigen::Matrix4d getT_ref(){ return this->T_ref; };

	// Set functions
	inline void setControlType(bool type_) {this->torqueCtrl = type_; };	
	inline void setRobotProxy(RobotProxy* rp) { this->robotProxy = rp; };
	inline void setRobotInterface(RobotInterface* ri) { this->robot = ri; };
	void setTaskOption(bool position_, bool orientation_, bool tracking_);
	void setT_des();
	inline void setT_init(Eigen::Matrix4d T_){ this->T_init = T_;};

private:

	// Classes
	RobotProxy* robotProxy;						//!< Dynamic instance for RobotProxy class
	RobotInterface* robot;						//!< Dynamic instance of RobotInterface class	
	VREPProxy* coppelia;						//!< Dynamic instance for VREPProxy class

	// Homogeneous transfomations
	Eigen::Matrix4d T_init;						//!< Initial 0_T_ee
	Eigen::Matrix4d T_des;						//!< Desired final homogeneous transformarion 0_T_ee
	Eigen::Matrix4d T_ref;						//!< Reference homogeneous transformation 0_T_ee
	std::vector<Eigen::Matrix4d> Tref_vector;	//!< Reference trajectory as vector of homogeneous transformations from T_init to T_des

	// Utils and modalities
	std::stringstream ctrlRateSS;
	double ctrlDt;								//!< Sample time of the Controller
	double dtAvg;								//!< Average time step among all the measured
	bool exit_loop;								//!< Boolean variable stating if the thread has to be terminated
	bool firstIt;								//!<Check if it is the first iteration (for derivative term)
	bool orientation;							//!< Orientation control
	bool position;								//!< Position control
	int simPort;								//!< Connection port number of the simulator server for the given HLInterface
	bool started;								//!< State if the HLInterface has started	
	int state;									//!< State ID of the current controller
	double time_;								//!< Local storage of timestamp	
	bool torqueCtrl;							//!< Dynamic (torque) or kinematic (velocity) control type
	bool tracking;								//!< To distinguish between tracking (true) and regulation (false)

	// Variables commonly used for controllers
	int jointNum;
	Eigen::Vector6d err, errD, errI;			//!< Pose, derivative, integral error
	Eigen::Vector3d p, pRef;					//!< Current and desired position
	Eigen::Vector6d v, vRef, vPrev;				//!< Current, desired and previous velocity
	Eigen::Vector6d a, aRef;					//!< Current and desired acceleration
	Eigen::Matrix3d R, RRef;					//!< Current and desired orientation matrix (0_T_ee.block(3,3))
	Eigen::Vector6d err_tf, errD_tf; 			//!< Errors in task frame
	Eigen::VectorXd u;							//!< Control law 
	Eigen::VectorXd g;							//!< Gravity vector
	Eigen::MatrixXd J, JT;						//!< Jacobian and its transpose
	Eigen::MatrixXd B;							//!< Inertia matrix
	Eigen::MatrixXd C;							//!< Coriolis matrix

	// For kinematic admittance
	Eigen::Vector7d dqcmd, dqcmd_prev; 			//!< Commanded joints velocity
	Eigen::Vector7d dqcmd_filtered, dqcmd_filtered_prev;
	Eigen::Vector6d fMsr, fMsr_prev; 			//!< Measure force (estimated with residuals)
	Eigen::Vector6d fMsr_filtered, fMsr_filtered_prev;	
	//bool force_trigger;						  //!<Check for the first contact force interaction

};


#endif // CONTROLLER_HPP_