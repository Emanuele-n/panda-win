#ifndef FRANKA_PROXY_HPP_
#define FRANKA_PROXY_HPP_

// Project Header files
#include "RobotProxy.hpp"
#include "Timer.hpp"

// Libfranka Header files
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include "examples_common.h"


class FrankaProxy : public RobotProxy {

public:


	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	* @brief Default contructor of FrankaProxy class
	*
	*/
	FrankaProxy();

	/**
	* @brief Default destroyer of FrankaProxy class
	*
	*/
	~FrankaProxy();

	/**
	 * @brief
	*/
	void moveJoint();

	/**
	* @brief Default init function
	*/
	void init();

	/**
	* @brief Gripper init function
	*/
	void initGripper();

	/**
	* @brief Default run function
	*/
	void mainLoop();

	/**
	* @brief Default clear function
	*/
	void clear();

	/**
	* @brief Callback launch function
	* Launch the control callback function consistently as specified in the libfranka documentation 
	* This implies that no explicit thread is needed to be launched for Franka, since the control loop 
	* is innerly handled by the library
	*/
	//void launchCtrlCallback();

	/**
	* @brief Set function (virtual)
	* Set the current vector of measured joint position
	* @param q: the current vector of measured joint position
	*/
	void setMsrJointPosition(const Eigen::VectorXf& q);

	/**
	* @brief Get function (virtual)
	* Get the current vector of measured joint position
	* @return the current vector of measured joint position
	*/
	Eigen::VectorXf getMsrJointPosition();

	/**
	* @brief Set function (virtual)
	* Set the current vector of measured joint torque
	* @param tau: the current vector of measured joint torque
	*/
	void setMsrJointTorque(const Eigen::VectorXf& tau);

	/**
	* @brief Get function (virtual)
	* Get the current vector of measured joint torque
	* @return the current vector of measured joint torque
	*/
	Eigen::VectorXf getMsrJointTorque();

	/**
	* @brief Set function (virtual)
	* Set the current vector of estimated external joint torque
	* @param tau: the current vector of estimated external joint torque
	*/
	void setEstExtJointTorque(const Eigen::VectorXf& tau);

	/**
	* @brief Get function (virtual)
	* Get the current vector of estimated external joint torque
	* @return the current vector of estimated external joint torque
	*/
	Eigen::VectorXf getEstExtJointTorque();

	/**
	* @brief Set function (virtual)
	* Set the current vector of estimated external Cartesian force
	* @param q: the current vector of estimated external Cartesian force
	*/
	void setEstExtCartForce(const Eigen::Vector6f& f) ;

	/**
	* @brief Get function (virtual)
	* Get the current vector of estimated external Cartesian force
	* @return the current vector of estimated external Cartesian force
	*/
	Eigen::Vector6f getEstExtCartForce();

	/**
	*@brief Get function
	* Get the full state of the robot from the proxy
	* @return the full state of the robot from the proxy
	*/
	/*inline RobotState getRobotState() {
		RobotState rs;
		rs.msrJointPosition = this->getMsrJointPosition();
		rs.msrJointTorque = this->getMsrJointTorque();
		rs.extEstJointTorque = this->getEstExtJointTorque();
		rs.extEstCartForce = this->getEstExtCartForce();
		return rs;
	}//*/

	/**
	* @brief Set Robot commands 
	* Set the input command joint vector
	* @param qcmd: the input vector to be set
	*/
	void setJointPositionCommands(const Eigen::VectorXf& qcmd);

	/**
	* @brief Set Robot commands
	* Set the input command joint torque vector
	* @param qdcmd: the input vector to be set
	*/
	void setJointVelocityCommands(const Eigen::VectorXf& qdcmd);

	/**
	* @brief Set Robot commands
	* Set the input command joint torque vector
	* @param taucmd: the input vector to be set
	*/
	void setJointTorqueCommands(const Eigen::VectorXf& taucmd);

	/**
	* @brief Set Robot commands
	* Set the input command joint torque vector with time stamp
	* @param time: the input time stamp
	* @param taucmd: the input vector to be set
	*/
	inline void setStampedJointTorqueCommands(const double& t, const Eigen::VectorXf& taucmd) {
		this->rs.stamp = t;
		this->rs.cmdJointTorque = taucmd;
	}

	/**
	* @brief Set Joint reference vector (virtual)
	* Set the joint reference vector for autonomous pose regolation
	* @param qcmd: the joint reference vector
	*/
	void setJointRefs(const Eigen::VectorXf& jref);

	/*
	* @brief Get function
	* Get the full robot state from the proxy
	* @return the RobotState structure
	*/
	inline alba::RobotState getRobotState() { return this->rs; }

	/*
	* @brief Thread callback function
	* Callback function specifying the forwarding of the control inputs to be sent to the robot
	*/
	void set_commands_callback_fcn();

	/*
	* @brief Thread callback function
	* Callback function specifying the retrieval of the robot state 
	*/
	void get_state_callback_fcn();

	/**
	* @brief Check function
	* Check if the gripper is mounted on the robot
	* @return true if the gripper is mounted, false otherwise
	*/
	inline bool isGripperMounted() { return this->gripperMounted; }

	/**
	* @brief Set function
	* Set if the gripper is mounted on the robot
	* @param mount_grip: the gripper mounting flag
	*/
	inline void mountGripper(const bool& mount_grip) { this->gripperMounted = mount_grip; }

	/**
	* @brief Set function
	* Set the gains for the inner Cartesian impedance control of the Franka Panda robot
	* @param cartimp: the Cartesian impedance control gains
	*/
	inline void setRobotCartesianImpedance(const std::array<double, 6>& cartimp) { this->panda->setCartesianImpedance(cartimp); }

	/**
	* @brief Set function
	* Set the gains for the inner Cartesian impedance control of the Franka Panda robot
	* @param cartimp: the Cartesian impedance control gains
	*/
	inline void setJointImpedance(const std::array<double, 7>& jointimp) { this->panda->setJointImpedance(jointimp); }

	/**
	* @brief Set function
	* Set the stiffness frame of the Franka Panda robot
	* @param T: the stiffness frame to be set
	*/
	void setStiffnessFrame(const Eigen::Matrix4d& T);

	/**
	* @brief Set function
	* Set enabled flag for the the callback routine for robot commands
	* @param run: the enabled flag
	*/
	inline void setCallbackEnabled(const bool& enable) { this->callback_enabled = enable; }

	/**
	* @brief Get function
	* Get the enabled flag of the callback routine
	* @param run: the enabled flag
	*/
	inline bool isCallbackEnabled() { return this->callback_enabled; }

	/**
	* @brief Set function
	* Set running flag for the the callback routine for robot commands
	* @param run: the running flag
	*/
	inline void setCallbackRunning(const bool& run) { this->callback_running = run; }

	/**
	* @brief Get function
	* Get the running flag of the callback routine
	* @param run: the running flag
	*/
	inline bool isCallbackRunning() { return this->callback_running; }

	// VVVV TEST FUNCTIONS FOR DEBUG, NOT NEEDED IN THE FUTURE VVVV
	/**
	* @brief Get function
	* Retrieve the stringstream storing the control rate (for debug purposes)
	* @return the string stored
	*/
	inline std::string getTrqStampedStr() { return this->cmdTrqStampedSS.str(); }

	/**
	* @brief Set PID Action command
	* Set the PID action contribution of the commanded input
	* @param pid: the PID action
	*/
	void setControlInputPIDAction(const Eigen::Vector6f& pid);

	/**
	* @brief Set PID Action command
	* Set the PID action contribution of the commanded input
	* @param pid: the PID action
	*/
	void setControlError(const Eigen::Vector6f& err);

	/**
	* @brief Set PID Action command
	* Set the PID action contribution of the commanded input
	* @param pid: the PID action
	*/
	void setPdes(const Eigen::Vector3f& pdes);

	inline void setDataNew(const bool& dn) { this->dataNew = dn; }
	inline bool hasNewData() { return this->dataNew; }

	void lockDataNewMutex() { this->dataNewMtx.lock(); }
	void unlockDataNewMutex() { this->dataNewMtx.unlock(); }

	franka::Robot* panda;		//!< Dynamic instance of panda

private:

	
	franka::Gripper* gripper;  //!< Dynamic instance of the gripper

	std::thread set_commands_thread;
	std::thread get_state_thread;

	bool gripperMounted;
	bool callback_enabled;
	bool callback_running;

	bool dataNew;
	std::mutex dataNewMtx;
	std::stringstream cmdTrqStampedSS; //<--- Test variable, not needed in the future

};

#endif // FRANKA_PROXY_HPP_
