#ifndef ROBOT_PROXY_HPP_
#define ROBOT_PROXY_HPP_

// Project Header files
#include "ExtSystemProxy.hpp"
#include "utils.hpp"

// Standard Header files
#include <mutex>

enum ROBOT_CONTROL_TYPE {KINEMATIC_CONTROL, TORQUE_CONTROL, IDLE_CONTROL};


namespace alba {
	struct RobotState {

		double stamp;
		bool initialized;
		Eigen::VectorXf msrJointPosition;
		Eigen::VectorXf msrJointVelocity;
		Eigen::VectorXf msrJointTorque;
		Eigen::VectorXf cmdJointVelocity;
		Eigen::VectorXf cmdJointTorque;
		Eigen::VectorXf extEstJointTorque;
		Eigen::Vector6f extEstCartForce;
		Eigen::Matrix4f T0ee;
		Eigen::Matrix4f T0ee_des;
		Eigen::Vector6f PIDaction;
		Eigen::Vector3f pdes;
		Eigen::Vector6f ctrlErr;
		Eigen::VectorXf	g_lib;	// Gravity vector taken from the proprietary robot library
		Eigen::VectorXf	c_lib;	// Coriolis vector taken from the proprietary robot library
		Eigen::MatrixXf	m_lib;  // Mass matrix taken from the proprietary robot library
		Eigen::VectorXf	fric_lib;	// Friction vector taken from the proprietary robot library
		Eigen::MatrixXf	Jac_lib;	// Jacobian matrix taken from the proprietary robot library

	};
};

class RobotProxy : public ExtSystemProxy {

public:

	/**
	* @brief Default contructor of RobotProxy class
	*
	*/
	RobotProxy() {
	
		this->availability(false);
		this->setRunning(false);
		this->t_ = 0.0;
		this->cmdReady = false;

		// Default control type is kinematic (this option may be overriden later)
		this->ctrlLevel = ROBOT_CONTROL_TYPE::KINEMATIC_CONTROL;
	}

	/**
	* @brief Default destroyer of RobotProxy class
	*
	*/
	~RobotProxy() {}

	/**
	* @brief Set function
	* Set the Degrees-of-Freedom of the robot system
	* @param num: the Degrees-of-Freedom of the robot system
	*/
	inline void setJointDOFs(const int& num) { this->jointDOFs = num; }

	/**
	* @brief Set function
	* Set the control type of the robot system
	* @param num: the control type of the robot system
	*/
	inline void setControlLevel(const int& lvl) { this->ctrlLevel = lvl; }

	/**
	* @brief Get function
	* Get the control type of the robot system
	* @return the control type of the robot system
	*/
	inline int getControlType() { return this->ctrlLevel; }

	/**
	* @brief Set function (virtual)
	* Set the current vector of measured joint position
	* @param q: the current vector of measured joint position
	*/
	virtual void setMsrJointPosition(const Eigen::VectorXf& q) = 0;

	/**
	* @brief Get function (virtual)
	* Get the current vector of measured joint position
	* @return the current vector of measured joint position
	*/
	virtual Eigen::VectorXf getMsrJointPosition() = 0;

	/**
	* @brief Set function (virtual)
	* Set the current vector of measured joint torque
	* @param tau: the current vector of measured joint torque
	*/
	virtual void setMsrJointTorque(const Eigen::VectorXf& tau) = 0;

	/**
	* @brief Get function (virtual)
	* Get the current vector of measured joint torque
	* @return the current vector of measured joint torque
	*/
	virtual Eigen::VectorXf getMsrJointTorque() = 0;

	/**
	* @brief Set function (virtual)
	* Set the current vector of estimated external joint torque
	* @param tau: the current vector of estimated external joint torque
	*/
	virtual void setEstExtJointTorque(const Eigen::VectorXf& tau) = 0;

	/**
	* @brief Get function (virtual)
	* Get the current vector of estimated external joint torque
	* @return the current vector of estimated external joint torque
	*/
	virtual Eigen::VectorXf getEstExtJointTorque() = 0;

	/**
	* @brief Set function (virtual)
	* Set the current vector of estimated external Cartesian force
	* @param q: the current vector of estimated external Cartesian force
	*/
	virtual void setEstExtCartForce(const Eigen::Vector6f& f) = 0;

	/**
	* @brief Get function (virtual)
	* Get the current vector of estimated external Cartesian force
	* @return the current vector of estimated external Cartesian force
	*/
	virtual Eigen::Vector6f getEstExtCartForce() = 0;

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
	* @brief Set Robot commands (virtual)
	* Set the input command joint vector
	* @param qcmd: the input vector to be set
	*/
	virtual void setJointPositionCommands(const Eigen::VectorXf& qcmd) = 0;

	/**
	* @brief Set Robot commands (virtual)
	* Set the input command joint vector
	* @param qdcmd: the input vector to be set
	*/
	virtual void setJointVelocityCommands(const Eigen::VectorXf& qdcmd) = 0;

	/**
	* @brief Set Robot commands (virtual)
	* Set the input command joint torque vector
	* @param taucmd: the input vector to be set
	*/
	virtual void setJointTorqueCommands(const Eigen::VectorXf& taucmd) = 0;

	/**
	* @brief Set Robot commands
	* Set the input command joint torque vector with time stamp
	* @param time: the input time stamp
	* @param taucmd: the input vector to be set
	*/
	virtual void setStampedJointTorqueCommands(const double& t, const Eigen::VectorXf& taucmd) = 0;

	/**
	* @brief Set Joint reference vector (virtual)
	* Set the joint reference vector for autonomous pose regolation 
	* @param qcmd: the joint reference vector
	*/
	virtual void setJointRefs(const Eigen::VectorXf& jref) = 0;
	

	inline void setSafeCmdJointTorques(const Eigen::VectorXf& taucmd) {
		{
			std::lock_guard<std::mutex> lock(cmdMtx);
			this->rs.cmdJointTorque = taucmd;
		}
	}

	inline Eigen::VectorXf& getSafeCmdJointTorques() {

		Eigen::VectorXf cmd;

		{
			std::unique_lock<std::mutex> lock(this->cmdMtx);
			cmd = this->rs.cmdJointTorque;
		}
		return cmd;
	}

	/*
	* @brief Get function
	* Get the full robot state from the proxy
	* @return the RobotState structure
	*/
	virtual alba::RobotState getRobotState() = 0;

	/**
	* @brief Set function
	* Set the desired Cartesian pose homogensous matrix
	* @param T: the desired Cartesian pose homogenous matrix to be set
	*/
	inline void setTeeDes(const Eigen::Matrix4f& T) { this->rs.T0ee_des = T; }

	/**
	* @brief Get function
	* Get the desired Cartesian pose homogensous matrix
	* @return the desired Cartesian pose homogenous matrix to be set
	*/
	inline Eigen::Matrix4f getTeeDes() { return this->rs.T0ee_des; }



protected:

	int jointDOFs;							//!< Degrees-of-Freedom of the robot system
	double t_;								//!< time stamp

	alba::RobotState rs;		//!< RobotState structure

	bool cmdReady;
	std::mutex cmdMtx;
	std::condition_variable cvInput;

	int ctrlLevel;				//!< Flag specifying the type of the controller to be used (kinematic or torque control)


};




#endif // ROBOT_PROXY_HPP_
