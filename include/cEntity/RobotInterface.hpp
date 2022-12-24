#ifndef ROBOT_INTERFACE_HPP_
#define ROBOT_INTERFACE_HPP_

// System Header files
#include <vector>
#include <mutex>

// Eigen Header files
#define EIGEN_RUNTIME_NO_MALLOC
#include <Eigen\Dense>

// Project Header files
#include "utils.hpp"

struct RobotDynParams {

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::MatrixXd B;		//!< Inertia matrix
	Eigen::MatrixXd C;		//!< Coriolis and centrifugal terms matrix
	Eigen::MatrixXd cvec;	//!< Coriolis and centrifugal terms vector (already multiplied by qdot)
	Eigen::VectorXd g;		//!< Gravity vector
	Eigen::VectorXd f;		//!< Friction vector
	Eigen::VectorXd tau;	//!< Torque vector

};

struct RobotDHParams {
	Eigen::VectorXd d;
	Eigen::VectorXd a;
	Eigen::VectorXd alpha;
};

class RobotInterface{ //: public Instrument {

public:

	/** PLEASE READ THIS CAREFULLY **/
	/* When we define a class containing fixed-size Eigen structures,
	 * and we later instantiate such class dynamically, it occurs an issue 
	 * that may result in undesired and unexpected program crash if not 
	 * properly handled, due to ACCESS VIOLATIONS.
	 * To prevent this, every class containing fixed-size Eigen variables
	 * must add the line below in the public part of the class definition, 
	 * in order to overload the new operator and explicitly handle alignment
	 * problems in the variable definitions. If you have a polymorphic structure
	 * among your class, place this macro in the Base class, like here. 
	 */
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
		
	/**
	* @brief Default constructor of the Instrument class
	*/
	RobotInterface();

	/**
	* @brief Constructor of the RobotInterface class with name argument
	* @param name_ the name of the Instrument
	*/
	RobotInterface(const std::string& name_);

	/**
	* @brief Constructor of the RobotInterface class with name argument
	* @param qnum the dofs of the robot
	*/
	RobotInterface(const int& qnum);

	/**
	* @brief Default destroyer of the RobotInterface class
	*/
	~RobotInterface() {}

	/**
	* @brief Init function (virtual)
	* Check the input pair of comment+value to assign the corresponding parameter
	* The function assumes that configFile has been previously set
	*/
	virtual void loadParamsFromConfigFile(const std::string& comment, const std::string& value) = 0;

	/**
	* @brief Set function
	* Set the number of joints of the robot
	* @param num: the number of joints of the robot
	*/
	inline void setJointNum(const int& num) { this->jointNum = num; }

	/**
	* @brief Get function
	* Get the number of joints of the robot
	* @return the number of joints of the robot
	*/
	inline int getJointNum() { return this->jointNum ; }

	/**
	* @brief Init function
	* Init all the dynamic data of the class with the known value of dofs of the 
	*/
	void initData();

	/**
	* @brief Forward kinematics function (virtual)
	* Compute the forward kinematic of the robot, based on the current set value of this->jointPosition
	* Internally set the array of homogeneous transformation matrices, i.e., this->Tbli
	*/
	virtual void computeKinematics() = 0;

	/**
	* @brief Forward differential kinematics function
	* Compute the forward differential kinematic of the robot, by evaluating the Jacobian matrix on the current set value of this->jointPosition
	* Internally set the array of homogeneous transformation matrices, i.e., this->Tbli
	*/
	void computeDiffKinematics();

	/**
	* @brief Jacobian function (virtual)
	* Compute the linear Jacobian matrix for the given chosen link (end-effector by default).
	* @param link the link of which the Jacobian matrix has to be computed
	* @return the requested Jacobian matrix
	*/
	virtual Eigen::MatrixXd computeLinearJacobian(const int& link) = 0;

	/**
	* @brief Jacobian function (virtual)
	* Compute the angular Jacobian matrix for the given chosen link (end-effector by default).
	* @param link the link of which the Jacobian matrix has to be computed
	* @return the requested Jacobian matrix
	*/
	virtual Eigen::MatrixXd computeAngularJacobian(const int& link) = 0;

	/**
	* @brief Jacobian function
	* Compute the full Jacobian matrix for the given chosen link (end-effector by default).
	* @param link the link of which the Jacobian matrix has to be computed
	* @return the requested Jacobian matrix
	*/
	Eigen::MatrixXd computeFullJacobian(const int& link);

	/**
	* @brief Forward dynamics function
	* Compute the dynamic model of the robot, based on the current set value of joint position, velocity and torques,
	* along with the known dynamic parameters of the model
	*/
	virtual void computeDynamics() = 0;

	/**
	* @brief Update function
	* Update the full state of the robot, given the input joint positions, velocities and torques
	* Internally compute forward kinematics, differential kinematics, dynamics and residual vectors
	*/
	void updateRobotState(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, const Eigen::VectorXd& tau, const bool& online);

	/**
	* @brief Force reconstruction function
	* Compute the Cartesian force vector from the given joint torque vector, through the static relationship
	* @param tau: the joint torque vector
	* @return the corresponding Cartesian force vector
	*/
	Eigen::Vector6d staticForceReconstruction(const Eigen::VectorXd& tau);

	/**
	* @brief Residual calculation function
	* COmpute the full residual vector from the known joint velocity qdot and the applied torques trq
	* Set internally the residual vector
	* @param q the joint position vector
	* @param qdot the joint velocity vector
	* @param teq the joint applied torques
	* @param gain the gain for the computation of the residual (default = 5.0)
	*/
	void computeResidualFull(const Eigen::VectorXd& q, const Eigen::VectorXd& q_dot, const Eigen::VectorXd& trq);

	/**
	* @brief Init function
	* Initialize the generalized momentum vector
	* Internally fills the variable p0
	*/
	inline void initializeGeneralizedMomentum() { this->p0 = this->robDynPars.B * this->jointMsrVelocity__; }

	/**
	* @brief Get function
	* Get the struct containing the model dynamic parameters
	* @return the dynamic parameters
	*/
	inline RobotDynParams getModelDynParams() { return this->robDynPars; }

	/**
	* @brief Get function
	* Get the struct containing the model dynamic parameters
	* @return the dynamic parameters
	*/
	inline RobotDynParams setModelDynParams(const RobotDynParams& rdp) { this->robDynPars = rdp; }

	/**
	* @brief Add an EndEffector to the robot
	* @param ee the EndEffector object to be added
	*/
	/*inline void addEndEffector(const EndEffector& ee) { this->tools.push_back(ee); }*/

	/**
	* @brief Check if at least an EndEffector is added to the robot
	* @return true if at least an EndEffector is added to the robot
	*/
	/*inline bool hasEndEffector() { return this->tools.size() > 0; }*/

	/**
	* @brief Get function
	* Get the i-th tool attached to the robot --> the last tool is the End-Effector of the robot
	*@param toolIdx: position of the tool in the tools array
	*/
	/*inline EndEffector getEndEffector(int toolIdx) { return this->tools[toolIdx]; }*/

	/**
	* @brief Set function
	* Set the sample time of the robot (for KUKA it has to be always = 0.005)
	* @param dt: the sample time to be set
	*/
	inline void setSampleTime(const float& dt) { this->dt__ = dt; }

	/**
	* @brief Get function
	* Get the sample time of the robot (for KUKA it has to be always = 0.005)
	* @return the sample time
	*/
	inline float getSampleTime() { return this->dt__; }

	/**
	* @brief Set function
	* Set the timstamp associated to the robot data
	* @param t: the timestamp
	*/
	inline void setDataTimestamp(const float& t) { this->timestamp__ = t; }
	
	/**
	* @brief Get function
	* Get the timstamp associated to the robot data
	* @return the timestamp
	*/
	inline float getDataTimestamp() { return this->timestamp__; }

	/**
	* @brief Reset data
	* Reset the robot data
	*/
	virtual void resetDynParams() = 0;

	/**
	* @brief Set function
	* Set the measured joint position vector of the PANDA robot
	* @param q the measured joint position vector to be set
	*/
	inline void setMsrJointPosition(const Eigen::VectorXd& q) { this->jointMsrPosition__ = q; }

	/**
	* @brief Set function
	* Set the commanded joint position vector of the robot
	* @param q the commanded joint position vector to be set
	*/
	inline void setCmdJointPosition(const Eigen::VectorXd& q) { this->jointCmdPosition__ = q; };

	/**
	* @brief Set function
	* Set the measured joint velocity vector of the robot
	* @param qd the measured joint velocity vector to be set
	*/
	inline void setMsrJointVelocity(const Eigen::VectorXd& qdot) { this->jointMsrVelocity__ = qdot; }

	/**
	* @brief Set function
	* Set the commandede joint velocity vector of the robot
	* @param qd the commanded joint velocity vector to be set
	*/
	inline void setCmdJointVelocity(const Eigen::VectorXd& qdot) { this->jointCmdVelocity__ = qdot; }

	/**
	* @brief Set function
	* Set the commanded joint torque vector of the robot
	* @param qd the commanded joint torque vector to be set
	*/
	inline void setCmdJointTorques(const Eigen::VectorXd& taucmd) { this->jointCmdTorques__ = taucmd; }

	/**
	* @brief Set function
	* Set the commanded joint torque vector of the robot
	* @param qd the commanded joint torque vector to be set
	*/
	inline void setEstJointDeltaTorquesCoeffs(const Eigen::VectorXd& delta_tau) { this->jointEstDeltaTorquesCoeffs__ = delta_tau; }

	/**
	* @brief Set function
	* Set the measured joint acceleration vector of the robot
	* @param qd the measured joint acceleration vector to be set
	*/
	inline void setMsrJointAcceleration(const Eigen::VectorXd& qdotdot) { this->jointAcceleration__ = qdotdot; }

	/**
	* @brief Set function
	* Set the measured joint torques vector of the robot
	* @param qd the measured joint torques vector to be set
	*/
	inline void setMsrJointTorques(const Eigen::VectorXd& tau) { this->jointMsrTorques__ = tau; }

	/**
	* @brief Get function
	* Get the measured joint torques vector of the robot
	* @return the measured joint torques vector to be set
	*/
	inline Eigen::VectorXd getMsrJointTorques() { return this->jointMsrTorques__; }

	/**
	* @brief Set function
	* Set the link length of the robot
	* @param ll: the link length of the robot
	*/
	inline void setLinkLengths(const Eigen::VectorXd& l) { this->linkLengths__ = l; }

	/**
	* @brief Get function
	* Get the measured joint position vector of the robot
	* @return the measured joint position vector to be set
	*/
	inline Eigen::VectorXd getMsrJointPosition() { return this->jointMsrPosition__; }

	/**
	* @brief Get function
	* Get the commanded joint position vector of the robot
	* @return  the commanded joint position vector to be set
	*/
	inline Eigen::VectorXd getCmdJointPosition() { return this->jointCmdPosition__; }

	/**
	* @brief Get function
	* Get the measured joint velocity vector of the robot
	* @return the measured joint velocity vector to be set
	*/
	inline Eigen::VectorXd getMsrJointVelocity() { return this->jointMsrVelocity__; }

	/**
	* @brief Get function
	* Get the commandede joint velocity vector of the robot
	* @return the commanded joint velocity vector to be set
	*/
	inline Eigen::VectorXd getCmdJointVelocity() { return this->jointCmdVelocity__; }

	/**
	* @brief Set function
	* Set the commanded joint torque vector of the robot
	* @param qd the commanded joint torque vector to be set
	*/
	inline Eigen::VectorXd getCmdJointTorques() { return this->jointCmdTorques__; }

	/**
	* @brief Set function
	* Set the the estimated joint delta torque vector of the robot
	* @return the estimated joint delta torque vector 
	*/
	inline Eigen::VectorXd getEstJointDeltaTorquesCoeffs() { return this->jointEstDeltaTorquesCoeffs__; }

	/**
	* @brief Set function 
	* Set the 4x4 homogeneous matrix of the EE pose wrt the base frame
	* @param T: the 4x4 homogeneous matrix
	*/
	inline void setTbee(const Eigen::Matrix4d& T) { this->Tbee__ = T; }
	
	/**
	* @brief Get function 
	* Get the 4x4 homogeneous matrix of the EE pose wrt the base frame
	* @return the 4x4 homogeneous matrix
	*/
	inline Eigen::Matrix4d getTbee() { return this->Tbee__; }

	/**
	* @brief Set function 
	* Set the Jacobian matrix related to the velocity of the EE wrt the base frame
	* @param T: the Jacobian matrix
	*/
	inline void setJacobian(const Eigen::MatrixXd& J) { this->Jbee__ = J; }

	/**
	* @brief Get function
	* Get the Jacobian matrix related to the velocity of the EE wrt the base frame
	* @return the Jacobian matrix
	*/
	inline Eigen::MatrixXd getJacobian() { return this->Jbee__; }

	/**
	* @brief Pseudo-inverse compuation function
	* Compute and return the nx6 pseudo-inverse matrix of the current Jacobian
	* @return the nx6 pseudo-inverse matrix of the current Jacobian
	*/
	Eigen::MatrixXd getJPinv();

	/**
	* @brief Set function 
	* Set the Cartesian velocity vector of the EE velocity wrt the base frame
	* @param v: the Cartesian velocity vector
	*/
	inline void setEEVelocity(const Eigen::Vector6d& v) { this->vbee__ = v; }

	/**
	* @brief Get function
	* Compute and return the 6D velocity of the end-effector
	* @return the 6D velocity vector of the end-effector
	*/
	Eigen::Vector6d getEEVelocity() { return this->vbee__; }

	/**
	* @brief Get function
	* Get the transformation matrix Trpy
	*/
	inline Eigen::Matrix3d getTrpy() { return this->Trpy; }

	/**
	* @brief Get function
	* Get the transformation matrix TeeOffset
	*/
	inline Eigen::Matrix4d getTeeOffset() { return this->TeeOffset; }

	/**
	* @brief Set function
	* Set the transformation matrix TeeOffset
	*/
	inline void setTeeOffset(Eigen::Matrix4d TeeOffset) { this->TeeOffset = TeeOffset; }

	/**
	* @brief Get function
	* Get the Cartesian Force vector at the end-effector wrt base frame, reconstructed from FRI residual
	* @return the Cartesian force vector
	*/
	inline void setMsrForce(const Eigen::Vector6d& fMsr) { this->fMsr = fMsr; }

	/**
	* @brief Get function
	* Get the Cartesian Force vector at the end-effector wrt base frame, reconstructed from FRI residual
	* @return the Cartesian force vector
	*/
	inline Eigen::Vector6d getMsrForce() { return this->fMsr; }

	/**
	* @brief Set function
	* Set the residual vector computed by this class, as estimation of the external torques applied on the model
	* @param r: the residual vecotr computed by this class
	*/
	inline void setResidualVector(const Eigen::VectorXd& r) {

		{
			std::lock_guard<std::mutex> lock(resMtx);
			this->res__ = r;
		}
	}
	
	/**
	* @brief Get function
	* Get the Residual vector of the manipulator
	* @return the residual vector
	*/
	inline Eigen::VectorXd getResidualVector() {

		{
			std::lock_guard<std::mutex> lock(resMtx);
			return this->res__;
		}
	}

	/**
	* @brief Check function
	* Verifies if the offset has to be taken into account for the computation of the residual vectors
	* @return true if the offset is accounted, false otherwise
	*/
	inline bool applyResidualOffset() { return this->withResOffset; }

	/**
	* @brief Get function
	* Get the residual gain vector
	* @return the residual gain vector
	*/
	inline Eigen::MatrixXd getResidualGain() { return this->resGain; }

	/**
	* @brief Set function
	* Set the residual gain vector
	*/
	inline void setResidualGain(Eigen::MatrixXd& resGain) { this->resGain = resGain; }

	/**
	* @brief Set function
	* Set the robot pose in the world frame
	*/
	inline void setTwb(Eigen::Matrix4d& Twb) { this->Twb = Twb; }

	/**
	* @brief Get function
	* Get the robot pose in the world frame
	* @return the robot pose in the world frame
	*/
	inline Eigen::Matrix4d getTwb() { return this->Twb; }

	/**
	* @brief Set function 
	* Set the Residual offset vector of the manipulator
	* @param r: the residual vector offset
	*/
	void setResidualVectorOffset(const Eigen::VectorXd& r) { this->resOffset__ = r; }

	/**
	* @brief Get function (virtual)
	* Get the Residual offset vector of the manipulator
	* @return the residual vector offset
	*/
	Eigen::VectorXd getResidualVectorOffset() { return this->resOffset__; }

	/**
	* @brief Get function
	* Get the Cartesian Force vector at the end-effector wrt base frame, reconstructed from residual
	* @return the Cartesian force vector
	*/
	inline Eigen::Vector6d getExtContactForce() { return this->extFbee__; }

	/**
	* @brief Get function
	* Retrieves the position of the i-th link
	* @return the i-th link position vector
	*/
	inline Eigen::Vector3d getLinkPosition(const int& link) { return this->Tbli__[link].block<3, 1>(0, 3); }

	/**
	* @brief Get function
	* Retrieves the rotation matrix of the i-th link
	* @return the i-th link orientation matrix
	*/
	inline Eigen::Matrix3d getLinkRotMat(const int& link) { return this->Tbli__[link].topLeftCorner(3, 3); }

	/**
	* @brief Get function
	* Retrieves the homogeneous transformation matrix of the i-th link
	* @return the i-th link transformation matrix
	*/
	inline Eigen::Matrix4d getLinkTransform(const int& link) { return this->Tbli__[link]; }

	/**
	* @brief Get function
	* Retrieves the position of the End-Effector tip (accounting also the optional EndEffector objects attached)
	* @return the tip position vector
	*/
	inline Eigen::Vector3d getEEPosition() { return this->getTbee().col(3).head(3); }

	/**
	* @brief Get function
	* Retrieves the rotation matrix of the End-Effector tip (accounting also the optional EndEffector objects attached)
	* @return the tip position vector
	*/
	inline Eigen::Matrix3d getEERotMat() { return getTbee().topLeftCorner<3, 3>(3, 3);}

	/**
	* @brief Get function
	* Retrieves the current Euler angles the End-Effector tip (accounting also the optional EndEffector objects attached)
	* @return the Euler angles vector (here convention ZYX = RPY)
	*/
	inline Eigen::Vector3d getEEEulAng() { return this->eulAng_ee; }

	/**
	* @brief Set function
	* Set if the dynamic model to be computed has to account the dynamic parameters of the tools
	* @param true if the dynamic model taks into account the tool, false otherwise
	*/
	inline void dynModelWithTool(const bool& withTool) { this->includeToolDynParams = withTool; }

	/**
	* @brief Check function
	* Check if the dynamic model to be computed has to account the dynamic parameters of the tools
	* @return true if the dynamic model taks into account the tool, false otherwise
	*/
	inline bool isDynModelWithTool() { return this->includeToolDynParams; }

	/**
	* @brief Set function
	* Set if the dynamic model has to account the friction
	* @param true if the friction is taken into account, false otherwise
	*/
	inline void includeFrictionInModel(const bool& fric) { this->withFriction = fric; }

	/**
	* @brief Check function
	* Check if the dynamic model takes into account the friction
	* @return true if the friction is taken into account, false otherwise
	*/
	inline bool isDynModelWithFriction() { return this->withFriction; }

	/**
	* @brief Set function
	* Set the startOnPlace flag to state if the robot will start from the currently set joint configuration
	* @param onplace: the flag to be set
	*/
	inline void setOnPlace(const bool& onplace) { this->startOnPlace = onplace; }

	/**
	* @brief Get function
	* Get the startOnPlace flag to state if the robot will start from the currently set joint configuration
	* @return the flag
	*/
	inline bool robotStartsOnPlace() { return this->startOnPlace; }

	/**
	* @brief Get function
	* State if use the model-based residual vector or the measurement acquired from FRI
	* @return true if using the model-based residual vector, false otherwise
	*/
	inline bool useModelBasedResidual() { return this->withModelBasedResidual; }

	/**
	* @brief Set function
	* Set the gravity vector in the RobotDynParams structure
	* @param g_: the input gravity vector
	*/
	inline void setDynGravityVector(const Eigen::VectorXd& g_) { this->robDynPars.g = g_; }

	/**
	* @brief Set function
	* Set the coriolis vector in the RobotDynParams structure
	* @param c_: the input coriolis vector
	*/
	inline void setDynCoriolisVector(const Eigen::VectorXd& c_) { this->robDynPars.cvec = c_; }

	/**
	* @brief Set function
	* Set the mass matrix in the RobotDynParams structure
	* @param m_: the input mass matrix
	*/
	inline void setDynMassMatrix(const Eigen::MatrixXd& m_) { this->robDynPars.B = m_; }

	/**
	* @brief Set function
	* Set the friction vector in the RobotDynParams structure
	* @param m_: the input friction vector
	*/
	inline void setDynFrictionVector(const Eigen::VectorXd& f_) { this->robDynPars.f = f_; }

	/**
	* @brief Conversion from geometric to �analytic Jacobian matrix function
	* Compute the analytic Jacobian, given the pre-computed geometric Jacobian
	* and the current orientation of the robot EE
	* @return the analytic Jacobian
	*/
	Eigen::MatrixXd geo2analyticJacobian();


	/**
	* @brief Compute the analytic Jacobian matrix function in closed-form
	* Compute the analytic Jacobian in closed form, with Euler angles orientation representation = ZYX
	* for the orientation of the EE
	* @param q: the current joint configuration
	* @return the analytic Jacobian
	*/
	virtual Eigen::MatrixXd computeAnalyticJacobian_zyx(const Eigen::VectorXd& q) = 0;

protected:

	std::string name;				//!< Name of the Instrument

	int jointNum;										//!< Number of joints of the robot manipulator
	Eigen::VectorXd jointMsrPosition__;					//!< Vector of measured joint position
	Eigen::VectorXd jointCmdPosition__;					//!< Vector of commanded joint position
	Eigen::VectorXd jointMsrVelocity__;					//!< Vector of measured joint velocity
	Eigen::VectorXd jointCmdVelocity__;					//!< Vector of commanded joint velocity
	Eigen::VectorXd jointAcceleration__;				//!< Vector of measured joint acceleration
	Eigen::VectorXd jointMsrTorques__;					//!< Vector of measured joint torques
	Eigen::VectorXd jointCmdTorques__;					//!< Vector of measured joint torques
	Eigen::VectorXd jointEstDeltaTorquesCoeffs__;				//!< Vector of estimated joint delta torques (static friction?)
	Eigen::VectorXd linkLengths__;						//!< Vector of link lengths of the manipulator
	std::vector <Eigen::MatrixXd> Tbli__;				//!< Array of 4x4 homogeneous transformation matrices of the i-th link pose wrt the link i-1
	//std::vector<EndEffector> tools;						//!< List of endeffectors attached to the robot (standard setup is force senor + needle)
	Eigen::Matrix4d Tbee__;								//!< 4x4 homogeneous matrix of the pose of the EE wrt the base
	Eigen::MatrixXd Jbee__;								//!< 6xn Jacobian matrix
	Eigen::Vector6d vbee__;								//!< 6D vector of velocity of the EE in base frame
	Eigen::VectorXd res__;								//!< Vector of estimated residual
	Eigen::VectorXd resOffset__;						//!< Vector of estimated residual offset
	Eigen::Vector6d extFbee__;							//!< External Cartesian force vector generated from residual vector through static relationship
	RobotDynParams robDynPars;							//!< Struct containing the dynamic parameters and contributions of the robot
	RobotDHParams robDHPars;							//!< Struct containing the DH parameters of the robot

	Eigen::Matrix3d Trpy;								//!< Rotation matrix from phi to w
	Eigen::Matrix4d Twb;								//!< Robot pose in the world frame
	Eigen::Matrix4d TeeOffset;							//!< End-effector pose w.r.t. last robot frame (which is the force sensor frame for Panda Robot)
	Eigen::Vector3i rotCounters;						//!< Counter of rotations around the three axes to unwrap the atan2 output from [-pi, pi] to [-inf inf]
	Eigen::Vector3d eulAng_ee;							//!< Instance of Euler angles of the robot end-effector at the current time-step (unwrapped)
	Eigen::Vector3d eulAngPrev;							//!< Instance of Euler angles of the robot end-effector at the previous time-step (to evaluate counter of rotations)

	Eigen::MatrixXd resGain;							//!< Matrix of residual gains
	Eigen::MatrixXd invResGainDt;						//!< Constant gain matrix occuring in the formulation of the residual expression as inverse matrix
	Eigen::VectorXd dynModelSum;						//!< Vector built by summing up all the components of the dynamic model
	Eigen::VectorXd residualSum;						//!< Cumulative residual variable
	Eigen::VectorXd p0;									//!< Generalized momentum at time 0

	bool startOnPlace;									//!< State if the robot has to start from the currently set joint configuration, or if it has to move to an initial specified one
	bool includeToolDynParams;							//!< State if the computation of the dynamic model has to account also the tool parameters
	bool withFriction;									//!< State if the computation of the dynamic model has to account also the friction
	bool withResOffset;									//!< State if the residual vector has to be take into account the initial offset to be removed
	bool withModelBasedResidual;						//!< State if the residual vector has to be read from FRI or computed from model-based formulation

	float timestamp__;									//!< Timestamp associated to the stored data
	float dt__;											//!< Sample time (e.g., for Kuka it has to be always = 0.005)
	std::mutex resMtx;									//! mutex for the residual vector

	Eigen::Vector6d fMsr;								//!< Measured force from F/T sensor

	/**
	* @brief Utility function
	* Compute the offset to apply in the last DH matrix of the forward kinematics to get the coordinates of the final tip
	* @return the offset to apply on the z-axis of the end-effector frame
	*/
	float computeEEOffset();

	/**
	* @brief CoM computation
	* Retrieves the total CoM of all the attached end-effectors
	* @return the total CoM
	*/
	void computeOverallEECoM(float out[]);

	/**
	* @brief Weight computation
	* Retrieves the total weight of all the attached end-effectors
	* @return the total weight
	*/
	float computeTotalEEWeight();

};


#endif // ROBOT_INTERFACE_HPP_
