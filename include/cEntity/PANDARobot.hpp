#ifndef PANDAROBOT_HPP_
#define PANDAROBOT_HPP_

// Project Header files
#include "RobotInterface.hpp"

// Eigen Header files
#define EIGEN_RUNTIME_NO_MALLOC
#include <Eigen\Dense>
//#include "LWR_Dynamic_Model_Lib.h"

// Number of dynamic paramters
#define DYN_PARAMS_NUM 10

// Number of joints
#ifndef PANDA_JOINT_NUM 
#define PANDA_JOINT_NUM 7
#endif // PANDA_JOINT_NUM 

// Twist dimension
#ifndef TWIST_DIM
#define TWIST_DIM 6
#endif // TWIST_DIM

// Space dimension
#ifndef SPACE_DIM
#define SPACE_DIM 3
#endif // SPACE_DIM


#define GRAVITY_DYN_PARAMS_NUM (SPACE_DIM * PANDA_JOINT_NUM + PANDA_JOINT_NUM)

// Typedef Eigen
namespace Eigen {

	typedef Eigen::Matrix<double, PANDA_JOINT_NUM, 1> Vector7d;
	typedef Eigen::Matrix<float, PANDA_JOINT_NUM, 1> vector7f;

	typedef Eigen::Matrix<double, TWIST_DIM, 1> Vector6d;
	typedef Eigen::Matrix<float, TWIST_DIM, 1> Vector6f;

	typedef Eigen::Matrix<double, TWIST_DIM, PANDA_JOINT_NUM> Matrix6x7d;
	typedef Eigen::Matrix<float, TWIST_DIM, PANDA_JOINT_NUM> Matrix6x7f;
	typedef Eigen::Matrix<double, PANDA_JOINT_NUM, TWIST_DIM> Matrix7x6d;
	typedef Eigen::Matrix<float, PANDA_JOINT_NUM, TWIST_DIM> Matrix7x6f;

	typedef Eigen::Matrix<float, PANDA_JOINT_NUM, PANDA_JOINT_NUM> matrix7d;
	typedef Eigen::Matrix<double, PANDA_JOINT_NUM, PANDA_JOINT_NUM> Matrix7d;
	typedef Eigen::Matrix<float, TWIST_DIM, TWIST_DIM> matrix6f;
	typedef Eigen::Matrix<double, TWIST_DIM, TWIST_DIM> Matrix6d;

	typedef Eigen::Matrix<double, SPACE_DIM, PANDA_JOINT_NUM> Matrix3x7d;
	typedef Eigen::Matrix<float, SPACE_DIM, PANDA_JOINT_NUM> Matrix3x7f;
	typedef Eigen::Matrix<double, PANDA_JOINT_NUM, SPACE_DIM> Matrix7x3d;
	typedef Eigen::Matrix<float, PANDA_JOINT_NUM, SPACE_DIM> Matrix7x3f;

}

// Link numbers
enum PANDA_LINKS {LINK1 = 0, LINK2, LINK3, LINK4, LINK5, LINK6, LINK7};

// Jacobian parts id
enum PANDA_JACOBIANS_ID { FULL_JACOBIAN, LINEAR_JACOBIAN, ANGULAR_JACOBIAN };


#pragma unmanaged
class PANDARobot : public RobotInterface {

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	* @brief Default constructor of the PANDARobot class
	*/
	PANDARobot();

	/**
	* @brief Default constructor of the PANDARobot class
	* @param name_: the identifying name of the instrument in use
	*/
	PANDARobot(const std::string& name_);

	/**
	* @brief Constructor of the RobotInterface class with name argument
	* @param qnum the dofs of the robot
	*/
	PANDARobot(const int& qnum);

	/**
	* @brief Default destroyer of the PANDARobot class
	*/
	~PANDARobot() {}

	/**
	* @brief Init function (overloaded)
	* Init all the dynamic data of the class with the known value of dofs of the
	*/
	void initData();

	/**
	* @brief Init function
	* Check the input pair of comment+value to assign the corresponding parameter
	* The function assumes that configFile has been previously set
	* @param comment: the string specifying the title comment of parameter
	* @param value: the string specifying the value of the parameter
	*/
	void loadParamsFromConfigFile(const std::string& comment, const std::string& value);

	/**
	* @brief Forward kinematics function
	* Compute the forward kinematic of the robot, based on the current set value of this->jointPosition
	* Internally set the array of homogeneous transformation matrices, i.e., this->Tbli
	*/
	void computeKinematics();


#ifdef BUILD_PANDA_DYN_MODEL
	/**
	* @brief Forward dynamics function
	* Compute the dynamic model of the robot, based on the current set value of joint position, velocity and torques,
	* along with the known dynamic parameters of the model
	*/
	void computeDynamics();
#endif // BUILD_PANDA_DYN_MODEL

	/**
	* @brief Set function
	* Set the residual vector computed by FRI, as estimation of the external torques applied on the model
	* @param r: the residual vecotr computed by FRI
	*/
	inline void setFRIResidual(const Eigen::Vector7d& r) { this->friResidual = r; }

	/**
	* @brief Get function
	* Retrieves the residual vector computed by FRI, as estimation of the external torques applied on the model
	* @return the residual vecotr computed by FRI
	*/
	inline Eigen::Vector7d getFRIResidual() { return this->friResidual; }

	/**
	* @brief Jacobian function
	* Compute the linear Jacobian matrix for the given chosen link (end-effector by default).
	* @param link the link of which the Jacobian matrix has to be computed
	* @return the requested Jacobian matrix
	*/
	Eigen::MatrixXd computeLinearJacobian(const int& link = LINK7);

	/**
	* @brief Jacobian function
	* Compute the angular Jacobian matrix for the given chosen link (end-effector by default).
	* @param link the link of which the Jacobian matrix has to be computed
	* @return the requested Jacobian matrix
	*/
	Eigen::MatrixXd computeAngularJacobian(const int& link = LINK7);


	/**
	* @brief Get function
	* Get the determinant of the product J * JT
	* @return the determinant of J * JT
	*/
	inline float getEEFrameOffset() { return this->eeDHOffset; }

	/**
	* @brief Get function
	* Get the Cartesian Force vector at the end-effector wrt base frame, reconstructed from FRI residual
	* @return the Cartesian force vector
	*/
	inline void setFriResFbee(const Eigen::Vector6d& f) { this->resFriFbe = f; }

	/**
	* @brief Get function
	* Get the Cartesian Force vector at the end-effector wrt base frame, reconstructed from FRI residual
	* @return the Cartesian force vector
	*/
	inline Eigen::Vector6d getFriResFbee() { return this->resFriFbe; }

	/**
	* @brief Get function
	* Get the determinant of the product J * JT
	* @return the determinant of J * JT
	*/
	inline float getDeterminantJJT() { return this->detJJT; }

	/**
	* @brief Get function
	* Get the determinant of the product J * JT
	* @return the determinant of J * JT
	*/
	inline float getDeterminantJTJ() { return this->detJTJ; }

	/**
	* @brief Get function
	* Get the Cartesian forces directly returned by FRI
	* @return the Cartesian force vector computed by FRI
	*/
	inline Eigen::Vector6d getFRICartesianForces() { return this->friFbee; }

	/**
	* @brief Set function
	* Set the Cartesian forces directly returned by FRI
	* @param f: the Cartesian force vector computed by FRI
	*/
	void setFRICartesianForces(const Eigen::Vector6d& f) { this->friFbee = f; }

	/**
	* @brief Set function
	* Set the current gravity vector computed by FRI
	* @param g: the input gravity vector
	*/
	inline void setFRIGravityVec(const Eigen::Vector7d& g) { this->friGravityVec = g; }

	/**
	* @brief Set function
	* Set the current inertia matrix computed by FRI
	* @param B: the input inertia matrix
	*/
	inline void setFRIInertiaMatrix(const Eigen::Matrix7d& B) { this->friInertiaMat = B; }

	/**
	* @brief Get function
	* Get the current gravity vector computed by FRI
	* @return the input gravity vector
	*/
	inline Eigen::Vector7d getFRIGravityVec() { return this->friGravityVec; }

	/**
	* @brief Get function
	* Get the current inertia matrix computed by FRI
	* @return the inertia matrix
	*/
	inline Eigen::Matrix7d getFRIInertiaMatrix() { return this->friInertiaMat; }


	/**
	* @brief Set function
	* Set the number of parameters employed for the friction model
	* @param num: the number of parameters
	*/
	static inline void setFrictionParamsNum(const int& num) { frictionParamsNum = num; }

	/**
	* @brief Set function
	* Set the friction parameters vector 
	* @param the friction parameters vector
	*/
	inline void setEstimatedFrictionParameters(const Eigen::VectorXd& fp) { frictionGravityParams = fp; }

	///**
	//* @brief Set function
	//* Save the friction parameters on a file
	//* @param filename: the name of the file where the friction parameters have to be saved
	//*/
	//void saveEstimatedFrictionParameters(const char* filename);

	///**
	//* @brief Load function
	//* Load the friction parameters from a file
	//* @param filename: the name of the file where the friction parameters are stored
	//*/
	//void loadEstimatedFrictionParameters(const char* filename);

	/**
	* @brief Delta-gravity computation function
	* Compute the delta-gravity contribution with the known joint positions, velocities and parameters of estimated the delta-dynamics
	* @param q_k: the joint position vector
	* @param fg: the estimated friction/gravity parameters
	* @return the resulting delta gravity torques
	*/
	//static Eigen::VectorXd computeDeltaGravity(const Eigen::Vector7d& q_k, const Eigen::VectorXd& fg);

	/**
	* @brief Delta-friction computation function
	* Compute the delta-friction contribution with the known joint positions and parameters of estimated the delta-dynamics
	* @param q_k: the joint velocity position
	* @param qdot_k: the joint velocity vector
	* @param fg: the estimated friction/gravity parameters
	* @return the resulting delta friction torques
	*/
	static Eigen::Vector7d computeDeltaFriction(const Eigen::Vector7d& q_k, const Eigen::Vector7d& qdot_k, const Eigen::VectorXd& fg);

	/**
	* @brief Delta-dynamics callback function
	* The function computes the delta-dynamics of the robot model, from the current joint positions
	* velocities and estimated dynamics parameters. The firm of the function needs to match the
	* callback firm to be called from the Gauss-Newton estimation algorithm, in the related estimation task
	* @param q: joint position data
	* @param qdot: joint velocity data
	* @param params: joint estimated friction parameters data
	* @param fricParamsNum: the number of coefficients used for friction for each joint
	* @param wGravity: if params accounts gravity term or not
	*/
	static Eigen::VectorXd PANDARobot::computeDeltaDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, const Eigen::VectorXd& params);// , const int& fricParamsNum, const bool& wGravity);

	/**
	* @brief Delta-dynamics callback function
	* The function computes the Jacobian matrix of the delta-dynamics of the robot model, from the current joint positions
	* velocities and estimated dynamics parameters. The firm of the function needs to match the
	* callback firm to be called from the Gauss-Newton estimation algorithm, in the related estimation task
	* @param q: joint position data
	* @param qdot: joint velocity data
	* @param params: joint estimated friction parameters data
	* @return the jacobian matrix
	*/
	static Eigen::MatrixXd computeDeltaDynJacobian(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, const Eigen::VectorXd& params);

	/**
	* @brief Delta-gravity Jacobian computation function
	* Compute the delta-gravity jacobian contribution with the known joint positions, velocities and parameters of estimated the delta-dynamics
	* @param q_k: the joint position vector
	* @param fg: the estimated friction/gravity parameters
	* @return the resulting delta gravity Jacobian matrix
	*/
	//static Eigen::MatrixXd computeDeltaGravityJacobian(const Eigen::Vector7d& q_k, const Eigen::VectorXd& fg);

	/**
	* @brief Check function
	* Check if the computation of the residual should take into account the locally estimated delta-gravity term
	* @return true if the delta-gravity term is taken into account, false otherwise
	*/
	static inline void useDeltaGravity(const bool& flag) { withDeltaGravity = flag; }
	
	/**
	* @brief Reset data
	* Reset the robot data
	*/
	void resetDynParams();

	/**
	* @brief Compute the analytic Jacobian matrix function in closed-form
	* Compute the analytic Jacobian in closed form, with Euler angles orientation representation = ZYX
	* for the orientation of the EE
	* @param q: the current joint configuration
	* @return the analytic Jacobian
	*/
	Eigen::MatrixXd computeAnalyticJacobian_zyx(const Eigen::VectorXd& q);

private:

//	CLWR_Dynamic_Model_Lib dyn;

	Eigen::Vector7d friResidual;						//!< Residual vector from FRI
	Eigen::Vector7d friResidual_off;					//!< FRI Residual vector offset
	Eigen::Vector6d resFriFbe;							//!< Cartesian Force vector at the end-effector wrt base frame, reconstructed from FRI residual
	Eigen::Vector6d friFbee;							//!< Cartesian Force vector at the end-effector wrt base frame, directly acquired from FRI

	Eigen::Vector7d friGravityVec;						//!< Gravity vector returned by the FRI
	Eigen::Matrix7d friInertiaMat;						//!< Inertia matrix returned by the FRI
	Eigen::VectorXd modelGravityVec;					//!< Gravity vector
	Eigen::MatrixXd modelInertiaMat;					//!< Inertia matrix

	static int frictionParamsNum;						//!< Number of parameters considered for the friction model
	static bool withDeltaGravity;						//!< State if the computation of the dynamic model has to account also the locally estimated delta-gravity 

	float eeDHOffset;									//!< Offset of the end effector frame w.r.t. link 7 (modified DH convention)
	
	Eigen::VectorXd frictionGravityParams;				//!< Dynamic array of estimated friction parameters
	Eigen::Vector7d tau_delta;							//!< Residual torque generated after friction parameters estimation

	float DeltaT;										//!< Sample time (for KUKA it has to be always = 0.005)
	float detJJT;										//!< Determinant of the J * JT
	float detJTJ;										//!< Determinant of the J * JT

	
};



#endif // PANDAROBOT_HPP_
