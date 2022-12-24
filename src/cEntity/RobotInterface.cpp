// Project Header files
#include "RobotInterface.hpp"

// Standard header files
#include <iostream>

/**
* @brief Default constructor of the Instrument class
*/
RobotInterface::RobotInterface(){ //: Instrument() {

	this->jointNum = -1;
	this->timestamp__ = 0.0;
}

/**
* @brief Constructor of the RobotInterface class with name argument
* @param name_ the name of the Instrument
*/
RobotInterface::RobotInterface(const std::string& name_){//: Instrument(name_) {

	this->name = name_;
	this->jointNum = -1;
	this->timestamp__ = 0.0;

}


/**
* @brief Constructor of the RobotInterface class with name argument
* @param qnum the dofs of the robot
*/
RobotInterface::RobotInterface(const int& qnum){//: Instrument(){

	this->setJointNum(qnum);
	this->initData();

}


/**
* @brief Init function
* Init all the dynamic data of the class with the known value of dofs of the
*/
void RobotInterface::initData() {

	if (this->jointNum != -1) {
		this->jointMsrPosition__.setZero(this->jointNum);
		this->jointCmdPosition__.setZero(this->jointNum);
		this->jointMsrVelocity__.setZero(this->jointNum);
		this->jointCmdVelocity__.setZero(this->jointNum);
		this->jointCmdTorques__.setZero(this->jointNum);
		this->jointEstDeltaTorquesCoeffs__.setZero(this->jointNum);
		this->jointAcceleration__.setZero(this->jointNum);
		this->jointMsrTorques__.setZero(this->jointNum);
		this->linkLengths__.setZero(this->jointNum);
		this->Tbli__.resize(this->jointNum);
		for (int i = 0; i < this->Tbli__.size(); i++) {
			this->Tbli__[i].setIdentity();
		}
		this->fMsr.setZero();

		this->Tbee__.setIdentity();
		this->Jbee__.setZero(SPACE_DIM * 2, this->jointNum);
		this->vbee__.setZero();
		this->res__.setZero(this->jointNum);
		this->resOffset__.setZero(this->jointNum);
		this->withResOffset = false;

		this->robDynPars.B.setZero(this->jointNum, this->jointNum);
		this->robDynPars.C.setZero(this->jointNum, this->jointNum);
		this->robDynPars.f.setZero(this->jointNum);
		this->robDynPars.g.setZero(this->jointNum);
		this->robDynPars.tau.setZero(this->jointNum);

		this->robDHPars.d.setZero(this->jointNum);
		this->robDHPars.a.setZero(this->jointNum);
		this->robDHPars.alpha.setZero(this->jointNum);

		this->resGain.setZero(this->jointNum, this->jointNum);
		//this->invResGainDt.setZero(this->jointNum, this->jointNum);
		this->invResGainDt.setIdentity(this->jointNum, this->jointNum);

		this->dynModelSum.setZero(this->jointNum);
		this->residualSum.setZero(this->jointNum);
		this->p0.setZero(this->jointNum);

		this->includeToolDynParams = false;
		this->withFriction = false;			
		this->timestamp__ = 0.0;

		this->rotCounters.setZero();
		this->eulAngPrev.setZero();
		this->eulAng_ee.setZero();

	}
	else {
		std::cout << "[RobotInterface] WARNING: jointNum not set!!!" << std::endl;
	}

}

/**
* @brief Forward differential kinematics function
* Compute the forward differential kinematic of the robot, by evaluating the Jacobian matrix on the current set value of this->jointPosition
* Internally set the array of homogeneous transformation matrices, i.e., this->Tbli
*/
void RobotInterface::computeDiffKinematics() {

	Eigen::MatrixXd J0n(SPACE_DIM * 2, this->jointNum);
	Eigen::Matrix6d Wne;	// Twist matrix
	Eigen::Matrix3d S;
	Eigen::Vector3d r0en;

	Eigen::Matrix4d Tbe = this->getTbee();
	Eigen::Matrix4d Tbn = this->getLinkTransform(this->jointNum - 1);
	Eigen::Vector3d p0n = Tbn.block<3, 1>(0, 3);
	Eigen::Vector3d p0e = Tbe.block<3, 1>(0, 3);

	r0en = p0n - p0e;

	// Compute the Jacobian matrix
	// Warning: when relating to the end-effector, this function computes the Jacobian matrix
	// with respect to the last frame n of the robot, corresponding to the case in which no 
	// end-effectors or devices are mounted. If additional end-effectors are mounted on the robot,
	// then you must explicitly consider to transform the Jacobian according to 
	// [ADL webpge]
	J0n = this->computeFullJacobian(this->jointNum - 1);
	//J0n = this->computeAnalyticJacobian_zyx(this->getMsrJointPosition());

	// If there are end-effctors mounted, compute the twist matrix for the 
	// Jacobian transformation
	Wne.setIdentity();
	//Wne.topRightCorner(3, 3) = skew(r0en.cast<float>()).cast<double>();

	// Compute the Jacobian matrix related to the end-effector
	this->Jbee__ = Wne * J0n;

	// Compute the end-effector Cartesian velocity
	this->vbee__ = this->Jbee__ * this->jointMsrVelocity__;

}


/**
* @brief Jacobian function
* Compute the full Jacobian matrix for the given chosen link (end-effector by default).
* @param link the link of which the Jacobian matrix has to be computed
* @return the requested Jacobian matrix
*/
Eigen::MatrixXd RobotInterface::computeFullJacobian(const int& link) {

	Eigen::MatrixXd J(SPACE_DIM*2 , this->jointNum);
	Eigen::MatrixXd Jl(SPACE_DIM , this->jointNum);
	Eigen::MatrixXd Ja(SPACE_DIM , this->jointNum);

	// Compute Linear part
	Jl = this->computeLinearJacobian(link);

	// Compute Linear part
	Ja = this->computeAngularJacobian(link);

	// Build the full Jacobian
	J.topRows(SPACE_DIM) = Jl;
	J.bottomRows(SPACE_DIM) = Ja;

	// Set the Jacobian matrix in the class
	this->Jbee__ = J;

	return J;

}

/**
* @brief Pseudo-inverse compuation function
* Compute and return the nx6 pseudo-inverse matrix of the current Jacobian
* @return the nx6 pseudo-inverse matrix of the current Jacobian
*/
Eigen::MatrixXd RobotInterface::getJPinv() {

	Eigen::MatrixXd JT(this->jointNum, SPACE_DIM*2), Jpinv(this->jointNum, SPACE_DIM * 2);
	Eigen::MatrixXd J(SPACE_DIM * 2, this->jointNum);

	J = this->Jbee__;
	JT = J.transpose();

	// Compute the Jacobian pseudo-inverse
	Jpinv = JT * (J * JT).inverse();

	// Return the pseudo-inverse
	return Jpinv;

}

/**
* @brief Update function
* Update the full state of the robot, given the input joint positions, velocities and torques
* Internally compute forward kinematics, differential kinematics, dynamics and residual vectors
*
*/
void RobotInterface::updateRobotState(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, const Eigen::VectorXd& tau, const bool& online) {

	// Set joint positions, joint velocities and joint measured torques on the robot
	this->setMsrJointPosition(q);
	this->setMsrJointVelocity(qdot);
	this->setMsrJointTorques(tau);

	// Compute forward kinematics
	this->computeKinematics();

	// Compute differential kinematics
	this->computeDiffKinematics();

	// Compute dynamics
	this->computeDynamics();

	// Compute residual vector
	//this->computeResidualFull(q, qdot, tau);
	/*if (online) {
		this->computeResidualFull(q, qdot, tau);
	}//*/
	
	// Compute external contact force vector from residual
	this->extFbee__ = this->staticForceReconstruction(this->res__);

	// Unwrap the Euler angles resulting from the current EE rotation matrix
	Eigen::Matrix3d R = this->getEERotMat();
	Eigen::Vector3d eulAng = rot2rpy(R);
	//std::cout << "eulAng = " << eulAng.transpose() << std::endl;
	for (int i = 0; i < eulAng.size(); i++) {
		if (this->eulAngPrev(i) > M_PI/2.0 && eulAng(i) < 0.0) {
			this->rotCounters(i) = this->rotCounters(i) + 1;
		}
		else if (this->eulAngPrev(i) < - M_PI / 2.0 && eulAng(i) > 0.0) {
			this->rotCounters(i) = this->rotCounters(i) -1;
		}
		this->eulAng_ee(i) = eulAng(i) + this->rotCounters(i) * 2.0 * M_PI;
	}
	this->eulAngPrev = eulAng;
}


/**
* @brief Conversion from geometric to analytic Jacobian matrix function
* Compute the analytic Jacobian, given the pre-computed geometric Jacobian
* and the current orientation of the robot EE
* @return the analytic Jacobian
*/
Eigen::MatrixXd RobotInterface::geo2analyticJacobian() {

	Eigen::MatrixXd Ja, Jg;
	Eigen::Matrix3d Trpy, Tinv;
	Eigen::Vector3d rpy;

	// Get geometric Jacobian
	Jg = this->getJacobian();

	// Fill the RPY transformation matrix
	rpy = rot2rpy(this->getEERotMat());
	Trpy << cos(rpy(1)) * cos(rpy(2)), -sin(rpy(2)), 0,
		    cos(rpy(1)) * sin(rpy(2)),  cos(rpy(2)), 0,
		   -sin(rpy(1)),                          0, 1;

	Tinv = Trpy.inverse();

	Ja = Jg;
	Ja.bottomRightCorner(SPACE_DIM, SPACE_DIM) = Tinv * Jg.bottomRightCorner(SPACE_DIM, SPACE_DIM);

	return Ja;

}



/**
* @brief Residual calculation function
* COmpute the full residual vector from the known joint velocity qdot and the applied torques trq
* Set internally the residual vector
* @param q the joint position vector
* @param qdot the joint velocity vector
* @param trq the joint applied torques
* @param gain the gain for the computation of the residual (default = 5.0)
*/
void RobotInterface::computeResidualFull(const Eigen::VectorXd& q, const Eigen::VectorXd& q_dot, const Eigen::VectorXd& trq) {

	// Define required variables
	Eigen::VectorXd rk(this->jointNum), qdot_k(this->jointNum), tau_k(this->jointNum), gk(this->jointNum), fk(this->jointNum), tau_friction(this->jointNum);
	Eigen::MatrixXd KDtIinv(this->jointNum, this->jointNum), Bk(this->jointNum, this->jointNum), Ck(this->jointNum, this->jointNum), CkT(this->jointNum, this->jointNum), K(this->jointNum, this->jointNum);
	Eigen::VectorXd q_(this->jointNum), qdot_(this->jointNum), trq_(this->jointNum), params_(this->jointNum);
	float dt = this->dt__;

	// Initialize variables
	KDtIinv = this->invResGainDt;
	K = this->resGain;
	q_ = q.cast<double>();
	qdot_ = q_dot.cast<double>();
	trq_ = trq.cast<double>();
	//params_ = this->frictionGravityParams.cast<double>();

	// Get variables of the current iteration
	Bk = this->robDynPars.B;
	Ck = this->robDynPars.C;
	gk = this->robDynPars.g;
	fk = this->robDynPars.f;
	CkT = Ck.transpose();
	qdot_k = q_dot;

	// Set the measured torques
	tau_k = trq;
	if (this->isDynModelWithFriction()) {
		//tau_k -= (this->computeDeltaFriction(q_, qdot_, params_)).cast<float>();
	}

	// Update dynModelSum
	this->dynModelSum += (tau_k + CkT * qdot_k - gk);

	// Compute the current value of the residual vector
	rk = KDtIinv * K * ((Bk * qdot_k - this->p0) - this->dynModelSum * dt - this->residualSum * dt);

	// Update the residual cumulative term
	this->residualSum += rk;

	Eigen::VectorXd rk_filt, rk_old = this->getResidualVector();
	rk_filt = 0.1 * rk + 0.9 * rk_old;
	this->setResidualVector(rk_filt);

	// Update the residual vector with the quantity computed above
	if (this->withResOffset) {
		// If offset reset is requested, subtract it to the residual measurement
		//this->setResidualVector(rk - this->resOffset__);
	}
	else {
		//this->setResidualVector(rk);
	}

	//this->residual = rk;

}


/**
* @brief Force reconstruction function
* Compute the Cartesian force vector from the given joint torque vector, through the static relationship
* @param tau: the joint torque vector
* @return the corresponding Cartesian force vector
*/
Eigen::Vector6d RobotInterface::staticForceReconstruction(const Eigen::VectorXd& tau) {

	Eigen::Vector6d f;
	Eigen::MatrixXd JT(this->jointNum,SPACE_DIM*2);
	Eigen::MatrixXd J(SPACE_DIM*2, this->jointNum), JThash(SPACE_DIM*2, this->jointNum);
	Eigen::MatrixXd Jl(SPACE_DIM, this->jointNum), JlThash(SPACE_DIM, this->jointNum);
	Eigen::MatrixXd JlT(this->jointNum, SPACE_DIM);
	f.setZero();

	// Build the pseudo-inverse of the transpose jacobian
	J = this->Jbee__;
	JT = J.transpose();
	Jl = J.topRows(SPACE_DIM);
	JlT = Jl.transpose();
	JThash = (J * JT).inverse() * J; //<-- This is the correct one
	JlThash = (Jl * JlT).inverse() * Jl; //<--- This is more robust if you assume moments equal to 0

	// Compute the force vector
	f = JThash * tau; // // <--- If you want to use full Jacobian
	//f.topRows(SPACE_DIM) = JlThash * tau; // <--- If you want to use only linear Jacobian

	// Return f
	return f;

}


/**
* @brief Utility function
* Compute the offset to apply in the last DH matrix of the forward kinematics to get the coordinates of the final tip
* @return the offset to apply on the z-axis of the end-effector frame
*/
/*float RobotInterface::computeEEOffset() {

	float offset = 0.0;
	float tool_i_size[SPACE_DIM];

	for (int i = 0; i < this->tools.size(); i++) {
		(this->tools[i].getSize(tool_i_size));
		offset += tool_i_size[Z_AXIS];
	}
	return offset;

}*/


/**
* @brief Weight computation
* Retrieves the total weight of all the attached end-effectors
* @return the total weight
*/
/*float RobotInterface::computeTotalEEWeight() {

	float weight = 0.0;

	for (int i = 0; i < this->tools.size(); i++) {
		float wi = this->tools[i].getWeight();
		weight += wi;
	}

	return weight;

}*/

/**
* @brief CoM computation
* Retrieves the total CoM of all the attached end-effectors
* @return the total CoM
*/
/*void RobotInterface::computeOverallEECoM(float out[]) {

	float com[SPACE_DIM];
	float tool_i_com[SPACE_DIM];

	std::memset(com, 0.0, SPACE_DIM * sizeof(float));
	std::memset(tool_i_com, 0.0, SPACE_DIM * sizeof(float));

	for (int i = 0; i < this->tools.size(); i++) {

		this->tools[i].getCoM(tool_i_com);

		com[0] += tool_i_com[0];
		com[1] += tool_i_com[1];
		com[2] += tool_i_com[2];
	}
	// TODO: Check this hard-coded value for force sensor + needle
	com[2] = 0.1069;
	//this->tools[0].getCoM(tool_i_com);
	//std::memcpy(com, tool_i_com, SPACE_DIM * sizeof(float));

	std::memcpy(out, com, SPACE_DIM * sizeof(float));

}*/

