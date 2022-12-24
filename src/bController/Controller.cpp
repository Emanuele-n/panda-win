// Project Header files
#include "Controller.hpp"
#include "RobotInterface.hpp"
#include "FrankaProxy.hpp"

Controller::Controller(){

	// Errors
	this->err.setZero();
	this->errD.setZero();
	this->errI.setZero();
	this->err_tf.setZero();
	this->errD_tf.setZero();

	// Dynamic variables
	this->u.setZero();
	this->g.setZero();
	this->J.setIdentity();
	this->JT.setIdentity();
	this->B.setIdentity();
	this->C.setIdentity();
	this->jointNum = 7;

	// Bools
	this->position = true;
	this->orientation = true;

	// Time variables
	this->dtAvg = 0.01;
	this->time_ = 0.0;

	// Cartesian variables
	this->p.setZero();
	this->v.setZero();
	this->a.setZero();
	this->R.setIdentity();

	// Reference velovity and acceleration
	this->vRef.setZero();
	this->aRef.setZero();
	this->vPrev.setZero();

	// Commanded velocity
	this->dqcmd.setZero();
	this->dqcmd_prev.setZero();
	this->dqcmd_filtered.setZero();
	this->dqcmd_filtered_prev.setZero();

	// Estimated forces
	this->fMsr.setZero();
	this->fMsr_prev.setZero();
	this->fMsr_filtered.setZero();	
	this->fMsr_filtered_prev.setZero();
}

Controller::~Controller(){}

Eigen::VectorXd Controller::controlLaw_Admittance(){

	// Sanity check
	if (this->torqueCtrl == true){
		std::cout << "\n[WARNING]: torque control is set, cannot use kinematic admittance control" << std::endl;
		return this->dqcmd.setZero();
	}

	// Initialize control specific variables
	// Proportional gains
	Eigen::Matrix6d Kp;		
	Kp.setIdentity();

	// Derivative gain		
	Eigen::Matrix6d Kd;	
	Kd.setIdentity();

	// Integral gain		
	Eigen::Matrix6d Ki;	
	Ki.setIdentity();

	// Set gains
	// Proportional gains
	Kp(0, 0) = 0.5;
	Kp(1, 1) = 0.5;
	Kp(2, 2) = 0.5;
	Kp(3, 3) = 0.5;
	Kp(4, 4) = 0.5;
	Kp(5, 5) = 0.5;

	// Derivative gains
	Kd(0, 0) = 0.05;
	Kd(1, 1) = 0.05;
	Kd(2, 2) = 0.05;
	Kd(3, 3) = 0.05;
	Kd(4, 4) = 0.05;
	Kd(5, 5) = 0.05;

	// Integral gains
	Ki(0, 0) = 0.1;
	Ki(1, 1) = 0.1;
	Ki(2, 2) = 0.1;
	Ki(3, 3) = 0.1;
	Ki(4, 4) = 0.1;
	Ki(5, 5) = 0.1;

	// Jacobian pseudoinverse
	Eigen::MatrixXd Jpinv;
	Jpinv.setIdentity();
	
	// Read the current robot state from the robot class
	this->p = this->robot->getEEPosition();
	this->v = this->robot->getEEVelocity();
	this->R = this->robot->getEERotMat();
	this->g = this->robot->getModelDynParams().g;
	this->fMsr = this->robot->getMsrForce();

	// Map force in EE frame
	//fMsr.head(3) = this->robot->getLinkRotMat(6)*fMsr.head(3);
	//fMsr.tail<3>() = this->robot->getLinkRotMat(6)*fMsr.tail<3>();

	// Get current references
	this->pRef = this->getP_ref();
	this->RRef = this->getR_ref();

	// vRef (should be updated from trajectory)

	// Compute quaternions
	Eigen::Quaterniond orientation(R);
	Eigen::Quaterniond orientation_d(RRef);

	if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
		orientation.coeffs() << -orientation.coeffs();
	}
	// "difference" quaternion
	Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
	
	// Proportional error
	this->err.head(3) = this->p - this->pRef;
	this->err.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
	this->err.tail(3) = - this->R * this->err.tail(3);

	// Derivative error
	this->errD = this->v - this->vRef;

	// Integral error
	this->errI += this->err * this->dtAvg;

	// Position only
	if (this->orientation == false){ this->err.tail(3).setZero(); }

	// Orientation only 
	if (this->position == false){ this->err.head(3).setZero(); }

	// Get Jacobian
	this->J = this->robot->getJacobian();
	this->JT = J.transpose();

	// Compute Jacobian right pseudoinverse
	Jpinv = this->JT * (this->J * this->JT).inverse();

	/*// Compute Cartesian acceleration with Euler's derivative
	if (this->firstIt == true) {											    
		this->firstIt = false;
		a.setZero();	
	}
	else {
		this->a = (this->v - this->vPrev) / this->dtAvg;
	}*/

	// Force filters (from De Luca et al. "Admittance Control for Human-Robot Interaction Using an Industrial Robot Equipped with a F/T Sensor")
	//double beta = 0.3;
	// High pass 
	//this->fMsr_filtered = this->fMsr_prev + (this->fMsr - this->fMsr_prev) * beta;
	// Low pass 
	//this->fMsr_filtered = this->fMsr_prev + (this->fMsr - this->fMsr_filtered_prev) * beta;

	// if force not detected
	// Kinematic control PID
	this->dqcmd = Jpinv * (-Kp * this->err - Kd * this->errD - Ki * this->errI); 

	// else ...

	// Velocity filter
	double alpha = 0.3;

	// Low pass
	this->dqcmd_filtered = this->dqcmd_prev + (this->dqcmd - this->dqcmd_filtered_prev) * alpha; 

	// Store values before returning
	// Velocities
	this->dqcmd_filtered_prev = this->dqcmd_filtered;
	this->dqcmd_prev = this->dqcmd;
	//this->vPrev = this->v;

	// Forces
	//this->fMsr_prev = this->fMsr;
	//this->fMsr_filtered_prev = this->fMsr_filtered;
	
	// Saturation
	//this->dqcmd = this->velocitySaturation(this->dqcmd);

	return this->dqcmd_filtered;
}

Eigen::VectorXd Controller::controlLaw_Impedance(){

	// Sanity check
	if (this->torqueCtrl == false){
		std::cout << "\n[WARNING]: velocity control is set, cannot use impedance control" << std::endl;
		return this->u.setZero();
	}

	// Initialize control specific variables
	// Jacobian pseudoinverse
	Eigen::MatrixXd Jpinv;

	// Compute PID + gravity compensation
	Eigen::VectorXd PIDxGravity;	
	PIDxGravity = this->controlLaw_PIDxGravity();

	// Read the current robot state from the robot class
	this->B = this->robot->getModelDynParams().B;
	this->C = this->robot->getModelDynParams().C;
	this->J = this->robot->getJacobian();
	// NOTE: J is not multiplied per Wtwist, why? if twisted you have to twist vRef and aRef
	this->JT = J.transpose();

	// Compute Jacobian right pseudoinverse
	Jpinv = this->JT * (this->J * this->JT).inverse();

	// this->aRef (updated from trapezoidal trajectory)

	this->u = this->B * Jpinv * this->aRef + this->C * Jpinv * this->vRef + PIDxGravity;
	return this->u;
}

Eigen::VectorXd Controller::controlLaw_PIDxGravity() {

	// Sanity check
	if (this->torqueCtrl == false){
		std::cout << "\n[WARNING]: velocity control is set, cannot use PID torque control" << std::endl;
		return this->u.setZero();
	}

	// Initialize control specific variables
	// Proportional gains
	Eigen::Matrix6d Kp;		
	Kp.setIdentity();

	// Derivative gain		
	Eigen::Matrix6d Kd;	
	Kd.setIdentity();

	// PID action
	Eigen::VectorXd pidAction;	
	pidAction.setZero();

	// Set gains
	// Proportional gains
	Kp(0, 0) = 100;
	Kp(1, 1) = 100;
	Kp(2, 2) = 100;
	Kp(3, 3) = 10;
	Kp(4, 4) = 10;
	Kp(5, 5) = 10;

	// Derivative gains
	Kd(0, 0) = 50;
	Kd(1, 1) = 50;
	Kd(2, 2) = 50;
	Kd(3, 3) = 2;
	Kd(4, 4) = 2;
	Kd(5, 5) = 2;

	// Read the current robot state from the robot class
	this->p = this->robot->getEEPosition();
	this->v = this->robot->getEEVelocity();
	this->R = this->robot->getEERotMat();
	this->g = this->robot->getModelDynParams().g;

	// Get current references
	this->pRef = this->getP_ref();
	this->RRef = this->getR_ref();

	// this->vRef (updated from trapezoidal trajectory)

	// Compute quaternions
	Eigen::Quaterniond Q(this->R);
	Eigen::Quaterniond Q_ref(this->RRef);

	if (Q_ref.coeffs().dot(Q.coeffs()) < 0.0) {
		Q.coeffs() << -Q.coeffs();
	}
	// "difference" quaternion
	Eigen::Quaterniond error_quaternion(Q.inverse() * Q_ref);
	
	// Compute error
	this->err.head(3) = this->p - this->pRef;
	this->errD.head(3) = this->v.head(3) - this->vRef.head(3);
	this->err.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
	this->err.tail(3) = - this->R * this->err.tail(3);

	// Position only
	if (this->orientation == false){
		this->err.tail(3).setZero();
	}

	// Orientation only 
	if (this->position == false){
		this->err.head(3).setZero();
	}

	// Compute error in task frame
	Eigen::Matrix6d Wtwist;
	Wtwist.setZero();
	Wtwist.topLeftCorner(3, 3) = R.transpose();
	Wtwist.bottomRightCorner(3, 3) = R.transpose(); 
	this->err_tf = Wtwist * this->err;
	this->errD_tf = Wtwist * this->errD;

	// Twist vRef aRef

	// Update PID action
	pidAction = (-Kd * this->errD_tf - Kp * this->err_tf);

	// Get Jacobian
	this->J = this->robot->getJacobian();
	this->J = Wtwist * this->J;
	this->JT = J.transpose();

	// Set control law
	this->u = this->g + this->JT * (pidAction);

	return this->u;
}

std::vector<double> Controller::evalTrapezoidalTrajectory(double t, double L){

	// TODO: maybe make a trajectory struct to initialize constant parameteres once 
	// Trajectory parameters
	const double v_max = 0.1;
	const double a_max = 0.1;
	double T = (L*a_max + pow(v_max, 2)) / (a_max*v_max);
	double Tcoast = v_max / a_max;

	// Initialize steps
	double delta = 0;
	double d_delta = 0;
	double dd_delta = 0;

	// Select the phase to compute delta steps
	if (t <= Tcoast) {
		//std::cout << "phase 1 " << std::endl;
		delta = a_max * pow(t, 2) / 2.0;
		d_delta = a_max * t;
		dd_delta = a_max;
	}
	else if (t > Tcoast && t <= T - Tcoast) {
		//std::cout << "phase 2 " << std::endl;
		delta = v_max * t - pow(v_max, 2) / (2.0 * a_max);
		d_delta = v_max;
		dd_delta = 0;
	}
	else if (t > T - Tcoast && t <= T) {
		//std::cout << "phase 3 " << std::endl;
		delta = -a_max * pow(t - T, 2) / 2.0 + v_max * T - pow(v_max, 2) / a_max;
		d_delta = -a_max * (t - T);
		dd_delta = -a_max;
	}
	else {
		//std::cout << "phase 4 " << std::endl;
		delta = L;
		d_delta = 0;
		dd_delta = 0;
	}

	if (L < 1e-4) {
		L = 1e-4;
		delta = 1e-4;
	}

	return {delta, d_delta, dd_delta};
}

void Controller::init() {

	Eigen::VectorXd q, qd, tau;
	int jointNum = this->robot->getJointNum();

	q.setZero(jointNum);
	qd.setZero(jointNum);
	tau.setZero(jointNum);
	
	this->exit_loop = false;
	this->started = false;
	this->time_ = 0.0;
	this->ctrlRateSS.str("");
	this->ctrlDt = 1e-3; // 1 kHz
    alba::RobotState rs = this->robotProxy->getRobotState();
    q = rs.msrJointPosition.cast<double>();
    qd = rs.msrJointVelocity.cast<double>();
    tau = rs.msrJointTorque.cast<double>();
	
	// Update the full state of the robot with the current measurements
	std::cout << "Robot structure initialization in Controller::init() ... " << std::endl;
	this->robot->updateRobotState(q, Eigen::VectorXd::Zero(jointNum), tau, false);

	// Init Coppelia in Controller
    this->initCoppelia();

	// TEST
	Eigen::Vector7d q_test;
	q_test.setZero();
	this->coppelia->setJointPosition(q_test, this->simPort, simx_opmode_oneshot);

	std::cout << "Controller initialization completed." << std::endl;

}

void Controller::generateTrajectoryOnline(){

	// "Duration" used for simple trajectory and quaternion interpolation
	double T = 10.0; 

	// Compute increment from time variables updated in MainLoop
	double increment = this->time_ + this->dtAvg;

	if (this->tracking == true){

		// Get initial and desired Cartesian positions
		Eigen::Vector3d p_init = this->T_init.block<3,1>(0,3);
		Eigen::Vector3d p_des = this->T_des.block<3,1>(0,3);		
		
		// Get initial and desired rotation matrix
		Eigen::Matrix3d R_init = this->T_init.block<3,3>(0,0);
		Eigen::Matrix3d R_des = this->T_des.block<3,3>(0,0);

		// Compute quaternions from rotation matrices
		Eigen::Quaterniond Q_init(R_init); 
		Eigen::Quaterniond Q_des(R_des);

		// Initialize step variables
		Eigen::Matrix4d T_step = this->T_init;
		Eigen::Matrix3d R_step = T_step.block<3,3>(0,0);
		Eigen::Quaterniond Q_step(R_step);
		Eigen::Vector3d p_step;
			
		// Trapezoidal trajectory
		double L = (p_des - p_init).norm();
		std::vector<double> steps = this->evalTrapezoidalTrajectory(increment, L); 
		p_step = p_init + steps[0] * (p_des - p_init) / L;		
		this->vRef.head(3) = ((p_des - p_init) / L) * steps[1];
		this->aRef.head(3) = ((p_des - p_init) / L) * steps[2];

		// Interpolate points
		if ( increment < T ){
			// Simple trajectory
			//p_step = p_init + increment * (p_des - p_init) / T;

			// Orientation step	(an alternative to quaternion interpolation could be AngleAxis trajectory)
			Q_step = Q_init.slerp(increment/T, Q_des);
		}
		// Append last desired point to trajectory if time has exceeded the desired duration
		else{
			p_step = p_des;
			Q_step = Q_des;
		}
		
		// Get rotation matrix from quaternion
		Q_step.normalize();
		R_step = Q_step.toRotationMatrix();
	
		// Update T step
		T_step.block<3,1>(0,3) = p_step;
		T_step.block<3,3>(0,0) = R_step;		

		// Append to trajectory (WARNING: this vector can be very long for very long tasks)
		this->Tref_vector.push_back(T_step);
  	}
	else{
		this->Tref_vector.push_back(this->T_des);
		//std::cout << Tref_vector[i] << std::endl;
	}
}

void Controller::initCoppelia(){

	// Set simPort
	this->simPort = 19997; 

	// New instance of Coppelia
	std::cout << "Creating Coppelia instance in Controller" << std::endl;
	this->coppelia = new VREPProxy();
	this->coppelia->setIPAddress(std::string("127.0.0.1"));

	// Initialize Coppelia
	this->coppelia->init();

	// Set synchro to true
	this->coppelia->setSynchro(true);
}

void Controller::mainLoop(){

	Eigen::VectorXd u, tau_c, g;

	// Time variables
	double dt, Ts, des_rate;
	float t_curr, t_prev, elapsed;
	Timer clock;
	int Tsus, counter, iterations;
	
	// Initialize local variables
	int jointNum = this->robot->getJointNum();
	u.setZero(jointNum);
	g.setZero(jointNum);
	des_rate = 1000.0;
	clock.setRate(des_rate);
	Ts = 1.0 / des_rate;
	dt = Ts;
	Tsus = (int)(Ts * 1e6);
	t_curr = 0.0;
	t_prev = 0.0;

	// At this point, the Controller should initialize structures related to the robot.
	// Therefore, as long as the robot instance is not initialized by other threads (SystemManager), Controller must wait
	this->waitForStartRequest();

	// Set residual offset
	tau_c = this->robotProxy->getRobotState().extEstJointTorque.cast<double>();
	this->robot->setResidualVectorOffset(tau_c);

	// Counter to update trajectory
	counter = 0;

	// Counter to update dtAvg
	iterations = 0;

	// Update the full state of the robot with the current measurements
	this->updateRobot();

	std::cout << "Controller::mainLoop() started" << std::endl;

	// Assign first control input
	Eigen::VectorXd ginit = this->robot->getModelDynParams().g;
	u = ginit;
	g = ginit;

	// Start the main loop
	auto start = std::chrono::high_resolution_clock::now();

	while (this->ok()) {

		// Measure starting time
		auto tic_ = std::chrono::high_resolution_clock::now();

		// Update the full state of the robot with the current measurements
		this->updateRobot();

		// Check new data timestamp from RobotInterface
		t_curr = robot->getDataTimestamp();
		g = this->robot->getModelDynParams().g;

		// If torque control compensate gravity
		if (this->torqueCtrl == true){ u = g;}
		// else set zero velocity
		else{ u.setZero(); }

		elapsed = t_curr - t_prev;
		// Important! Evaluate only if the robot configuration has changed!
		if (elapsed != 0.0) { 

			// Update the sample time
			if (elapsed > 0.01) {
				elapsed = static_cast<float>(this->ctrlDt);
			}

			//this->strategy->setSampleTime(dt);
			this->robot->setSampleTime(static_cast<float>(dt));

			// Update trajectory
			this->generateTrajectoryOnline();
			this->updateT_ref(counter);

			// Call the control law method
			if (this->torqueCtrl == false){	
				u = this->controlLaw_Admittance();
			}
			else{ 
				u = this->controlLaw_PIDxGravity();
				//u = this->controlLaw_Impedance(); 
			}
			

			// If torque control update the commanded torques in robot
			if (this->torqueCtrl == true){ this->robot->setCmdJointTorques(u); }
			// else update the commanded velocities
			else{ this->robot->setCmdJointVelocity(u); }
			
			// Update prev variables
			t_prev = t_curr;

			// Update counter to update reference 
			counter ++;
		}

		// If torque control update the commanded torques in robotProxy
		if (this->torqueCtrl == true){ u -= g; this->robotProxy->setStampedJointTorqueCommands(this->time_, u.cast<float>()); }
		// else update the commanded velocities
		else{ this->robotProxy->setJointVelocityCommands(u.cast<float>()); }

		// Measure the ending time and the elapsed time
		auto toc_ = std::chrono::high_resolution_clock::now();
		auto tictoc_ = round<std::chrono::microseconds>(toc_ - tic_).count();
		std::this_thread::sleep_for(std::chrono::microseconds(Tsus - tictoc_));

		// Measure dt
		auto tac_ = std::chrono::high_resolution_clock::now();
		dt = std::chrono::duration_cast<std::chrono::microseconds>(tac_ - tic_).count() * 1e-6;

		// Update total time
		this->time_ += dt;
		iterations++;
		this->dtAvg = this->time_ / iterations;
		//this->ctrlRateSS << 1.0 / dt << ";" << std::endl;
		//std::cout << "[Ctrl] dt = " << dt << std::endl;
		//std::cout << "[Ctrl] time_ = " << time_ << std::endl;
		//std::cout << "[Ctrl] dtAvg = " << this->dtAvg << std::endl;
	}

	std::cout << "Leaving Controller::mainLoop() ... " << std::endl;

}

Eigen::VectorXd Controller::torqueSaturation(Eigen::VectorXd tau){
	double tMax1 = 20;//87.0;
	double tMax2 = 5;//12;
	double tMax = -1;
	for (int i = 0; i < this->jointNum; ++i) {
		if (i < 4)
			tMax = tMax1;
		else
			tMax = tMax2;
		if (abs(tau(i)) >= tMax)
			if (tau(i) >= 0) tau(i) = tMax;
			else 	    	 tau(i) = -tMax;
		//std::cout << "commanded tau" << i << " :" << tau(i) << std::endl;
	}
	return tau;
}

void Controller::updateRobot(const bool& online) {

	Eigen::VectorXd q, qd, tau, tau_c;
	Eigen::Vector6d fext, fext_hat;
	Eigen::MatrixXd J = this->robot->getJacobian();
	Eigen::MatrixXd JT = J.transpose();
	Eigen::MatrixXd JTpinv = (J * JT).inverse() * J;
	double jointNum = this->robot->getJointNum();

	q.setZero(static_cast<Eigen::Index>(jointNum));
	qd.setZero(static_cast<Eigen::Index>(jointNum));
	tau.setZero(static_cast<Eigen::Index>(jointNum));
	tau_c.setZero(static_cast<Eigen::Index>(jointNum));

	alba::RobotState rs = this->robotProxy->getRobotState();
	q = rs.msrJointPosition.cast<double>();
	qd = rs.msrJointVelocity.cast<double>();
	tau = rs.msrJointTorque.cast<double>();
	tau_c = rs.extEstJointTorque.cast<double>();
	fext = rs.extEstCartForce.cast<double>();
	
	this->robot->setDataTimestamp(static_cast<float>(this->time_));

	// ??? No more used ???
	tau_c -= this->robot->getResidualVectorOffset();		
	fext_hat = JTpinv * tau_c;

	// Update the full state of the robot with the current measurements
	this->robot->updateRobotState(q, qd, tau, online);
	//this->robot->setMsrForce(fext_hat);

	// Update robot in Coppelia (used only as player for now)
	this->coppelia->setJointPosition(q, this->simPort, simx_opmode_oneshot);

	// If you are working on the real Franka robot, use the proprietary matrices of the dynamic model and ovveride the previously computed ones	
	// Initialize proprietary matrices
	Eigen::VectorXf glib, clib;
	Eigen::MatrixXf mlib;

	// Get proprietary matrices
	glib = this->robotProxy->getRobotState().g_lib;
	clib = this->robotProxy->getRobotState().c_lib;
	mlib = this->robotProxy->getRobotState().m_lib;

	// Set proprietary matrices
	this->robot->setDynGravityVector(glib.cast<double>());
	this->robot->setDynCoriolisVector(clib.cast<double>());
	this->robot->setDynMassMatrix(mlib.cast<double>());
	
	// Compute and set external forces 
	this->robot->computeResidualFull(q, qd, tau);
	Eigen::VectorXd res = this->robot->getResidualVector();
	fext = this->robot->staticForceReconstruction(res);
	this->robot->setMsrForce(fext);
}

void Controller::updateT_ref(int counter){
	
	// Update reference trajectgory until the desired position has been reached
	if ( counter < this->Tref_vector.size() ){ this->T_ref = this->Tref_vector[counter]; } 
	else{ this->T_ref = this->Tref_vector.back(); }

}

Eigen::Vector7d Controller::velocitySaturation(Eigen::Vector7d q_dot){
	double qdMax1 = 2.1750;
	double qdMax2 = 2.6100;
	double qdMax = -1;
	for (int i = 0; i < 7; ++i) {
		if (i < 4)
			qdMax = qdMax1;
		else
			qdMax = qdMax2;
		if (abs(q_dot(i)) >= qdMax)
			if (q_dot(i) >= 0) q_dot(i) = qdMax;
			else 	    	 q_dot(i) = -qdMax;
		
	}
	return q_dot;
}

void Controller::waitForStartRequest() {

	Timer clock;

	bool condition = (!this->started && this->ok() && !(this->robotProxy->getRobotState().initialized));

	// Wait for external threads to set started on true
	while (condition) { 
		clock.timeSleep(0.01);
		condition = (!this->started && this->ok() && !(this->robotProxy->getRobotState().initialized));
	}

}


// Get functions
Eigen::Vector3d Controller::getP_des(){

	Eigen::Vector3d pDes;
	pDes.setZero();

	pDes = T_des.block<3, 1>(0, 3);

	return pDes;

}

Eigen::Matrix3d Controller::getR_des(){

	Eigen::Matrix3d RDes;
	RDes.setZero();

	RDes = T_des.block<3, 3>(0, 0);
	
	return RDes;

}

Eigen::Vector3d Controller::getP_ref(){

	Eigen::Vector3d pRef;
	pRef.setZero();

	pRef = this->T_ref.block<3, 1>(0, 3);

	return pRef;

}

Eigen::Matrix3d Controller::getR_ref(){

	Eigen::Matrix3d RRef;
	RRef.setZero();

	RRef = T_ref.block<3, 3>(0, 0);
	
	return RRef;

}


// Set functions
void Controller::setTaskOption(bool position_, bool orientation_, bool tracking_){
	this->position = position_;
	this->orientation = orientation_;
	this->tracking = tracking_;
}

void Controller::setT_des() {

	// Transformation is written by rows

	// Init to identity
	this->T_des.setIdentity();

	// Read 0_T_ee_desired.ini
	mINI::INIFile file("../../0_T_ee_desired.ini");
	mINI::INIStructure ini;
	file.read(ini);

	// Return a copy of the desired matrix
	std::string TDES = ini.get("desired").get("t");

	try {
		if (TDES.size() > 1) {
			// Choose delimiter
			const char* delim = " ";

			// Convert to vector
			std::vector<std::string> out;
			tokenize(TDES, delim, out);

			// Assign desired matrix
			// Rdes
			this->T_des(0, 0) = stod(out[0]);
			this->T_des(0, 1) = stod(out[1]);
			this->T_des(0, 2) = stod(out[2]);

			this->T_des(1, 0) = stod(out[4]);
			this->T_des(1, 1) = stod(out[5]);
			this->T_des(1, 2) = stod(out[6]);

			this->T_des(2, 0) = stod(out[8]);
			this->T_des(2, 1) = stod(out[9]);
			this->T_des(2, 2) = stod(out[10]);

			// pDes
			this->T_des(0, 3) = stod(out[3]);
			this->T_des(1, 3) = stod(out[7]);
			this->T_des(2, 3) = stod(out[11]);

			std::cout << "\nDesired 0_T_ee:\n" << this->T_des << std::endl;
		}
		else {
			throw 404;
		}
	}
	catch(int err) {
		std::cout << "\n[ERROR]\nCannot find desired 0_T_ee." << err << std::endl;
	}
}


// TRASH
/*void Controller::generateTrajectoryOffline(){

	// "Duration"
	double T = 1.0; 

	if (this->tracking == true){

		// Get the current 0_T_ee and use it as first point of the trajectory
		this->Tref_vector.push_back(this->T_init);

		// Get initial and desired Cartesian positions
		Eigen::Vector3d p_init;
		Eigen::Vector3d p_des;
		p_init.setZero();
		p_des.setZero();
		p_init = this->T_init.block<3,1>(0,3);
		p_des = this->T_des.block<3,1>(0,3);

		// Get initial and desired rotation matrix
		Eigen::Matrix3d R_init = T_init.block<3,3>(0,0);
		Eigen::Matrix3d R_des = T_des.block<3,3>(0,0);

		// Compute quaternions from rotation matrices
		Eigen::Quaterniond Q_init(R_init); 
		Eigen::Quaterniond Q_des(R_des);

		// Initialize step variables
		Eigen::Matrix4d T_step = this->T_init;
		Eigen::Matrix3d R_step = T_step.block<3,3>(0,0);
		Eigen::Quaterniond Q_step(R_step);
		Eigen::Vector3d p_step;

		// Problems with synchronization of dt used here, meaning ctrlDt*10, and the one actually measured in Controller::MainLoop
		// I think this is the reason why Marco did not generate the trajectory offline but updated them online with the measured dt
		for (double t = 0.0; t <= T; t += this->ctrlDt*10){
			
			//Postion
			p_step = p_init + t * (p_des - p_init) / T;

			// Orientation step			
			Q_step = Q_init.slerp(t/T, Q_des);

			// Get rotation matrix from quaternion
			Q_step.normalize();
			R_step = Q_step.toRotationMatrix();
		
			// Update T step
			T_step.block<3,1>(0,3) = p_step;
			T_step.block<3,3>(0,0) = R_step;		

			// Append to trajectory
			this->Tref_vector.push_back(T_step);
		}

  	}
	else{

		for (double t = 0.0; t <= T; t += this->ctrlDt){
			this->Tref_vector.push_back(this->T_des);
			//std::cout << Tref_vector[i] << std::endl;
		}
	}

}*/


