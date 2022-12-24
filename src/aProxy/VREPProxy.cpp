// Project Header files
#include "VREPProxy.hpp"
#include "Timer.hpp"
#include <cstdlib>

// Eigen Header files
#include <Eigen/Geometry>

VREPProxy::VREPProxy() {

	// Initialize the member variables with default values
	this->clientID = -1;
	this->connected = false;
	this->running = false;
	this->IPaddr = std::string("1.1.1.1");
	this->jointsHandle = {-1, -1, -1, -1, -1, -1, -1};
	this->dummyNum = 0;

	// Fill graph names
	graphNames.push_back("x_error");
	graphNames.push_back("y_error");
	graphNames.push_back("z_error");
	graphNames.push_back("a_error");
	graphNames.push_back("b_error");
	graphNames.push_back("g_error");

	graphTauNames.push_back("tau1");
	graphTauNames.push_back("tau2");
	graphTauNames.push_back("tau3");
	graphTauNames.push_back("tau4");
	graphTauNames.push_back("tau5");
	graphTauNames.push_back("tau6");
	graphTauNames.push_back("tau7");

	graphForceNames.push_back("forceX");
	graphForceNames.push_back("forceY");
	graphForceNames.push_back("forceZ");

	graphDeltaPr.push_back("deltaPr_x");
	graphDeltaPr.push_back("deltaPr_y");
	graphDeltaPr.push_back("deltaPr_z");
	graphDeltaPr.push_back("deltaPr_a");
	graphDeltaPr.push_back("deltaPr_b");
	graphDeltaPr.push_back("deltaPr_g");	
}

VREPProxy::~VREPProxy() {}

void VREPProxy::init() {

	//std::cout << "initialization..." << std::endl;

	// Close eventual pending open communication channels
	simxFinish(-1);

	//!< Check and open a new connection to V-REP (timeout after 5s)
	this->checkConnection();

	if (this->isSystemConnected()) { // If successfully connected ...

		// Set synchronous communication mode (MF: DO WE REALLY NEED IT?)
		if (this->synchro)
			simxSynchronous(this->clientID, 1);
		else
			simxSynchronous(this->clientID, false);		
	}

	simxGetObjectHandle(this->clientID, "Franka_base", &(this->panda_baseHdl), simx_opmode_blocking);
	simxGetObjectHandle(this->clientID, "Franka_connection", &(this->forceSensorHdl), simx_opmode_blocking);

	this->simObjects.insert({ "Franka_base",this->panda_baseHdl });
	this->simObjects.insert({ "Franka_connection",this->forceSensorHdl });

	// Load joint handle (for PANDA robot)
	int nJoints = 7;
	for (int i = 0;i < nJoints;++i) {
		std::string name = "Franka_joint" + std::to_string(i+1);
		simxGetObjectHandle(this->clientID, name.c_str(), &(this->jointsHandle[i]), simx_opmode_blocking);
		this->simObjects.insert({ name.c_str(),this->jointsHandle[i] });
	}
}

bool VREPProxy::addSimObject(const char* objName, const int& simport, const int& mode) {

	bool ret = false;
	int handle;
	const char* name = objName;
	simxGetObjectHandle(this->portIdMap[simport], name, &handle, simx_opmode_blocking);
	
	if (handle > 0 && this->simObjects.find(std::string(objName)) == this->simObjects.end()) {

		this->simObjects.insert({ std::string(objName),handle });
		ret = true;

	}

	return ret;
}

void VREPProxy::checkConnection() {

	this->portIdMap.insert({ 19997, -1 });
	//this->portIdMap.insert({ 19996, -1 });

	std::map< int, int >::iterator it;
	bool avail = true;

	for (it = this->portIdMap.begin(); it != this->portIdMap.end(); ++it) {
		it->second = simxStart((simxChar*)this->IPaddr.c_str(), it->first, true, true, 50, 5);

		if (it->second != -1) {
			std::cout << "Coppelia simulator at port " << it->first << " is successfully connected!" << std::endl;
			this->connected = true;
		}
		else {
			std::cout << "Could not connect to Coppelia at port " << it->first << ".Check the Coppelia settings... " << std::endl;
			this->connected = false;
		}
	}
	
	// Default client
	this->clientID = (this->portIdMap.begin())->second;	
	
}

bool VREPProxy::checkObjectCollision(const char* obj1_name, const char* obj2_name, const int& simport, const int& mode) {

	bool colliding = false;
	int obj1Handle, obj2Handle;
	simxUChar collisionState;

	obj1Handle = this->simObjects[obj1_name];
	obj2Handle = this->simObjects[obj2_name];

	simxCheckCollision(this->portIdMap[simport], obj1Handle, obj2Handle, &collisionState, mode);
	colliding = (bool)collisionState;

	return colliding;
}

float VREPProxy::checkObjectDistance(const char* obj1_name, const char* obj2_name, const int& simport, const int& mode) {

	float distance = 1000.0;
	int obj1Handle, obj2Handle;
	//simxUChar collisionState;

	obj1Handle = this->simObjects[obj1_name];
	obj2Handle = this->simObjects[obj2_name];

	simxCheckDistance(this->portIdMap[simport], obj1Handle, obj2Handle, &distance, mode);

	return distance;
}

void VREPProxy::close() {

	// Call the corresponding remote V-REP API to close the communication with V-REP
	//simxFinish(this->clientID);

	simxFinish(this->portIdMap[19997]);
	//simxFinish(this->portIdMap[19996]);
}

char* VREPProxy::errorConnectionMessage() {

	return "[V-REP] V-REP not detected. Cannot run the required functionalities.";

}

void VREPProxy::getFloatSignal(const char* signalName, float& value, const int& simport, const int& mode) {

	simxGetFloatSignal(this->portIdMap[simport], signalName, &value, mode);

}

void VREPProxy::getForceMsr(Eigen::VectorXd & forceMsr, int simx_opmode) {
	simxFloat force[3];
	simxFloat torque[3];
	unsigned char *state = nullptr;
	
	simxReadForceSensor(this->clientID, this->forceSensorHdl, state, force, torque, simx_opmode);

	// Check if the sensor data is available (state variable is always null....)
	if (abs(force[0]) > 1e6)
		forceMsr.setZero();
	else {
		forceMsr(0) = force[0];
		forceMsr(1) = force[1];
		forceMsr(2) = force[2];
		forceMsr(3) = torque[0];
		forceMsr(4) = torque[1];
		forceMsr(5) = torque[2];
	}
}

void VREPProxy::getJointPosition(Eigen::VectorXd& q, const int& simport, const int& simx_opmode){
	//iterate over the joint handles
	simxFloat *qf = new simxFloat[q.size()];
	for (int i = 0;i < this->jointsHandle.size();++i) {
		simxGetJointPosition(this->portIdMap[simport], this->jointsHandle[i], &qf[i], simx_opmode);
		q[i] = qf[i];
	}
	//release memory
	delete[] qf;
}

void VREPProxy::getJointTorque(Eigen::VectorXd & tau, const int& simport, const int& simx_opmode) {
	//iterate over the joint handles
	simxFloat *tauf = new simxFloat[tau.size()];

	//simxFloat tauf[7];
	for (int i = 0;i < this->jointsHandle.size();++i) {
		simxGetJointForce(this->portIdMap[simport], this->jointsHandle[i], &tauf[i], simx_opmode);
		tau[i] = tauf[i];
	}
	//release memory
	delete[] tauf;
}

void VREPProxy::getJointVelocity(Eigen::VectorXd & dq, const int& simport, const int& simx_opmode) {
	//iterate over the joint handles
	simxFloat *dqf = new simxFloat[dq.size()];
	//simxFloat dqf[7];
	for (int i = 0;i < this->jointsHandle.size();++i) {
		simxGetObjectFloatParameter(this->portIdMap[simport], this->jointsHandle[i], 2012, &dqf[i], simx_opmode);
		dq[i] = dqf[i];
	}
	//release memory
	delete[] dqf;
}

Eigen::Vector3f VREPProxy::getObjectPositionFromName(const char* obj_name, const char* ref_name, const int& simport, const int& mode) {

	Eigen::Vector3f p;

	int objHandle, refHandle;

	objHandle = this->simObjects[obj_name];
	refHandle = (!std::string(ref_name).empty()) ? this->simObjects[ref_name] : -1;

	simxGetObjectPosition(this->portIdMap[simport], objHandle, refHandle, &p(0), mode);

	return p;
}

Eigen::Matrix3f VREPProxy::getObjectOrientationFromName(const char* obj_name, const char* ref_name, const int& simport, const int& mode) {

	int objHandle, refHandle;
	Eigen::Matrix3f R;
	Eigen::Quaternionf q;
	objHandle = this->simObjects[obj_name];
	refHandle = (!std::string(ref_name).empty()) ? this->simObjects[ref_name] : -1;

	// q.coeffs() should have been stored in the order x, y, z, w, that should be the same as CoppeliaSim
	simxGetObjectQuaternion(this->portIdMap[simport], objHandle, refHandle, &q.coeffs()(0), mode);

	R = q.toRotationMatrix();
	return R;
}

void VREPProxy::getRobotBasePose(Eigen::Matrix4d& pose, const std::string& ref, int simx_opmode) {

	int handleRef = -1;	//world frame handle

	if (ref.compare("panda_base") == 0) {
		handleRef = this->panda_baseHdl;
		//std::cout << handleRef << '\n';
		//std::cout << "Getting the desired pose in the Panda base reference frame\n";
	}
	//reading pose from Vrep scene
	simxFloat position[3];
	simxGetObjectPosition(this->clientID, this->panda_baseHdl, handleRef, position, simx_opmode);
	simxFloat quaternion[4];
	simxGetObjectQuaternion(this->clientID, this->panda_baseHdl, handleRef, quaternion, simx_opmode);

	//assinging pose
	pose.setIdentity();
	pose(0, 3) = position[0];
	pose(1, 3) = position[1];
	pose(2, 3) = position[2];
	Eigen::Quaternionf q_eigen{ quaternion };
	Eigen::Matrix3d R{ (q_eigen.normalized().toRotationMatrix()).cast<double>() };
	pose.topLeftCorner(3, 3) = R;
}

void VREPProxy::sendErrorSignals(const Eigen::Matrix<double, 6, 1>& error, float det_JJT) {

	for (int i = 0; i < 6; i++) {
		double err = error(i);
		if (i >= 3)
			err = err *(180 / 3.14);
		simxInt ret = simxSetFloatSignal(this->clientID, (this->graphNames[i]).c_str(), static_cast<float>(error(i)), simx_opmode_oneshot);
	}
	simxSetFloatSignal(this->clientID, "det_JJT", det_JJT, simx_opmode_oneshot);
}

void VREPProxy::sendSignals(const Eigen::VectorXd& s, std::string& type) {
	std::vector<std::string> graphs;
	if (type.compare("tauDiff") == 0 || type.compare("tau_model") == 0 || type.compare("tau_meas") == 0)
		graphs = this->graphTauNames;
	else if (type.compare("forceMsr") == 0 || type.compare("forceDes") == 0)
		graphs = this->graphForceNames;
	else if (type.compare("deltaPr") == 0)
		graphs = this->graphDeltaPr;

	int length = static_cast<int>(s.size());
	for (int i = 0; i < length; i++) {
		if (type.compare("tau_model") == 0)
			simxInt ret = simxSetFloatSignal(this->clientID, graphs[i].append("_model").c_str(), static_cast<float>(s(i)), simx_opmode_oneshot);
		else if (type.compare("tau_meas") == 0)
			simxInt ret = simxSetFloatSignal(this->clientID, graphs[i].append("_meas").c_str(), static_cast<float>(s(i)), simx_opmode_oneshot);
		else if (type.compare("tauDiff") == 0)
			simxInt ret = simxSetFloatSignal(this->clientID, graphs[i].c_str(), static_cast<float>(s(i)), simx_opmode_oneshot);
		else if (type.compare("forceMsr") == 0)
			simxInt ret = simxSetFloatSignal(this->clientID, graphs[i].append("_measured_(N)").c_str(), static_cast<float>(s(i)), simx_opmode_oneshot);
		else if (type.compare("forceDes") == 0)
			simxInt ret = simxSetFloatSignal(this->clientID, graphs[i].append("_desired_(N)").c_str(), static_cast<float>(s(i)), simx_opmode_oneshot);
		else if (type.compare("deltaPr") == 0)
			simxInt ret = simxSetFloatSignal(this->clientID, graphs[i].c_str(), static_cast<float>(s(i)), simx_opmode_oneshot);
	}
}

void VREPProxy::setChildObject(const char* childName, const char* parentName, const int& simport, const int& mode) { 

	const char* name_c = childName;
	const char* name_p = parentName;
	std::cout << "name_c = " << name_c << std::endl;
	std::cout << "name_p = " << name_p << std::endl;
	simxSetObjectParent(this->portIdMap[simport], this->simObjects[std::string(name_c)], this->simObjects[std::string(name_p)], true, mode);

}

void VREPProxy::setFloatSignal(const char* signalName, const float& value, const int& simport, const int& mode) {

	simxSetFloatSignal(this->portIdMap[simport], signalName, value, mode);

}

void VREPProxy::setJointPosition(const Eigen::VectorXd& q, const int& simport, int simx_opmode) {

	//iterate over the joint handles
	for (int i = 0;i < this->jointsHandle.size();++i) {
		//simxSetJointPosition(this->clientID, this->jointsHandle[i], q[i], simx_opmode);
		simxSetJointPosition(this->portIdMap[simport], this->jointsHandle[i], static_cast<float>(q(i)), simx_opmode);
	}
}

void VREPProxy::setJointTargetPosition(Eigen::VectorXd& q, const int& simport, const int& simx_opmode) {
	//iterate over the joint handles
	for (int i = 0;i < this->jointsHandle.size();++i) {
		simxSetJointTargetPosition(this->portIdMap[simport], this->jointsHandle[i], static_cast<float>(q[i]), simx_opmode);
	}
}

void VREPProxy::setJointTargetVelocity(const Eigen::VectorXd & dq, const int& simport, const int& simx_opmode) {
	//iterate over the joint handles
	for (int i = 0;i < this->jointsHandle.size();++i) {
		simxSetJointTargetVelocity(this->portIdMap[simport], this->jointsHandle[i], static_cast<float>(dq[i]), simx_opmode);
	}
}

void VREPProxy::setJointTorque(const Eigen::VectorXd& tau, const int& simport, int simx_opmode) {

	//iterate over the joint handles
	for (int i = 0; i < this->jointsHandle.size(); ++i) {
		simxSetJointMaxForce(this->portIdMap[simport], this->jointsHandle[i], static_cast<float>(abs(tau[i])), simx_opmode_oneshot);
		if (tau[i] >= 0) {
			//simxSetJointTargetPosition(this->portIdMap[simport], this->jointsHandle[i], 1e3, simx_opmode_oneshot); // if CoppeliaSim joint control loop is enabled for this joint
			simxSetJointTargetVelocity(static_cast<simxInt>(this->portIdMap[simport]), static_cast<simxInt>(this->jointsHandle[i]), static_cast<simxFloat>(10e10),  static_cast<simxInt>(simx_opmode_oneshot)); // if CoppeliaSim joint control loop is NOT enabled for this joint
		}
		else {
			//simxSetJointTargetPosition(this->portIdMap[simport], this->jointsHandle[i], -1e3, simx_opmode_oneshot); // if CoppeliaSim joint control loop is enabled for this joint
			simxSetJointTargetVelocity(static_cast<simxInt>(this->portIdMap[simport]), static_cast<simxInt>(this->jointsHandle[i]), static_cast<simxFloat>(-10e10), static_cast<simxInt>(simx_opmode_oneshot)); // if CoppeliaSim joint control loop is NOT enabled for this joint
		}
	}
}

int VREPProxy::setObjectOrientationFromName(const char* obj_name, const char* ref_name, const Eigen::Vector3f& abg, const int& simport, const int& mode) {

	int ret = -1;
	int objHandle, refHandle;

	objHandle = this->simObjects[obj_name];
	refHandle = (!std::string(ref_name).empty()) ? this->simObjects[ref_name] : -1;

	// q.coeffs() should have been stored in the order x, y, z, w, that should be the same as CoppeliaSim
	ret = simxSetObjectOrientation(this->portIdMap[simport], objHandle, refHandle, &abg(0), mode);

	return ret;

}

int VREPProxy::setObjectPositionFromName(const char* obj_name, const char* ref_name, const Eigen::Vector3f& p, const int& simport, const int& mode) {

	int ret = -1;
	int objHandle, refHandle;

	objHandle = this->simObjects[obj_name];
	refHandle = (!std::string(ref_name).empty()) ? this->simObjects[ref_name] : -1;

	std::cout << "obj_name = " << obj_name << std::endl;
	std::cout << "objHandle = " << objHandle << std::endl;
	std::cout << "p = " << p.transpose() << std::endl;
	std::cout << std::endl;

	ret = simxSetObjectPosition(this->portIdMap[simport], objHandle, refHandle, &p(0), mode);

	return ret;

}

int VREPProxy::setObjectRotationMatrixFromName(const char* obj_name, const char* ref_name, const Eigen::Matrix3f& R, const int& simport, const int& mode) {

	int ret = -1;
	int objHandle, refHandle;
	Eigen::Quaternionf q(R);
	
	objHandle = this->simObjects[obj_name];
	refHandle = (!std::string(ref_name).empty()) ? this->simObjects[ref_name] : -1;

	// q.coeffs() should have been stored in the order x, y, z, w, that should be the same as CoppeliaSim
	ret = simxSetObjectQuaternion(this->portIdMap[simport], objHandle, refHandle, &q.coeffs()(0), mode);

	return ret;

}

void VREPProxy::showFrame(const Eigen::Matrix4d& T, const simxUChar* color, simxInt& dummyHandle, const int& simport) {

	simxFloat pos[3], quat[4]; //ori[3], 
	Eigen::Matrix3d R;
	Eigen::Quaterniond qeig;

	pos[0] = static_cast<float>(T(0, 3));
	pos[1] = static_cast<float>(T(1, 3));
	pos[2] = static_cast<float>(T(2, 3));

	R = T.topLeftCorner(3, 3);
	qeig = Eigen::Quaterniond(R);
	quat[0] = static_cast<float>(qeig.x());
	quat[1] = static_cast<float>(qeig.y());
	quat[2] = static_cast<float>(qeig.z());
	quat[3] = static_cast<float>(qeig.w());

	// Create the dummy
	if (dummyHandle == -1) {
		simxCreateDummy(this->portIdMap[simport], static_cast<simxFloat>(0.01), color, &dummyHandle, static_cast<simxInt>(simx_opmode_blocking));
	}

	// Set position of the dummy
	simxSetObjectPosition(this->portIdMap[simport], dummyHandle, this->panda_baseHdl, pos, static_cast<simxInt>(simx_opmode_oneshot));

	// Set orientation of the dummy
	simxSetObjectQuaternion(this->portIdMap[simport], dummyHandle, this->panda_baseHdl, quat, static_cast<simxInt>(simx_opmode_oneshot));

}

simxInt VREPProxy::showPoint(const Eigen::Vector3d& p, const int& frame_ref, const int& simport) {

	simxFloat pos[3];
	simxInt dummyHandle;

	pos[0] = static_cast<float>(p(0));
	pos[1] = static_cast<float>(p(1));
	pos[2] = static_cast<float>(p(2));

	int refHdl = this->panda_baseHdl;

	// Create the dummy
	simxCreateDummy(this->portIdMap[simport], static_cast<simxFloat>(0.005), NULL, &dummyHandle, static_cast<simxInt>(simx_opmode_blocking));

	// Set position of the dummy
	simxSetObjectPosition(this->portIdMap[simport], dummyHandle, refHdl, pos, static_cast<simxInt>(simx_opmode_blocking));

	// Return the handle of the dummy
	return dummyHandle;	
}

void VREPProxy::startSim(const int& simport) {

	// Call the corresponding remote V-REP API to start the simulation
	simxStartSimulation(this->portIdMap[simport], simx_opmode_blocking);

	// Set running flag on true
	this->setRunning(true);
}

void VREPProxy::stopSim(const int& simport) {

	// Call the corresponding remote V-REP API to stop the simulation
	simxStopSimulation(this->portIdMap[simport], simx_opmode_blocking);

	// Set running flag on false
	this->setRunning(false);
}

