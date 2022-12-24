// Project Header files
#include "CoppeliaOutLoop.hpp"

CoppeliaOutLoop::CoppeliaOutLoop() {

	// Initialize the member variables with default values
	this->clientID = -1;
	this->connected = false;
	this->running = false;
	this->IPaddr = std::string("127.0.0.1");
	this->jointsHandle = {-1, -1, -1, -1, -1, -1, -1};

}

CoppeliaOutLoop::~CoppeliaOutLoop() {}

void CoppeliaOutLoop::checkConnection() {

	this->portIdMap.insert({ 19997, -1 });

	std::map< int, int >::iterator it;
	bool avail = true;
	for (it = this->portIdMap.begin(); it != this->portIdMap.end(); ++it) {
		it->second = simxStart((simxChar*)this->IPaddr.c_str(), it->first, true, true, 50, 5);
		if (it->second != -1) {
			std::cout << "Coppelia simulator at port " << it->first << " is successfully connected!" << std::endl;
			this->connected = true;
		}
		else {
			std::cout << "Could not connect to V-REP at port " << it->first << ".Check the V - REP settings... " << std::endl;
			this->connected = false;
		}
	}

	// Default client
	this->clientID = (this->portIdMap.begin())->second;
	
}


void CoppeliaOutLoop::init() {

	//std::cout << "\n[COPPELIA]\nCoppelia initialization..." << std::endl;

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

	int nJoints = 7;
	for (int i = 0;i < nJoints;++i) {
		std::string name = "Franka_joint" + std::to_string(i+1);
		simxGetObjectHandle(this->clientID, name.c_str(), &(this->jointsHandle[i]), simx_opmode_blocking);
		this->simObjects.insert({ name.c_str(),this->jointsHandle[i] });
	}

	this->robotProxy = new FrankaProxy;
	this->robotProxy->panda = new franka::Robot("172.16.0.2");
	franka::RobotState current_state = this->robotProxy->panda->readOnce();
	Eigen::Map<const Eigen::Vector7d> q_init(current_state.q.data());
	//std::cout << "q_init = " << q_init.transpose() << std::endl;

}

void CoppeliaOutLoop::mainLoop(){

	while(this->whiteLEDTrack){

		// This is only used to track the robot in Coppelia if LED are white since readOnce() or read() can only be called if out of control loop
		// Get current configuration
		franka::RobotState current_state = this->robotProxy->panda->readOnce();
		Eigen::Map<const Eigen::Vector7d> q(current_state.q.data());
			
		// Send to Coppelia
		this->setJointPosition(q, 19997, simx_opmode_oneshot);
	}
	//std::cout << "\nOut of main loop for white led tracking\n" << std::endl;
}



void CoppeliaOutLoop::setJointPosition(const Eigen::VectorXd& q, const int& simport, int simx_opmode) {

	// Iterate over the joint handles
	for (int i = 0;i < this->jointsHandle.size();++i) {
		simxSetJointPosition(this->portIdMap[simport], this->jointsHandle[i], static_cast<simxFloat>(q(i)), simx_opmode);
	}
}


