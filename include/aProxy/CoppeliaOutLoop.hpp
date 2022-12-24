#ifndef COPPELIA_OUT_LOOP_HPP_
#define COPPELIA_OUT_LOOP_HPP_

// System Header files
#include <iostream>
#include <map>

// V-REP header files
extern "C" {
	#include "extApi.h"
}

// Eigen Header files
#include <Eigen\Dense>

// Project Header files
#include "FrankaProxy.hpp"


class CoppeliaOutLoop{ 

public:

	/**
	* @brief Default constructor of the CoppeliaOutLoop class
	*
	*/
	CoppeliaOutLoop();

	/**
	* @brief Default destroyer of the CoppeliaOutLoop class
	*
	*/
	~CoppeliaOutLoop();

	/**
	*@brief Verify if the sensor is properly connected to the system and ready for communication
	*/
	void checkConnection();

	/**
	* @brief Get function
	* Get the active tool V-REP handler
	* @param the port at which we stay connected
	* @return the active tool V-REP handler
	*/
	inline int getClientID(const int& port) { return this->portIdMap[port]; }

	/**
	* @brief Init function
	* This function initializes the V-REP Remote APIs and opens the connection with the simulator
	*/
	void init();

	/**
	*@brief Returns if the sensor is available
	*@return flag stating if the sensor is present
	*/
	inline bool isSystemConnected(){ return this->connected; };

	void mainLoop();

	/**
	* @brief Set function
	* Set the joint position in the V-REP scene
	* @param configuration vector q
	* @param simport: the connected port
	* @param simx_opmode: the transmission modelity
	*/
	void setJointPosition(const Eigen::VectorXd& q, const int& simport, int simx_opmode);

	/**
	* @brief Set function
	* Set the IP address of the V-REP simulator server
	*/
	inline void setIPAddress(const std::string& ip_) { this->IPaddr = ip_; }

	/**
	* @brief Set function
	* Set the running flag
	* @param the running flag
	*/
	inline void setRunning(const bool& run_) { this->running = run_; }

	/**
	* @brief Set function
	* Set the synchro flag
	**/
	void setSynchro(bool synchro) { this->synchro = synchro; }

	/**
	* @brief Show function
	* Show the input point in the simulated scene as a dummy object
	* @param the 3D coordinates of the given point, in the vision sensor frame
	* @return the handle of the dummy object
	*/
	simxInt showPoint(const Eigen::Vector3d& p, const int& frame_ref, const int& simport);

	/**
	* @brief Show function
	* Show the input reference frame in the simulated scene as a dummy object
	* @param T: the 4x4 homogeneous transformation matrix representing the desired reference frame
	* @param color: the color of the dummy to be shown
	* @pararm dummyHandle: the handle of the dummy object to be shown. If -1, a dummy is created
	* @param simport: the CoppeliaSim connection port
	*/
	void showFrame(const Eigen::Matrix4d& T, const simxUChar* color, simxInt& dummyHandle, const int& simport);

	/**
	 * @brief To start/stop main loop the boolean whiteLEDTrack is used. Therefore the corresponding thread is joinable too
	 * 
	 */
	inline void startWhiteTrack(){this->whiteLEDTrack = true;};
	inline void stopWhiteTrack(){this->whiteLEDTrack = false;};

protected:

	int clientID;								//!< ID number of the V-REP Client remote API
	bool connected;								//!< If the sensor is available
	std::string IPaddr;							//!< IP address of the V-REP simulator server
	std::vector <int> jointsHandle;				//!< V-REP handler array of the robot joints (PANDA)
	std::map < int, int > portIdMap;			//!< Map of <port,id> V-REP clients
	FrankaProxy *robotProxy;					//|< Dynamic instance of the robotProxy class	
	bool running;								//!< Flag stating if the simulation is running	
	std::map < std::string, int > simObjects;	//!< Map of <name,id> V-REP objects
	bool synchro;								//!< Flag stating if the simulation is perfectly synchronized	
	bool whiteLEDTrack;							//!< Flag to exit from mainLoop
					
};

#endif // COPPELIA_OUT_LOOP_
