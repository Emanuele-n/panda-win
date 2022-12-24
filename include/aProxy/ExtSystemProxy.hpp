#ifndef EXT_SYSTEM_PROXY_HPP_
#define EXT_SYSTEM_PROXY_HPP_

// Standard Header files
#include <string>
#include <iostream>

/******************* vvv Read below  vvv *********************/
/* 
* It seems that the following #define BOOST_THREAD_DYN_LINK
* is fundamental to make everything work, at least on the KUKA
* LAB PC (running Windows 7, VS2015 v140 with Boost 1.66.0).
* If not inserted, the program builds successfully but fails at
* startup, notifying a very generic error (0xc000007b), making 
* extremely hard to catch the reason of the issue.
* However, this didn't seem to be a problem when running the 
* program on Marco's laptop (Windows 10, same VS and Boost 
* version).
*/
/******************* ^^^ Read above  ^^^ *********************/
//#define BOOST_THREAD_DYN_LINK
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>

class ExtSystemProxy {

public:

	/**
	* @brief Default constructor
	*/
	ExtSystemProxy() {
	
		this->available = false;
		this->running = false;
	
	}

	/**
	* @brief Default destroyer
	*/
	~ExtSystemProxy() {}

	/**
	* @brief Default init function (here virtual)
	*/
	virtual void init() = 0;

	/**
	* @brief Default run function (here virtual)
	*/
	virtual void mainLoop() = 0;

	/**
	* @brief Default clear function (here virtual)
	*/
	virtual void clear() = 0;

	/**
	* @brief Default thread start function
	*/
	inline void startThread() { this->proxy_thread = boost::thread(boost::bind(&ExtSystemProxy::mainLoop, this)); };

	/**
	* @brief Join thread function
	* Join the previously launched thread for the current ExtSystemProxy object
	*/
	inline void joinThread() { this->proxy_thread.join(); }

	/**
	* @breif Get function
	* Retrieves the pointer to the thread object
	*
	*/
	inline boost::thread* getThreadObjPtr() { return &(this->proxy_thread); }

	/**
	* @brief Check function
	* Check if the external system is available
	* @return true if the external system is available
	*/
	inline bool isAvailable() { return this->available; }

	/**
	* @brief Check function
	* Check if the main loop of the external system is running
	* @return true if the main loop of the external system is running
	*/
	inline bool isRunning() { return this->running; }

	/**
	* @brief Set function
	* Set the available flag
	* @param the value of the available flag to be set
	*/
	inline void availability(const bool& aval) { this->available = aval; }

	/**
	* @brief Set function
	* Set the running flag
	* @param the value of the running flag to be set
	*/
	inline void setRunning(const bool& running_) { this->running = running_; }

	/**
	* @brief External Reset function
	* Set the flag to terminate the Geoamgic thread from an external source
	*/
	inline void extReset() { this->running = false; }


protected:

	bool available;						//!< Flag stating if the external system is available
	bool running;						//!< Flag stating if the main loop of the system is running

	boost::thread proxy_thread;

};


#endif // EXT_SYSTEM_PROXY_HPP_