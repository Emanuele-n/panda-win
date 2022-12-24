// System Header files
#include <iostream>

// Project Header files
#include "utils.hpp"
#include "ini.h"

bool extractEigenTransform(const std::vector < std::string >& content, Eigen::Matrix4d& T) {

	std::string line;
	std::string comma(",");
	std::string semicolumn(";");
	int commaPos = -1;

	if (content.size() != 4) {
		std::cout << "ERROR: Wrong numbers of lines detected in the Registered Transformation file." << std::endl;
		return false;
	}

	for (int i = 0; i < content.size(); i++) {

		std::string val[4];
		line = content[i];

		for (int j = 0; j < 4; j++) {
			commaPos = static_cast<int>(line.find(comma));
			val[j] = line.substr(0, commaPos);
			line = line.substr(commaPos + 1, line.length());

			T(i, j) = std::stod(val[j]);
		}

	}
	return true;
}

void ind2sub(const int& c, const int& idx, int& i, int& j) {

	i = idx / c;
	j = idx % c;

}

int Menu(){

	// Print user options
	printMenu();

	// Iterate until a correct option is selected
	int maxentry = 4;
	int option = -1000;
	while (option < 0 || option > maxentry){
	
		if (option != -1000){
			std::cout << "\nWrong input" << std::endl;
			printMenu();
		}

		// Wait for user input
		std::cin >> option;

	}
	return option;
}

std::vector < double > parseCSVLine(const std::string& line) {

	std::vector < double > vec;
	std::string comma(",");
	std::string semicolumn(";");

	std::string temp_line = line;
	bool endloop = false;
	while (!endloop && temp_line.length() > 0) {

		// Find the comma
		int delimiter = static_cast<int>(temp_line.find(comma));

		// If not found, look for the semicolumn
		if (delimiter == std::string::npos) {
			delimiter = static_cast<int>(temp_line.find(semicolumn));
			endloop = true;
		}

		// Assign the j-th value
		vec.push_back(std::stod(temp_line.substr(0, delimiter)));

		// Update the line
		temp_line = temp_line.substr(delimiter + 1, temp_line.length());
	}
	return vec;
}

void printHelp(){
    std::cout << "\n[HELP]" << std::endl;
    std::cout << "The default initial configuration is  [0, -pi/4, 0, -3 * pi/4, 0, pi/2, pi/4]" << std::endl;
    std::cout << "Default options:\n torque contorl (-d) \n trajectory tracking (-t) \n both position and orientation control\n" << std::endl;
    std::cout << "The desired transformation 0_T_ee is from the base of the robot to the tip of the last joint and it can be set in 0_T_ee_desired.ini." << std::endl;
    std::cout << "To help this process you can move the robot manually and save the transformation with option 3. Get current 0_T_ee" << std::endl;
    std::cout << "\n[PARAMETERS]" << std::endl;
    std::cout << "-r\tRun regulation task" << std::endl;
    std::cout << "-t\tRun tracking task" << std::endl;
    std::cout << "-k\tKinematic (velocity) control" << std::endl;
    std::cout << "-d\tDynamic (torque) control" << std::endl;
    std::cout << "-p\tPosition control only" << std::endl;
    std::cout << "-o\tOrientation control only" << std::endl;
}

void printMenu(){

	std::cout << ""
		"\nPlease select the desired functionality:  \n" 
		" 1. Initial Configuration \n" <<
        " 2. Cartesian Task \n" <<
        " 3. Get current 0_T_ee\n" <<
		" 0. Exit the program.\n" << std::endl;
}

void printporks(bool position, bool orientation, bool tracking, bool torque, bool simulation){
	std::cout << "\nActive options" << std::endl;
	if (position){ std::cout << " Position control: ON" << std::endl; }
	else{ std::cout << " Position control: OFF" << std::endl; }
	if (orientation){ std::cout << " Orientation control: ON" << std::endl; }
	else{ std::cout << " Orientation control: OFF" << std::endl; }
	if (tracking){ std::cout << " Task type: tracking" << std::endl; }
	else{ std::cout << " Task type: regulation" << std::endl; }
	if (torque){ std::cout << " Control level: torque" << std::endl; }
	else{ std::cout << " Control level: velocity" << std::endl; }
	if (simulation){ std::cout << " Simulation mode: ON" << std::endl; }
	else{ std::cout << " Simulation mode: OFF" << std::endl; }
}

Eigen::Vector3d rpy_last = Eigen::Vector3d::Zero();

Eigen::Vector3d rot2rpy(const Eigen::Matrix3d& R, Eigen::Vector3d& old){

	Eigen::Vector3d rpy_p, rpy_m;
	double pitch_p = atan2(-R(2, 0), sqrt(R(2, 1)*R(2, 1) + R(2, 2)*R(2, 2)));
	double pitch_m = atan2(-R(2, 0), -sqrt(R(2, 1)*R(2, 1) + R(2, 2)*R(2, 2)));

	if (abs(abs(pitch_p) - M_PI / 2) < 0.001) {
		std::cout <<  "ERROR: inverse kinematic singularity" << '\n';
	}

	double roll_p = atan2(R(2, 1) / cos(pitch_p), R(2, 2) / cos(pitch_p));
	double roll_m = atan2(R(2, 1) / cos(pitch_m), R(2, 2) / cos(pitch_m));

	double yaw_p = atan2(R(1, 0) / cos(pitch_p), R(0, 0) / cos(pitch_p));
	double yaw_m = atan2(R(1, 0) / cos(pitch_m), R(0, 0) / cos(pitch_m));

	/*rpy_p << roll_p, pitch_p, yaw_p;
	rpy_m << roll_m, pitch_m, yaw_m;//*/

	rpy_p << yaw_p, pitch_p, roll_p;
	rpy_m << yaw_m, pitch_m, roll_m;//*/

	/*if (old != Eigen::Vector3d(1, 1, 1)) {
		if ((rpy_p - old).norm() < (rpy_m - old).norm()) {
			old = rpy_p;
			std::cout << "positive amgles " << old.transpose() << std::endl;
			return rpy_p;
		}
		else {
			old = rpy_m;
			std::cout << "negative amgles " << old.transpose() << std::endl;
			return rpy_m;
		}
	}
	else {	
		return rpy_p;
	}//*/
	return rpy_p;
}

Eigen::Vector3d rot2zyx(const Eigen::Matrix3d& R) {

	Eigen::Vector3d zyx;

	float sy = static_cast<float>(sqrt(R(0,0)*R(0, 0) + R(1,0)*R(1, 0)));

	//Calculate Euler angles
	//zyx << atan2(R(2,1), R(2,2)), atan2(-R(2,0), sy), atan2(R(1,0), R(0,0));
	zyx << atan2(R(1, 0), R(0, 0)), atan2(-R(2,0), sy), atan2(R(2, 1), R(2, 2)) ;

	return zyx;

}

Eigen::Matrix3d rpy2rot(const Eigen::Vector3d& rpy) {
	Eigen::Matrix3d R;
	/*R = Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ())
		* Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX());//*/

	R = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitZ())
		* Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitX());

	return R;
}

// TODO: solve caps lock problem
void saveT(std::array<double, 16> O_T_EE){

	std::vector<std::string> tempTvec;
	std::string savedT;

	// T is saved by columns in a single array in libfranka.
	// Hence here it is transformed into the string following the same format as in Controller::setT_des
	for (int i = 0; i < 13; i+=4){
		tempTvec.push_back(std::to_string(O_T_EE[i]));
	}
	for (int i = 1; i < 14; i+=4){
		tempTvec.push_back(std::to_string(O_T_EE[i]));
	}
	for (int i = 2; i < 15; i+=4){
		tempTvec.push_back(std::to_string(O_T_EE[i]));
	}
	
	for (int i = 0; i < tempTvec.size(); i++){
		savedT += " ";
		savedT += tempTvec[i];
	}

	//std::cout << savedT << std::endl;

	// Save transformation
	mINI::INIFile file("../../0_T_ee_desired.ini");
	mINI::INIStructure ini;
	file.read(ini);

	ini["desired"]["t"] = savedT;

	file.write(ini);
}

Eigen::Matrix3f skew(const Eigen::Vector3f& v) {

	Eigen::Matrix3f S;

	S << 0, -v(2), v(1),
		v(2), 0, -v(0),
		-v(1), v(0), 0;

	return S;
}

int sub2ind(const int& c, const int& i, const int& j) {

	return j + i * c;

}

void tokenize(std::string const &str, const char* delim, std::vector<std::string> &out){
    char *token = strtok(const_cast<char*>(str.c_str()), delim);
    while (token != nullptr)
    {
        out.push_back(std::string(token));
        token = strtok(nullptr, delim);
    }
}

/** Utility messages**/ /// <--- Maybe these can become exceptions (future development)
char* dismissalMessage(){

	return "\nOne is glad to be of service.";

}

char* notAvailableOptionMessage(){

	return "\nThe chosen option is not available in the current system configuration. \n";

}

char* welcomeMessage(){

	/*return "Please select your user role (General User as default)\n" 
		"\t1 General User\n"
		"\t2 Technician \n\n";//*/

	return "";

}

char* wrongInputMessage(){

	return "\nWrong input.Please insert a correct number for the corresponding action to request.\n ";
}

char* wrongOptionMessage(){

	return "\nThe choice does not correspond to an available option. Repeat your choice: \n";

}


