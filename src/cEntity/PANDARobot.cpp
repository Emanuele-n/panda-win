// Standard Header files
//#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>

// Project Header files
#include "PANDARobot.hpp"
#include "utils.hpp"

// FrankaPandaDynModelLib
#include "franka_model.hpp"


int PANDARobot::frictionParamsNum;						//!< Number of parameters considered for the friction model
bool PANDARobot::withDeltaGravity;						//!< State if the computation of the dynamic model has to account also the locally estimated delta-gravity 


PANDARobot::PANDARobot() : RobotInterface() {

	this->jointNum = PANDA_JOINT_NUM;
	this->initData();
	
}

PANDARobot::PANDARobot(const std::string& name_) : RobotInterface(name_) {

	this->jointNum = PANDA_JOINT_NUM;
	this->initData();

}

PANDARobot::PANDARobot(const int& qnum) : RobotInterface(qnum) {this->initData();}

void PANDARobot::initData() {

	RobotInterface::initData();

	this->resFriFbe.setZero();							//!< Cartesian Force vector at the end-effector wrt base frame, reconstructed from FRI residual
	this->detJJT = 0.0;
	this->detJTJ = 0.0;
	this->friResidual_off.setZero();

	//set DH parameters
	this->robDHPars.d(0) = 0.333;
	this->robDHPars.d(1) = 0;
	this->robDHPars.d(2) = 0.316;
	this->robDHPars.d(3) = 0;
	this->robDHPars.d(4) = 0.384;
	this->robDHPars.d(5) = 0;
	this->robDHPars.d(6) = 0;

	this->robDHPars.a(0) = 0;
	this->robDHPars.a(1) = 0;
	this->robDHPars.a(2) = 0;
	this->robDHPars.a(3) = 0.0825;
	this->robDHPars.a(4) = -0.0825;
	this->robDHPars.a(5) = 0;
	this->robDHPars.a(6) = 0.088;

	this->robDHPars.alpha(0) = 0;
	this->robDHPars.alpha(1) = -M_PI / 2;
	this->robDHPars.alpha(2) =  M_PI / 2;
	this->robDHPars.alpha(3) =  M_PI / 2;
	this->robDHPars.alpha(4) = -M_PI / 2;
	this->robDHPars.alpha(5) =	M_PI / 2;
	this->robDHPars.alpha(6) =	M_PI / 2;

	this->eeDHOffset = static_cast<float>(0.107);

	this->TeeOffset.setIdentity();

	double gain = 5.0;
	this->resGain = Eigen::MatrixXd::Identity(PANDA_JOINT_NUM, PANDA_JOINT_NUM) * gain;
	this->invResGainDt = (Eigen::MatrixXd::Identity(PANDA_JOINT_NUM, PANDA_JOINT_NUM) + this->resGain * this->dt__).inverse();

}

void PANDARobot::loadParamsFromConfigFile(const std::string& comment, const std::string& value) {

	/* Please note that this function is called several times*/
	if (comment.find("Links length") != std::string::npos) {
		std::vector < double > vec = parseCSVLine(value);
		for (int j = 0; j < vec.size(); j++) {
			this->linkLengths__(j) = vec[j];
		}
	}
	else if (comment.find("Dynamic model with tool") != std::string::npos) {
		this->includeToolDynParams = (std::stod(value)) ? true : false;
	}
	else if (comment.find("With Friction") != std::string::npos) {
		this->withFriction = (std::stod(value)) ? true : false;
	}
	else if (comment.find("Use model-based residual") != std::string::npos) {
		this->withModelBasedResidual = (std::stod(value)) ? true : false;
	}
	else if (comment.find("Residual gain") != std::string::npos) {
		float gain = static_cast<float>(std::stod(value));
		this->resGain = Eigen::MatrixXd::Identity(PANDA_JOINT_NUM, PANDA_JOINT_NUM) * gain;
		this->invResGainDt = (Eigen::MatrixXd::Identity(PANDA_JOINT_NUM, PANDA_JOINT_NUM) + this->resGain * this->dt__).inverse();
	}
	else if (comment.find("Start on place") != std::string::npos) {
		this->startOnPlace = (std::stod(value)) ? true : false;
	}
	else if (comment.find("Residual offset") != std::string::npos) {
		this->withResOffset = (std::stod(value)) ? true : false;
	}
	else if (comment.find("Initial configuration") != std::string::npos) {
		std::vector < double > vec = parseCSVLine(value);
		for (int j = 0; j < vec.size(); j++) {
			this->jointMsrPosition__(j) = (vec[j]) * M_PI / 180.0;
			this->jointCmdPosition__(j) = (vec[j]) * M_PI / 180.0;
		}
	}


}

void PANDARobot::computeKinematics() {

	// load DH parameter (modified convention from C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca,
	//					'Dynamic Identification of the Franka Emika Panda Robot With Retrieval of Feasible Parameters Using Penalty-Based Opmization'. IEEE RA-L, 2019.)

	// they should be object variable, for now put them here
	Eigen::VectorXd d{this->robDHPars.d};
	Eigen::VectorXd a{this->robDHPars.a};
	Eigen::VectorXd alphas(this->robDHPars.alpha);
	Eigen::VectorXd theta(PANDA_JOINT_NUM);
	Eigen::Matrix4d TeeOffset;

	theta << this->jointMsrPosition__(0), this->jointMsrPosition__(1), this->jointMsrPosition__(2), this->jointMsrPosition__(3),
			 this->jointMsrPosition__(4), this->jointMsrPosition__(5), this->jointMsrPosition__(6); 

	Eigen::Matrix4d Ti[PANDA_JOINT_NUM+1]; //+1 due to E-E frame (modified DH frames)

	// Compute local transformation matrices from link i-1 to link i
	for (int i = LINK1; i <= LINK7; i++) {
		Ti[i] <<		        cos(theta(i)),				-sin(theta(i)),                 0,					  a(i),
		              cos(alphas(i))*sin(theta(i)), cos(alphas(i))*cos(theta(i)),	-sin(alphas(i)),	 -d(i)* sin(alphas(i)),
		              sin(alphas(i))*sin(theta(i)), sin(alphas(i))*cos(theta(i)),    cos(alphas(i)),      d(i)*cos(alphas(i)),
								0,							   0, 						   0,						1;
	}
	// Consider the last offset df from the Franka Panda DH table
	Ti[LINK7 + 1].setIdentity();
	Ti[LINK7 + 1](2, 3) = this->eeDHOffset;


	// Compute the homogeneous transformation matrices of the i-th link pose wrt the base frame
	this->Tbli__[0] = Ti[0];

	for (int i = LINK2; i <= LINK7; i++) {
		this->Tbli__[i] = this->Tbli__[i - 1] * Ti[i];
	}
	this->Tbli__[LINK7] = this->Tbli__[LINK7] * Ti[LINK7 + 1];//*/
	this->Tbee__ = this->Tbli__[LINK7];

	// Check if there are end-effector objects mounted at the robot tip.
	// If so, update robot kinematics and dynamics consistently
	/*Eigen::Matrix4d Tprev = this->Tbee__;
	if (this->tools.size() > 0) {
		for (int i = 0; i < this->tools.size(); i++) {
			Eigen::Matrix4d Tee = this->tools[i].getTee();
			this->Tbee__ = Tprev * Tee;
			Tprev = this->Tbee__;

			//std::cout << "[PandaRobot] Tee = \n " << Tee << std::endl;
		}
	}*/


}

Eigen::MatrixXd PANDARobot::computeAnalyticJacobian_zyx(const Eigen::VectorXd& q) {

	Eigen::MatrixXd Ja;
	Ja.setZero(SPACE_DIM * 2, PANDA_JOINT_NUM);

	// Precompute sin and cos
	double q1 = q(0), q2 = q(1), q3 = q(2), q4 = q(3), q5 = q(4), q6 = q(5), q7 = q(6);
	/*double t2 = cos(q3);
	double t3 = cos(q4);
	double t4 = cos(q5);
	double t5 = cos(q6);
	double t6 = cos(q7);
	double t7 = sin(q3);
	double t8 = sin(q4);
	double t9 = sin(q5);
	double t10 = sin(q6);
	double t11 = sin(q7);
	double t12 = q1 + q2;
	double t13 = pow(t4,2);
	double t14 = pow(t5, 2);
	double t15 = pow(t6, 2);
	double t16 = cos(t12);
	double t17 = sin(t12);
	double t18 = t2*t4;
	double t19 = t2*t9;
	double t20 = t4*t7;
	double t21 = t7*t9;
	double t22 = t7*t8*t10;
	double t23 = t8*t9*t11;
	double t32 = t3*t6*t10;
	double t35 = t2*t8*t10;
	double t36 = t3*t7*t10;
	double t37 = t5*t7*t8;
	double t39 = t3*t7*1.65e+2;
	double t44 = t7*t8*7.68e+2;
	double t51 = t4*t5*t6*t8;
	double t24 = t3*t16;
	double t25 = t3*t18;
	double t26 = t6*t18;
	double t27 = t8*t16;
	double t28 = t3*t17;
	double t29 = t3*t19;
	double t30 = t3*t20;
	double t31 = t5*t19;
	double t33 = t8*t17;
	double t34 = t3*t21;
	double t38 = t10*t19;
	double t40 = t19*1.65e+2;
	double t41 = -t23;
	double t42 = -t39;
	double t48 = t16*t20;
	double t53 = t16*t21;
	double t54 = t17*t20;
	double t58 = t5*t8*t20;
	double t61 = t17*t21;
	double t65 = t11*t22;
	double t66 = -t35;
	double t67 = -t37;
	double t69 = -t44;
	double t71 = t11*t18*1.76e+2;
	double t81 = t17*t23;
	double t88 = t16*(7.9e+1 / 2.5e+2);
	double t89 = t17*(7.9e+1 / 2.5e+2);
	double t92 = t6*t22*1.76e+2;
	double t43 = -t40;
	double t45 = t2*t24;
	double t46 = t2*t27;
	double t47 = t2*t28;
	double t49 = t4*t27;
	double t50 = t5*t30;
	double t52 = t2*t33;
	double t55 = t9*t27;
	double t56 = t4*t33;
	double t57 = t10*t30;
	double t59 = t6*t34;
	double t60 = t11*t31;
	double t62 = t9*t33;
	double t63 = -t29;
	double t64 = -t30;
	double t68 = -t38;
	double t70 = t30*1.65e+2;
	double t72 = t18*t24;
	double t73 = t19*t24;
	double t74 = t17*t25;
	double t75 = t19*t28;
	double t76 = t11*t48;
	double t77 = t6*t10*t28;
	double t83 = -t53;
	double t84 = -t54;
	double t87 = -t65;
	double t90 = t6*t31*1.76e+2;
	double t91 = t11*t34*1.76e+2;
	double t93 = -t88;
	double t94 = -t89;
	double t95 = t18 + t34;
	double t96 = t21 + t25;
	double t99 = t5*t6*t53;
	double t103 = t24*(4.8e+1/1.25e+2);
	double t104 = t28*(4.8e+1/1.25e+2);
	double t105 = t27*(3.3e+1/4.0e+2);
	double t106 = t33*(3.3e+1/4.0e+2);
	double t110 = t5*t24*t26;
	double t121 = t53*(3.3e+1/4.0e+2);
	double t122 = t61*(3.3e+1/4.0e+2);
	double t131 = t36 + t58;
	double t153 = t32 + t41 + t51;
	double t78 = t11*t50;
	double t79 = -t45;
	double t80 = -t50;
	double t82 = -t52;
	double t85 = -t56;
	double t86 = -t60;
	double t97 = t11*t73;
	double t98 = t6*t10*t46;
	double t100 = t5*t6*t56;
	double t101 = -t73;
	double t102 = -t77;
	double t107 = t6*t50*1.76e+2;
	double t108 = -t105;
	double t109 = -t106;
	double t111 = t19 + t64;
	double t112 = t20 + t63;
	double t117 = t45*(3.3e+1/4.0e+2);
	double t118 = t46*(4.8e+1/1.25e+2);
	double t119 = t47*(3.3e+1/4.0e+2);
	double t120 = t52*(4.8e+1/1.25e+2);
	double t123 = t5*t96;
	double t124 = t6*t95;
	double t125 = t11*t95;
	double t128 = -t121;
	double t129 = t27 + t47;
	double t130 = t28 + t46;
	double t154 = t37 + t57 + t68;
	double t164 = t49 + t61 + t74;
	double t169 = t55 + t75 + t84;
	double t113 = -t97;
	double t114 = -t98;
	double t115 = t5*t6*t85;
	double t116 = -t107;
	double t126 = -t119;
	double t127 = -t120;
	double t132 = t5*t111;
	double t133 = -t124;
	double t134 = t10*t111;
	double t135 = t24 + t82;
	double t136 = t33 + t79;
	double t137 = t4*t129;
	double t138 = t5*t130;
	double t139 = t9*t129;
	double t140 = t10*t129;
	double t141 = t10*t130;
	double t152 = t22 + t31 + t80;
	double t156 = t66 + t123;
	double t167 = t53 + t72 + t85;
	double t168 = t48 + t62 + t101;
	double t172 = t5*t164;
	double t179 = t11*t169;
	double t196 = t26 + t59 + t78 + t86 + t87;
	double t142 = t4*t138;
	double t143 = t4*t136;
	double t144 = t5*t135;
	double t145 = t9*t136;
	double t146 = t10*t135;
	double t147 = t10*t136;
	double t150 = t22 + t132;
	double t155 = t137*(3.3e+1/4.0e+2);
	double t157 = t67 + t134;
	double t161 = t11*t152;
	double t162 = pow((t37 - t134),2);
	double t163 = t61 + t137;
	double t166 = t84 + t139;
	double t175 = t5*t167;
	double t177 = -t11*(t54 - t139);
	double t178 = t11*t168;
	double t184 = t11*(t54 - t139);
	double t186 = -t179;
	double t223 = t42 + t43 + t69 + t70 + t71 + t90 + t91 + t92 + t116;
	double t227 = t76 + t81 + t99 + t102 + t110 + t113 + t114 + t115;
	double t148 = t4*t144;
	double t149 = -t142;
	double t158 = t143*(3.3e+1/4.0e+2);
	double t159 = t6*t150;
	double t160 = t11*t150;
	double t165 = t48 + t145;
	double t170 = t5*t163;
	double t171 = t10*t163;
	double t173 = t83 + t143;
	double t180 = -t5*(t53 - t143);
	double t181 = -t10*(t53 - t143);
	double t183 = -t175;
	double t185 = -t178;
	double t187 = t10*(t53 - t143);
	double t189 = t184*(-1.1e+1/1.25e+2);
	double t194 = t133 + t161;
	double t198 = pow((t124 - t161),2);
	double t202 = t146 + t172;
	double t228 = 1.0/pow(t227,2);
	double t151 = -t148;
	double t174 = t11*t165;
	double t176 = -t171;
	double t190 = t125 + t159;
	double t192 = t147 + t149;
	double t193 = t133 + t160;
	double t197 = pow((t124 - t160),2);
	double t199 = -1.0/(t124 - t160);
	double t201 = t146 + t170;
	double t203 = t141 + t180;
	double t205 = t141 + t183;
	double t207 = t6*t202;
	double t208 = t138 + t187;
	double t182 = -t174;
	double t188 = t174*(1.1e+1/1.25e+2);
	double t191 = t140 + t151;
	double t195 = pow(t190,2);
	double t200 = 1.0/t197;
	double t204 = t144 + t176;
	double t206 = t6*t201;
	double t209 = t6*t203;
	double t210 = t6*t205;
	double t215 = t162 + t197;
	double t218 = t186 + t207;
	double t219 = pow((t179 - t207),2);
	double 	t211 = t206*(1.1e+1/1.25e+2);
	double t213 = t209*(1.1e+1/1.25e+2);
	double t216 = 1.0/t215;
	double t217 = t184 + t206;
	double t220 = t182 + t209;
	double t221 = t185 + t210;
	double t224 = pow((t178 - t210), 2);
	double t225 = pow((t174 - t209), 2);
	double t212 = -t211;
	double t214 = -t213;
	double 	t222 = pow(t217, 2);
	double t226 = 1.0/t225;
	double t230 = t219 + t224;
	double t229 = t222 + t225;
	double t231 = t222*t226;
	double t232 = 1.0/t230;
	double t237 = t93 + t103 + t108 + t122 + t126 + t127 + t155 + t189 + t212;
	double t238 = t94 + t104 + t109 + t117 + t118 + t128 + t158 + t188 + t214;
	double t233 = t231 + 1.0;
	double t234 = 1.0/t229;
	double t235 = 1.0/sqrt(t229);
	double t239 = t195 + t229;
	double t236 = pow(t235,3);
	double t240 = 1.0/t239;
	double t241 = t225*t233*t234;
	
	Ja.col(0) << t237, t238, 0.0, t241, 0.0, 0.0; 
	Ja.col(1) << t237, t238, 0.0, t241, 0.0, 0.0;
	Ja.col(2) <<
		(t16 * t223) / 2.0e+3,
		(t17* t223) / 2.0e+3,
		t21* (3.3e+1 / 4.0e+2) + t25 * (3.3e+1 / 4.0e+2) - t2 * t3 * (3.3e+1 / 4.0e+2) - t2 * t8 * (4.8e+1 / 1.25e+2) - t11 * t112 * (1.1e+1 / 1.25e+2) + t6 * (t35 - t123) * (1.1e+1 / 1.25e+2),
		-t153 * t224 * t228 * t232 * (t6 * t22 + t11 * t18 + t6 * t31 + t11 * t34 + t6 * t80),
		t229* t240* (t235* (t11* t112 - t6 * (t35 - t123)) + (t190 * t236 * (t217 * (t6 * (t17 * t22 + t5 * (t17 * t19 - t20 * t28)) + t11 * (t17 * t18 + t21 * t28)) * 2.0 + (t6 * (t16 * t22 + t5 * (t16 * t19 - t20 * t24)) + t11 * (t16 * t18 + t21 * t24)) * (t174 - t209) * 2.0)) / 2.0),
		(t153 * 1.0 / pow(t196, 2) * t198) / (t198 + pow(t154, 2));
	Ja.col(3) <<
		t28 * (-3.3e+1 / 4.0e+2) - t33 * (4.8e+1 / 1.25e+2) + t45 * (4.8e+1 / 1.25e+2) - t46 * (3.3e+1 / 4.0e+2) + t4 * t130 * (3.3e+1 / 4.0e+2) - t6 * (t142 - t147) * (1.1e+1 / 1.25e+2) + t9 * t11 * t130 * (1.1e+1 / 1.25e+2),
		t24* (3.3e+1 / 4.0e+2) + t27 * (4.8e+1 / 1.25e+2) + t47 * (4.8e+1 / 1.25e+2) - t52 * (3.3e+1 / 4.0e+2) - t4 * t135 * (3.3e+1 / 4.0e+2) - t6 * t191 * (1.1e+1 / 1.25e+2) - t9 * t11 * t135 * (1.1e+1 / 1.25e+2),
		t7* (t3 * 7.68e+2 - t8 * 1.65e+2 + t23 * 1.76e+2 - t32 * 1.76e+2 - t51 * 1.76e+2 + t4 * t8 * 1.65e+2)* (-5.0e-4),
		-t224 * t228 * t232 * (-t2 + t2 * t13 + t9 * t30 - t2 * t13 * t15 + t2 * t14 * t15 + t9 * t15 * t64 - t2 * t13 * t14 * t15 + t5 * t9 * t11 * t26 * 2.0 + t9 * t14 * t15 * t64 + t3 * t5 * t6 * t7 * t11 + t6 * t8 * t10 * t11 * t20 + t5 * t8 * t10 * t15 * t21 - t3 * t5 * t6 * t7 * t11 * t13 * 2.0),
		-t229 * t240 * (t235 * (t6 * t131 - t8 * t11 * t21) + (t190 * t236 * (t217 * (t6 * t191 + t9 * t11 * t135) * 2.0 + (t6 * (t142 - t147) - t9 * t11 * t130) * (t174 - t209) * 2.0)) / 2.0),
		t197* t216* ((t3* t5* t7 - t8 * t10 * t20) / (t124 - t160) + t200 * (t11 * t131 + t6 * t8 * t21) * (t37 - t134));

	Ja.col(4) <<
		t48 * (-3.3e+1 / 4.0e+2) - t145 * (3.3e+1 / 4.0e+2) - t11 * (t53 - t143) * (1.1e+1 / 1.25e+2) + t5 * t6 * t165 * (1.1e+1 / 1.25e+2),
		t54* (-3.3e+1 / 4.0e+2) + t139 * (3.3e+1 / 4.0e+2) - t11 * t163 * (1.1e+1 / 1.25e+2) + t5 * t6 * (t54 - t139) * (1.1e+1 / 1.25e+2),
		t18* (-3.3e+1 / 4.0e+2) - t34 * (3.3e+1 / 4.0e+2) - t11 * t111 * (1.1e+1 / 1.25e+2) + t5 * t124 * (1.1e+1 / 1.25e+2),
		t224* t228* t232* (-t7 * t8 + t7 * t8 * t15 + t10 * t11 * t26 + t10 * t15 * t31 + t11 * t21 * t32 + t10 * t15 * t80 - t7 * t8 * t14 * t15),
		t229* t240* (t235* (t11* t111 + t5 * t133) - (t190 * t236 * (t217 * (t11 * t163 - t5 * t6 * (t54 - t139)) * 2.0 + (t11 * (t53 - t143) - t5 * t6 * t165) * (t174 - t209) * 2.0)) / 2.0),
		-t197 * t216 * ((t10 * t95) / (t124 - t160) - t200 * (t37 - t134) * (t6 * t111 + t5 * t125));
	Ja.col(5) <<
		t6 * t208 * (-1.1e+1 / 1.25e+2),
		t6* t204* (1.1e+1 / 1.25e+2),
		t6* (t37 - t134)* (1.1e+1 / 1.25e+2),
		t6* t196* t224* t228* t232,
		t229* t240* ((t190* t236* (t6* t204* t217 * 2.0 - t6 * t208 * (t174 - t209) * 2.0)) / 2.0 - t6 * t235 * (t37 - t134)),
		-t197 * t216 * (t150 / (t124 - t160) - t11 * t162 * t200);

	Ja.col(6) << 
		t6* t165* (1.1e+1 / 1.25e+2) + t11 * t203 * (1.1e+1 / 1.25e+2),
		t11* t201* (-1.1e+1 / 1.25e+2) + t6 * (t54 - t139) * (1.1e+1 / 1.25e+2),
		t124* (1.1e+1 / 1.25e+2) - t160 * (1.1e+1 / 1.25e+2),
		-t154 * t224 * t228 * t232,
		-t229 * t240 * (t235 * (t124 - t160) + (t190 * t236 * (t217 * (t11 * t201 - t6 * (t54 - t139)) * 2.0 - (t174 - t209) * (t6 * t165 + t11 * t203) * 2.0)) / 2.0),
		t190* t216* (t37 - t134);*/

	double t2 = cos(q1);
	double t3 = cos(q2);
	double t4 = cos(q3);
	double t5 = cos(q4);
	double t6 = cos(q5);
	double t7 = cos(q6);
	double t8 = cos(q7);
	double t9 = sin(q1);
	double t10 = sin(q2);
	double t11 = sin(q3);
	double t12 = sin(q4);
	double t13 = sin(q5);
	double t14 = sin(q6);
	double t15 = sin(q7);
	double t16 = t2*t4;
	double t17 = t3*t5;
	double t18 = t2*t11;
	double t19 = t4*t9;
	double t20 = t3*t12;
	double t21 = t5*t10;
	double t22 = t9*t11;
	double t23 = t10*t12;
	double t26 = t10*t11*t13;
	double t27 = t3*6.32e+2;
	double t38 = t4*t6*t10;
	double t40 = t4*t6*t15;
	double t48 = t3*t11*t13;
	double t49 = t4*t10*t13;
	double t50 = t6*t10*t11;
	double t51 = t4*t10*1.65e+2;
	double t61 = t4*t7*t8*t13;
	double t70 = t5*t11*t13*t15;
	double t71 = t8*t11*t12*t14;
	double t85 = t5*t6*t7*t8*t11;
	double t24 = t9*t23;
	double t25 = t12*t22;
	double t28 = t3*t16;
	double t29 = t4*t17;
	double t30 = t7*t17;
	double t31 = t3*t18;
	double t32 = t3*t19;
	double t33 = t2*t20;
	double t34 = t2*t21;
	double t35 = t5*t18;
	double t36 = t4*t20;
	double t37 = t4*t21;
	double t39 = t6*t20;
	double t41 = t3*t22;
	double t42 = t2*t23;
	double t43 = t9*t20;
	double t44 = t9*t21;
	double t45 = t12*t18;
	double t46 = t5*t22;
	double t47 = t4*t23;
	double t52 = t20*1.65e+2;
	double t53 = t17*7.68e+2;
	double t54 = -t51;
	double t56 = t16*t17;
	double t57 = t16*t20;
	double t58 = t16*t21;
	double t59 = t17*t19;
	double t62 = t19*t20;
	double t63 = t19*t21;
	double t64 = t6*t11*t21;
	double t66 = t6*t15*t19;
	double t69 = t11*t13*t21;
	double t74 = -t40;
	double t75 = t11*t14*t23;
	double t76 = t14*t26;
	double t81 = -t48;
	double t82 = -t50;
	double t84 = t14*t17*1.76e+2;
	double t89 = t7*t8*t13*t19;
	double t99 = -t71;
	double t105 = t7*t26*1.76e+2;
	double t55 = -t52;
	double t60 = t6*t37;
	double t65 = t7*t47;
	double t67 = t14*t39;
	double t68 = -t28;
	double t72 = -t36;
	double t73 = -t37;
	double t77 = -t41;
	double t78 = -t42;
	double t79 = -t44;
	double t80 = -t46;
	double t83 = t37*1.65e+2;
	double t86 = t6*t15*t31;
	double t87 = t8*t14*t34;
	double t90 = -t57;
	double t91 = -t58;
	double t93 = t13*t15*t42;
	double t94 = t13*t15*t46;
	double t95 = t8*t14*t25;
	double t96 = -t63;
	double t98 = -t69;
	double t100 = -t75;
	double t101 = -t76;
	double t102 = t47*7.68e+2;
	double t103 = t7*t39*1.76e+2;
	double t104 = t14*t47*1.76e+2;
	double t106 = t18 + t32;
	double t107 = t19 + t31;
	double t108 = t17 + t47;
	double t109 = t23 + t29;
	double t110 = t13*t15*t56;
	double t111 = t8*t14*t57;
	double t112 = t7*t8*t13*t31;
	double t113 = t6*t7*t8*t42;
	double t114 = t6*t7*t8*t46;
	double t115 = -t89;
	double t118 = t6*t8*t16*t30;
	double t136 = t49 + t64;
	double t165 = t24 + t35 + t59;
	double t229 = t61 + t70 + t74 + t85 + t99;
	double t88 = t14*t60;
	double t92 = -t60;
	double t97 = -t67;
	double t116 = t13*t15*t80;
	double t117 = t7*t60*1.76e+2;
	double t119 = t16 + t77;
	double t120 = t22 + t68;
	double t121 = t20 + t73;
	double t122 = t21 + t72;
	double t123 = t8*t14*t90;
	double t124 = -t112;
	double t125 = t6*t7*t8*t80;
	double t127 = t5*t106;
	double t128 = t6*t106;
	double t129 = t6*t107;
	double t130 = t6*t109;
	double t131 = t7*t108;
	double t132 = t12*t106;
	double t133 = t13*t106;
	double t134 = t13*t107;
	double t135 = t14*t108;
	double t140 = t12*t14*t107;
	double t153 = t33 + t91;
	double t154 = t43 + t96;
	double t155 = t38 + t98;
	double t156 = t7*t136;
	double t168 = t42 + t56 + t80;
	double t169 = t25 + t34 + t90;
	double t171 = t45 + t62 + t79;
	double t177 = t6*t165;
	double t178 = t13*t165;
	double t126 = -t117;
	double t137 = t5*t129;
	double t138 = t6*t131;
	double t139 = t5*t134;
	double t141 = t5*t120;
	double t142 = t6*t119;
	double t143 = t6*t120;
	double t144 = -t129;
	double t145 = t6*t121;
	double t146 = -t131;
	double t147 = t12*t120;
	double t148 = t13*t119;
	double t149 = t13*t120;
	double t150 = -t134;
	double t151 = t13*t121;
	double t152 = t14*t121;
	double t161 = t12*t14*t119;
	double t164 = t24 + t127;
	double t166 = t79 + t132;
	double t170 = t26 + t39 + t92;
	double t173 = t81 + t130;
	double t182 = -t7*(t44 - t132);
	double t185 = t6*t168;
	double t186 = -t14*(t44 - t132);
	double t188 = t13*t168;
	double t189 = t14*t169;
	double t191 = t14*t171;
	double t195 = -t13*t15*(t44 - t132);
	double t201 = t100 + t156;
	double t230 = t30 + t65 + t88 + t97 + t101;
	double t296 = t66 + t86 + t87 + t93 + t95 + t110 + t113 + t115 + t116 + t118 + t123 + t124 + t125;
	double t157 = t5*t142;
	double t158 = -t137;
	double t159 = -t138;
	double t160 = t5*t148;
	double t162 = -t142;
	double t167 = t26 + t145;
	double t172 = t34 + t147;
	double t174 = t6*t164;
	double t175 = t13*t164;
	double t176 = t14*t164;
	double t179 = t82 + t151;
	double t180 = t78 + t141;
	double t190 = t14*t170;
	double t192 = t6*t182;
	double t194 = -t6*(t42 - t141);
	double t196 = -t13*(t42 - t141);
	double t197 = -t14*(t42 - t141);
	double t198 = -t8*(t50 - t151);
	double t199 = -t15*(t50 - t151);
	double t203 = t139 + t143;
	double t210 = -t7*(t137 - t149);
	double t213 = t129 + t188;
	double t214 = t148 + t177;
	double t218 = t150 + t185;
	double t237 = -t7*(t134 - t185);
	double t241 = -t15*(t142 - t178);
	double t246 = -t15*(t129 + t13*(t42 - t141));
	double t248 = t15*(t142 - t178);
	double t249 = -t7*t8*(t129 + t13*(t42 - t141));
	double t251 = t15*(t129 + t13*(t42 - t141));
	double t252 = t7*t8*(t129 + t13*(t42 - t141));
	double t283 = t27 + t53 + t54 + t55 + t83 + t84 + t102 + t103 + t104 + t105 + t126;
	double t297 = pow(1.0/t296,2);
	double t163 = -t157;
	double t181 = t7*t172;
	double t183 = t14*t172;
	double t184 = t7*t167;
	double t187 = t14*t167;
	double t193 = t13*t15*t172;
	double t202 = t128 + t160;
	double t205 = t149 + t158;
	double t206 = t152 + t159;
	double t208 = t15*t203;
	double t211 = t148 + t174;
	double t215 = t134 + t194;
	double t216 = t162 + t175;
	double t219 = t162 + t178;
	double t220 = t146 + t190;
	double t226 = t7*t214;
	double t227 = t15*t213;
	double t228 = t144 + t196;
	double t232 = pow((t131 - t190),2);
	double t238 = -t15*(t142 - t175);
	double t244 = -t7*t8*(t142 - t175);
	double t247 = t15*(t142 - t175);
	double t250 = t7*t8*(t142 - t175);
	double t253 = t176 + t192;
	double t254 = t140 + t210;
	double t266 = t189 + t237;
	double t200 = t6*t181;
	double t204 = t133 + t163;
	double t207 = t15*t202;
	double t212 = t135 + t184;
	double t217 = t146 + t187;
	double t221 = t7*t211;
	double t223 = t14*t211;
	double t224 = t15*t211;
	double t231 = pow((t131 - t187),2);
	double t233 = t7*t215;
	double t234 = t14*t215;
	double t236 = t15*t215;
	double t240 = -t226;
	double t242 = -1.0/(t131 - t187);
	double t255 = t8*t253;
	double t258 = t8*t254;
	double t273 = t8*t266;
	double t209 = t7*t204;
	double t222 = t8*t212;
	double t225 = t15*t212;
	double t235 = -t221;
	double t243 = 1.0/t231;
	double t245 = -t233;
	double t256 = t197 + t200;
	double t261 = -t258;
	double t263 = t182 + t223;
	double t267 = t191 + t240;
	double t268 = t181 + t234;
	double t272 = -t8*(t221 + t14*(t44 - t132));
	double t277 = t195 + t255;
	double t279 = t224 + t250;
	double t281 = t236 + t252;
	double t284 = t227 + t273;
	double t290 = pow((t238 + t8*(t221 + t14*(t44 - t132))),2);
	double t239 = -t225;
	double t257 = t161 + t209;
	double t259 = t8*t256;
	double t264 = t186 + t235;
	double t265 = t199 + t222;
	double t271 = t183 + t245;
	double t274 = t8*t267;
	double t275 = pow((t225 + t8*(t50 - t151)),2);
	double t280 = t208 + t261;
	double t286 = pow(t284,2);
	double t287 = t247 + t272;
	double t260 = t8*t257;
	double t269 = t198 + t239;
	double t270 = pow(t265,2);
	double t276 = t8*t271;
	double t278 = t193 + t259;
	double t285 = t231 + t275;
	double t288 = t248 + t274;
	double 	t262 = -t260;
	double t289 = 1.0/t285;
	double t291 = pow(t288,2);
	double t292 = t251 + t276;
	double t282 = t207 + t262;
	double t293 = pow(t292,2);
	double t294 = 1.0/t292;
	double t298 = t286 + t291;
	double t295 = 1.0/t293;
	double t299 = 1.0/t298;
	double t300 = t290 + t293;
	double t301 = 1.0/t300;
	double t302 = 1.0/sqrt(t300);
	double t304 = t270 + t300;
	double t303 = pow(t302,3);
	double t305 = 1.0/t304;

	Ja.col(0) << t18 * (-3.3e+1 / 4.0e+2) + t24 * (3.3e+1 / 4.0e+2) - t32 * (3.3e+1 / 4.0e+2) - t44 * (4.8e+1 / 1.25e+2) + t127 * (3.3e+1 / 4.0e+2) + t132 * (4.8e+1 / 1.25e+2) - t221 * (1.1e+1 / 1.25e+2) - t9 * t10 * (7.9e+1 / 2.5e+2) - t14 * (t44 - t132) * (1.1e+1 / 1.25e+2), t22 * (-3.3e+1 / 4.0e+2) + t28 * (3.3e+1 / 4.0e+2) + t34 * (4.8e+1 / 1.25e+2) - t42 * (3.3e+1 / 4.0e+2) + t141 * (3.3e+1 / 4.0e+2) + t147 * (4.8e+1 / 1.25e+2) + t183 * (1.1e+1 / 1.25e+2) - t233 * (1.1e+1 / 1.25e+2) + t2 * t10 * (7.9e+1 / 2.5e+2), 0.0, t293 * t301 * (t290 * t295 + 1.0), 0.0, 0.0;
	Ja.col(1) << (t2* t283) / 2.0e+3, (t9* t283) / 2.0e+3, t10* (-7.9e+1 / 2.5e+2) - t21 * (4.8e+1 / 1.25e+2) + t23 * (3.3e+1 / 4.0e+2) + t29 * (3.3e+1 / 4.0e+2) + t36 * (4.8e+1 / 1.25e+2) - t3 * t4 * (3.3e+1 / 4.0e+2) - t14 * t122 * (1.1e+1 / 1.25e+2) + t7 * (t48 - t130) * (1.1e+1 / 1.25e+2), -t229 * t286 * t297 * t299 * (t15 * t82 + t8 * t14 * t17 + t7 * t8 * t26 + t13 * t15 * t20 + t7 * t8 * t39 + t8 * t14 * t47 + t13 * t15 * t73 + t7 * t8 * t92), t300* t305* (t302* (t15* (t13* t109 + t3 * t6 * t11) + t8 * (t14 * t122 - t7 * (t48 - t130))) + (t265 * t303 * ((t8 * (t7 * (t6 * t154 + t10 * t13 * t22) + t14 * (t9 * t17 + t19 * t23)) + t15 * (t13 * t154 - t6 * t10 * t22)) * (t238 + t8 * (t221 + t14 * (t44 - t132))) * 2.0 + t292 * (t8 * (t7 * (t6 * t153 + t10 * t13 * t18) + t14 * (t2 * t17 + t16 * t23)) + t15 * (t13 * t153 - t6 * t10 * t18)) * 2.0)) / 2.0), (t229 * 1.0 / pow(t230,2) * t232) / (t232 + pow((t8 * (t50 - t13 * t20 + t13 * t37) + t15 * (t135 + t7 * t170)),2));
	Ja.col(2) << t19*(-3.3e+1/4.0e+2) - t31*(3.3e+1/4.0e+2) + t140*(1.1e+1/1.25e+2) + t5*t107*(3.3e+1/4.0e+2) + t12*t107*(4.8e+1/1.25e+2) - t7*(t137 - t149)*(1.1e+1/1.25e+2), t16*(3.3e+1/4.0e+2) - t41*(3.3e+1/4.0e+2) - t161*(1.1e+1/1.25e+2) - t209*(1.1e+1/1.25e+2) - t5*t119*(3.3e+1/4.0e+2) - t12*t119*(4.8e+1/1.25e+2), (t10*(t11*1.65e+2 - t5*t11*1.65e+2 - t11*t12*7.68e+2 + t4*t7*t13*1.76e+2 - t11*t12*t14*1.76e+2 + t5*t6*t7*t11*1.76e+2))/2.0e+3, t293*t301*(t282*t294 + t280*t295*(t238 + t8*(t221 + t14*(t44 - t132)))), t300* t305* (t302* (t15* t155 + t8 * (t75 - t156)) - (t265 * t303 * (t280 * t292 * 2.0 - t282 * (t238 + t8 * (t221 + t14 * (t44 - t132))) * 2.0)) / 2.0), t231* t289* ((t8* t155 - t15 * (t75 - t156)) / (t131 - t187) + t243 * (t225 + t8 * (t50 - t151)) * (t14 * t136 + t7 * t11 * t23));
	Ja.col(3) << t34 * (-3.3e+1 / 4.0e+2) - t42 * (4.8e+1 / 1.25e+2) + t141 * (4.8e+1 / 1.25e+2) - t147 * (3.3e+1 / 4.0e+2) + t200 * (1.1e+1 / 1.25e+2) - t14 * (t42 - t141) * (1.1e+1 / 1.25e+2), t24* (-4.8e+1 / 1.25e+2) - t44 * (3.3e+1 / 4.0e+2) - t127 * (4.8e+1 / 1.25e+2) + t132 * (3.3e+1 / 4.0e+2) - t176 * (1.1e+1 / 1.25e+2) + t6 * t7 * (t44 - t132) * (1.1e+1 / 1.25e+2), t17* (-3.3e+1 / 4.0e+2) - t20 * (4.8e+1 / 1.25e+2) + t37 * (4.8e+1 / 1.25e+2) - t47 * (3.3e+1 / 4.0e+2) + t138 * (1.1e+1 / 1.25e+2) - t152 * (1.1e+1 / 1.25e+2), -t293 * t301 * (t277 * t294 + t278 * t295 * (t238 + t8 * (t221 + t14 * (t44 - t132)))), -t300 * t305 * (t302 * (t8 * (t138 - t152) + t13 * t15 * t108) - (t265 * t303 * (t278 * t292 * 2.0 - t277 * (t238 + t8 * (t221 + t14 * (t44 - t132))) * 2.0)) / 2.0), t231* t289* ((t15* (t138 - t152) - t8 * t13 * t108) / (t131 - t187) + t243 * (t225 + t8 * (t50 - t151)) * (t7 * t121 + t6 * t135));
	Ja.col(4) << t7*(t129 + t13*(t42 - t141))*(-1.1e+1/1.25e+2), t7*(t142 - t175)*(1.1e+1/1.25e+2), t7*(t50 - t151)*(1.1e+1/1.25e+2), t293*t301*(t279*t294 + t281*t295*(t238 + t8*(t221 + t14*(t44 - t132)))), -t300*t305*(t302*(t15*t167 + t7*t8*(t50 - t151)) + (t265*t303*(t281*t292*2.0 - t279*(t238 + t8*(t221 + t14*(t44 - t132)))*2.0))/2.0), -t231 * t289 * ((t8 * t167 + t7 * t199) / (t131 - t187) - t14 * t243 * (t225 + t8 * (t50 - t151)) * (t50 - t151));
	Ja.col(5) << t181*(1.1e+1/1.25e+2) + t234*(1.1e+1/1.25e+2), t223*(-1.1e+1/1.25e+2) + t7*(t44 - t132)*(1.1e+1/1.25e+2), t131*(1.1e+1/1.25e+2) - t187*(1.1e+1/1.25e+2), -t293*t301*(t8*t263*t294 + t8*t268*t295*(t238 + t8*(t221 + t14*(t44 - t132)))), -t300*t305*((t265*t303*(t8*t263*(t238 + t8*(t221 + t14*(t44 - t132)))*2.0 - t8*t268*t292*2.0))/2.0 + t8*t302*(t131 - t187)), t231*t289*(t15 + t212*t243*(t225 + t8*(t50 - t151)));
	Ja.col(6) << 0.0, 0.0, 0.0, -t230*t286*t297*t299, t300*t305*(t302*(t225 + t8*(t50 - t151)) - (t265*t303*((t238 + t8*(t221 + t14*(t44 - t132)))*(t15*(t221 + t14*(t44 - t132)) + t8*(t142 - t175))*2.0 - t292*(t8*(t129 + t13*(t42 - t141)) - t15*t271)*2.0))/2.0), t265*t289*(t131 - t187);

	return Ja;
	
}

#ifdef BUILD_PANDA_DYN_MODEL
void PANDARobot::computeDynamics() {

	Eigen::Vector7d  q, dq, tau, gravity, friction;
	Eigen::Matrix7d mass, coriolis;

	//get joint position and velocity
	q = this->getMsrJointPosition();
	dq = this->getMsrJointVelocity();
	tau = this->getMsrJointTorques().cast<double>();

	//get dynamics parameters (from  C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca,
	//					      'Dynamic Identification of the Franka Emika Panda Robot With Retrieval of Feasible Parameters Using Penalty-Based Opmization'. IEEE RA-L, 2019.)
	//mass = MassMatrix(q);
	//coriolis = CoriolisMatrix(q, dq);
	//gravity = GravityVector(q);
	//friction = Friction(dq);


	/// FIRST VERSION OF THE FRANKA PANDA DYNAMIC MODEL LIBRARY
	//get_MassMatrix(q.data(), mass.data());
	//get_CoriolisMatrix(q.data(), dq.data(), coriolis.data());
	//get_GravityVector(q.data(), gravity.data());
	//get_FrictionTorque(dq.data(), friction.data());

	/// SECOND VERSION OF THE FRANKA PANDA DYNAMIC MODEL LIBRARY (+ ACCURACY & PAYLOAD ACCOUNT)
	/*mass = franka_model::computeFrankaMassMatrix(q);
	coriolis = franka_model::computeFrankaCoriolisMatrix(q, dq);
	gravity = franka_model::computeFrankaGravityVector(q);
	friction = franka_model::computeFrankaFrictionVector(dq);*/

	/// SECOND DLL-BASED VERSION OF THE FRANKA PANDA DYNAMIC MODEL LYBRARY
	double** B = new double* [PANDA_JOINT_NUM];
	double** C = new double* [PANDA_JOINT_NUM];
	double* g = new double[PANDA_JOINT_NUM];
	double* tauf = new double[PANDA_JOINT_NUM];
	double* dyn_pars_tip = new double[DYN_PARAMS_NUM];

	for (int i = 0; i < PANDA_JOINT_NUM; i++) {
		B[i] = new double[PANDA_JOINT_NUM];
		C[i] = new double[PANDA_JOINT_NUM];
	}

	// Test
	for (int i = 0; i < DYN_PARAMS_NUM; i++) {
		dyn_pars_tip[i] = 0.0;
	}

	/*// Antenna case
	if (this->tools.size() == 1) { // antenna case
		dyn_pars_tip[0] = 1.33;
		dyn_pars_tip[1] = +1.4096e-02;
		dyn_pars_tip[2] = -1.0729e-05;
		dyn_pars_tip[3] = +9.0000e-02;

		dyn_pars_tip[0] = 0.0; 1.33;
		dyn_pars_tip[1] = 0.0; 0.0144;
		dyn_pars_tip[2] = 0.0; 0.0;
		dyn_pars_tip[3] = 0.0; 0.1203;
		dyn_pars_tip[4] = 0.0; -1.8790;
		dyn_pars_tip[5] = 0.0; 0.2073;
		dyn_pars_tip[6] = 0.0; 0.3402;
		dyn_pars_tip[7] = 0.0; -1.4419;
		dyn_pars_tip[8] = 0.0; 0.8634;
		dyn_pars_tip[9] = 0.0; -0.5887;

		dyn_pars_tip[0] = 1.2; 1.33;
		dyn_pars_tip[1] = 0.0140; 0.0144;
		dyn_pars_tip[2] = 0.0; 0.0;
		dyn_pars_tip[3] = 0.09; 0.1200 - 0.107; 0.1203; // this should be without the -0.107, but if I add it it doesn't work
		dyn_pars_tip[4] = 0.0; -1.8790;
		dyn_pars_tip[5] = 0.0; 0.2073;
		dyn_pars_tip[6] = 0.0; 0.3402;
		dyn_pars_tip[7] = 0.0; -1.4419;
		dyn_pars_tip[8] = 0.0; 0.8634;
		dyn_pars_tip[9] = 0.0; -0.5887;

	}*/


	computeFrankaMassMatrix(B, q.data(),dyn_pars_tip);
	computeFrankaCoriolisMatrix(C, q.data(), dq.data(), dyn_pars_tip);
	computeFrankaGravityVector(g, q.data(), dyn_pars_tip);
	computeFrankaFrictionVector(tauf, dq.data());

	//save dynamic parameters
	for (int i = 0; i < PANDA_JOINT_NUM; i++) {
		for (int j = 0; j < PANDA_JOINT_NUM; j++) {
			mass(i, j) = B[i][j];
			coriolis(i, j) = C[i][j];
			this->robDynPars.B(i, j) = mass(i,j);
			this->robDynPars.C(i, j) = coriolis(i,j);
		}
	}
	std::memcpy(gravity.data(), g, PANDA_JOINT_NUM * sizeof(double));
	std::memcpy(friction.data(), tauf, PANDA_JOINT_NUM * sizeof(double));
	
	// Delete dynamic matrices
	for (int i = 0; i < PANDA_JOINT_NUM; i++) delete[] B[i];
	for (int i = 0; i < PANDA_JOINT_NUM; i++) delete[] C[i];
	delete[] B;
	delete[] C;
	delete[] g;
	delete[] tauf;

	//*/


	this->robDynPars.g = gravity;
	this->robDynPars.f = friction;
	this->robDynPars.tau = tau;
	this->robDynPars.cvec = coriolis * dq;

}
#endif // BUILD_PANDA_DYN_MODEL

Eigen::Vector7d PANDARobot::computeDeltaFriction(const Eigen::Vector7d& q_k, const Eigen::Vector7d& qdot_k, const Eigen::VectorXd& fg) {

	Eigen::Vector7d dtau_f;
	dtau_f.setZero();

	// The vector of the estimating parameters of the delta-dynamics should be arranged as follows
	// fg = [Fs1, Fv1, sigK1, ..., Fs7, Fv7, sigK7, mx1, my1, mz1, ... , mx7, my7, mz7, m1, ..., m7]^T

	for (int i = 0; i < PANDA_JOINT_NUM; i++) {

		double Fs = fg(i*frictionParamsNum + 0);
		double Fv = fg(i*frictionParamsNum + 1);
		double ki = fg(i*frictionParamsNum + 2);
		double expDen_k = (1.0 + exp(-ki * qdot_k(i)));
		dtau_f(i) = (Fs + Fv / expDen_k);

		if (frictionParamsNum == 4) {
			double Fq = fg(i*frictionParamsNum + 3);
			dtau_f(i) += Fq * q_k(i);
		}


	}


	return dtau_f;
}

Eigen::VectorXd PANDARobot::computeDeltaDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, const Eigen::VectorXd& params){

	Eigen::VectorXd deltaDyn;
	Eigen::VectorXd delta_friction;
	Eigen::VectorXd delta_gravity;
	int N, S, M;

	// Get the data sizes
	N = frictionParamsNum;
	S = (N * PANDA_JOINT_NUM);
	M = (PANDA_JOINT_NUM);
	//S = (withDeltaGravity) ? (N * JOINT_NUM + GRAVITY_DYN_PARAMS_NUM) : (N * JOINT_NUM);
	//M = (withDeltaGravity) ? (2 * JOINT_NUM) : (JOINT_NUM);
	deltaDyn.setZero(M);

	// Compute the delta-friction
	delta_friction = PANDARobot::computeDeltaFriction(q, qdot, params);

	// Compute the delta-gravity
	//delta_gravity = PANDARobot::computeDeltaGravity(q, params);

	//Fill the delta-dynamics vector
	//deltaDyn.topRows(JOINT_NUM) = delta_friction;
	deltaDyn = delta_friction;
	
	/*if (withDeltaGravity) {
		deltaDyn.bottomRows(JOINT_NUM) = delta_gravity;
	}//*/

	// Return the vector
	return deltaDyn;

}

Eigen::MatrixXd PANDARobot::computeDeltaDynJacobian(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, const Eigen::VectorXd& params) {

	Eigen::MatrixXd Jacobian, Jf, Jg;
	int N, S, M;

	// The vector of the estimating parameters of the delta-dynamics should be arranged as follows
	// fg = [Fs1, Fv1, sigK1, ..., Fs7, Fv7, sigK7, mx1, my1, mz1, ... , mx7, my7, mz7, m1, ..., m7]^T (size = 49)
	int fricOffset = frictionParamsNum * PANDA_JOINT_NUM; //<-- TODO: from file
	//int mxyzNum = SPACE_DIM * JOINT_NUM; //<-- TODO: from file
	//float g0 = -9.81; // [m/s^2]

	N = frictionParamsNum;
	S = (N * PANDA_JOINT_NUM);
	M = (PANDA_JOINT_NUM);

	//S = (withDeltaGravity) ? (N * JOINT_NUM + GRAVITY_DYN_PARAMS_NUM) : (N * JOINT_NUM);
	//M = (withDeltaGravity) ? (2 * JOINT_NUM) : (JOINT_NUM);

	Jacobian.setZero(M, S);
	Jf.setZero(PANDA_JOINT_NUM, N * PANDA_JOINT_NUM);
	//Jg.setZero(JOINT_NUM, GRAVITY_DYN_PARAMS_NUM);

	// Fill Jacobian for friction part	
	for (int i = 0; i < PANDA_JOINT_NUM; i++) {
		double Fv_i = params(N*i + 1);
		double Ki_i = params(N*i + 2);
		double expDen_i = (1.0 + exp(-Ki_i * qdot(i)));

		Jf(i, N*i + 0) = 1.0;
		Jf(i, N*i + 1) = 1.0 / expDen_i;
		Jf(i, N*i + 2) = Fv_i * qdot(i) * exp(-Ki_i*qdot(i)) / pow(expDen_i, 2);

		if (N == 4) {
			Jf(i, N*i + 3) = q(i);
		}
	}

	// Fill Jacobian for gravity part
	//Jg = PANDARobot::computeDeltaGravityJacobian(q, params);

	// Fill the Jacobian matrix
	//Jacobian.block(0,0,JOINT_NUM, N * JOINT_NUM) = Jf;
	//Jacobian.block(JOINT_NUM, N * JOINT_NUM, JOINT_NUM, GRAVITY_DYN_PARAMS_NUM) = Jg;
	Jacobian = Jf;

	// Return the jacobian matrix
	return Jacobian;

}

Eigen::MatrixXd PANDARobot::computeLinearJacobian(const int& link) {
	//Eigen::VectorXd d{ this->robDHPars.d };
	//Eigen::VectorXd a{ this->robDHPars.a };
	//Eigen::VectorXd alphas(this->robDHPars.alpha);
	//Eigen::VectorXd theta(PANDA_JOINT_NUM);

	//Eigen::matrix4d dTi[PANDA_JOINT_NUM + 1]; //+1 due to E-E frame (modified DH frames)
	//Eigen::matrix4d Ti[PANDA_JOINT_NUM + 1]; //+1 due to E-E frame (modified DH frames)

	//Eigen::matrixXd Jl(SPACE_DIM, PANDA_JOINT_NUM);

	//theta << this->jointMsrPosition__(0), this->jointMsrPosition__(1), this->jointMsrPosition__(2), this->jointMsrPosition__(3),
	//	this->jointMsrPosition__(4), this->jointMsrPosition__(5), this->jointMsrPosition__(6);


	//// Compute local transformation matrices from link i-1 to link i
	//for (int i = LINK1; i <= END_EFFECTOR; i++) {
	//	Ti[i] <<		        cos(theta(i)),				-sin(theta(i)),                 0,					  a(i),
	//	              cos(alphas(i))*sin(theta(i)), cos(alphas(i))*cos(theta(i)),	-sin(alphas(i)),	 -d(i)* sin(alphas(i)),
	//	              sin(alphas(i))*sin(theta(i)), sin(alphas(i))*cos(theta(i)),    cos(alphas(i)),      d(i)*cos(alphas(i)),
	//							0,							   0, 						   0,						1;
	//}
	////Compute local transformation matrices from link i-1 to link i
	//for (int i = LINK1; i <= END_EFFECTOR; i++) {
	//	dTi[i] <<				-sin(theta(i)),					 -cos(theta(i)),			0,		 0,
	//					cos(alphas(i))*cos(theta(i)),	 -cos(alphas(i))*sin(theta(i)),		0,		 0,
	//					sin(alphas(i))*cos(theta(i)),	 -sin(alphas(i))*sin(theta(i)),		0,		 0,
	//								0,									0,					0,		 0;
	//}

	////add the offset due to modified DH convention and robot tool (if any)
	//Ti[END_EFFECTOR + 1].setIdentity();
	//Ti[END_EFFECTOR + 1](2, 3) = this->eeDHOffset + this->computeEEOffset();

	//Eigen::matrix4d dTbee; //derivative of the homogeneuos tranformation matrix of EE w.r.t. robot base
	//for (int l = LINK1; l <= link; ++l) {
	//	dTbee.setIdentity();
	//	for (int i = LINK1; i <= END_EFFECTOR; i++) {
	//		if (i == l)
	//			dTbee *= dTi[l];
	//		else
	//			dTbee *= Ti[i];
	//	}

	//	dTbee = dTbee * Ti[END_EFFECTOR + 1];
	//	//extract derivative of EE position w.r.t. robot base
	//	Jl(0, l) = dTbee(0, 3);
	//	Jl(1, l) = dTbee(1, 3);
	//	Jl(2, l) = dTbee(2, 3);
	//}


	Eigen::MatrixXd Jl(SPACE_DIM, PANDA_JOINT_NUM);
	Eigen::Vector3d zbl;
	Eigen::Vector3d pbl;
	Eigen::Vector3d pbee;
	Eigen::Vector3d pblee;
	Eigen::Vector3d j;
	
	Jl.setZero();
	zbl.setZero();
	pbl.setZero();
	pbee.setZero();
	pblee.setZero();
	j.setZero();
	
	pbee = this->getEEPosition().cast<double>();

	for (int l = LINK1; l <= link; ++l) {
		//extract derivative of EE position w.r.t. robot base
		zbl = this->getLinkRotMat(l).cast<double>().col(2).head(3);
		pbl = this->getLinkPosition(l).cast<double>();
		pblee = pbee - pbl;

		j = zbl.cross(pblee);
			
		Jl(0, l) = j(0);
		Jl(1, l) = j(1);
		Jl(2, l) = j(2);
	}

	return Jl;

}

Eigen::MatrixXd PANDARobot::computeAngularJacobian(const int& link) {
	
	Eigen::MatrixXd Ja(SPACE_DIM,PANDA_JOINT_NUM), Jaa(SPACE_DIM, PANDA_JOINT_NUM);
	Eigen::Matrix3d Trpy;

	for (int i = 0;i <= link;++i) { //<=link because in the modified convention zi is on joint i not i+1!
		Ja.col(i) = this->getLinkRotMat(i).col(2);
	}

	//transform angular jacobian to be consistent with the used angle representation (RPY)
	//Eigen::Matrix3d R{ this->getEERotMat() };
	Eigen::Vector3d rpy = rot2rpy(this->getEERotMat());

	//get RPY transformation matrix
	double cP = cos(rpy(1));
	double sP = sin(rpy(1));
	double cY = cos(rpy(2));
	double sY = sin(rpy(2));

	Trpy << cP*cY,	- sY,	0,
			cP*sY,	  cY,	0,
			 -sP,     0,	1;
	this->Trpy = Trpy;

	Jaa = Trpy.inverse() * Ja;

	//return Jaa;
	return Ja;
}

void PANDARobot::resetDynParams() {

	// Reset the cumulative contributions of the residual
	this->dynModelSum.setZero(this->jointNum);
	this->residualSum.setZero(this->jointNum);

	// Reset the residual
	this->res__.setZero(this->jointNum);

	// Compute the new offset to be taken into account in the computation of the residual
	Eigen::VectorXd g = this->getModelDynParams().g;
	Eigen::VectorXd tau = this->getMsrJointTorques();
	Eigen::VectorXd resOff = g - tau;
	this->setResidualVectorOffset(resOff);

	//std::cout << "Robot dynamic paramters reset. " << std::endl;
}







/**
* @brief Delta-gravity Jacobian computation function
* Compute the delta-gravity jacobian contribution with the known joint positions, velocities and parameters of estimated the delta-dynamics
* @param q_k: the joint position vector
* @param fg: the estimated friction/gravity parameters
* @return the resulting delta gravity Jacobian matrix
*/
/*Eigen::MatrixXd PANDARobot::computeDeltaGravityJacobian(const Eigen::Vector7d& q_k, const Eigen::VectorXd& fg) {

	Eigen::MatrixXd Jg;
	Jg.setZero(JOINT_NUM, GRAVITY_DYN_PARAMS_NUM);
	float g0 = -9.81; // [m/s^2]

	float q2 = q_k(1);
	float q3 = q_k(2);
	float q4 = q_k(3);
	float q5 = q_k(4);
	float q6 = q_k(5);
	float q7 = q_k(6);

	float t2 = cos(q2);
	float t3 = s2;
	float t4 = cos(q3);
	float t5 = sin(q3);
	float t6 = cos(q4);
	float t7 = sin(q4);
	float t8 = c5;
	float t9 = g0*t2*t4*t7;
	float t10 = t9 - g0*t3*t6;
	float t11 = sin(q5);
	float t12 = sin(q6);
	float t13 = cos(q6);
	float t14 = g0*t2*t5*t8;
	float t15 = g0*t3*t7*t11;
	float t16 = g0*t2*t4*t6*t11;
	float t17 = sin(q7);
	float t18 = cos(q7);
	float t19 = g0*t2*t4*t7*t13;
	float t20 = g0*t2*t5*t11*t12;
	float t21 = t19 + t20 - g0*t3*t6*t13 - g0*t3*t7*t8*t12 - g0*t2*t4*t6*t8*t12;
	float t22 = g0*t2*t4*t7*(3.9E1 / 1.0E2);
	float t24 = g0*t3*(2.0 / 5.0);
	float t25 = g0*t3*t6*(3.9E1 / 1.0E2);
	float t23 = t22 - t24 - t25;
	float t26 = g0*t3*t4*t8;
	float t27 = g0*t3*t4*t11*t12;
	float t28 = g0*t3*t5*t6*t8*t12;
	float t29 = t27 + t28 - g0*t3*t5*t7*t13;
	float t30 = g0*t3*t4*t6;
	float t31 = t30 - g0*t2*t7;
	float t32 = g0*t3*t4*t6*t13;
	float t33 = g0*t2*t6*t8*t12;
	float t34 = g0*t3*t4*t7*t8*t12;
	float t35 = t32 + t33 + t34 - g0*t2*t7*t13;
	float t36 = g0*t3*t4*t6*(3.9E1 / 1.0E2);
	float t38 = g0*t2*t7*(3.9E1 / 1.0E2);
	float t37 = t36 - t38;
	float t39 = g0*t3*t4*t6*t8;
	float t40 = g0*t3*t5*t8*t12;
	float t41 = g0*t3*t4*t6*t11*t12;
	float t42 = t40 + t41 - g0*t2*t7*t11*t12;
	float t43 = g0*t2*t7*t8*t13;
	float t44 = g0*t3*t5*t11*t13;
	float t45 = t43 + t44 - g0*t2*t6*t12 - g0*t3*t4*t7*t12 - g0*t3*t4*t6*t8*t13;

	Jg(1,3) = g0*t2;
	Jg(1,5) = -g0*t3;
	Jg(1,6) = g0*t2*t4;
	Jg(1,7) = g0*t3;
	Jg(1,8) = -g0*t2*t5;
	Jg(1,9) = g0*t3*t7 + g0*t2*t4*t6;
	Jg(1,10) = -g0*t2*t5;
	Jg(1,11) = t10;
	Jg(1,12) = -g0*t2*t5*t11 + g0*t3*t7*t8 + g0*t2*t4*t6*t8;
	Jg(1,13) = t10;
	Jg(1,14) = t14 + t15 + t16;
	Jg(1,15) = -g0*t3*t6*t12 + g0*t2*t4*t7*t12 - g0*t2*t5*t11*t13 + g0*t3*t7*t8*t13 + g0*t2*t4*t6*t8*t13;
	Jg(1,16) = -t14 - t15 - t16;
	Jg(1,17) = t21;
	Jg(1,18) = -g0*t2*t5*t8*t17 - g0*t3*t7*t11*t17 - g0*t3*t6*t12*t18 - g0*t2*t4*t6*t11*t17 + g0*t2*t4*t7*t12*t18 - g0*t2*t5*t11*t13*t18 + g0*t3*t7*t8*t13*t18 + g0*t2*t4*t6*t8*t13*t18;
	Jg(1,19) = -g0*t2*t5*t8*t18 + g0*t3*t6*t12*t17 - g0*t3*t7*t11*t18 - g0*t2*t4*t6*t11*t18 - g0*t2*t4*t7*t12*t17 + g0*t2*t5*t11*t13*t17 - g0*t3*t7*t8*t13*t17 - g0*t2*t4*t6*t8*t13*t17;
	Jg(1,20) = t21;
	Jg(1,23) = g0*t3*(-2.0 / 5.0);
	Jg(1,24) = g0*t3*(-2.0 / 5.0);
	Jg(1,25) = t23;
	Jg(1,26) = t23;
	Jg(1,27) = t23;
	Jg(2,6) = -g0*t3*t5;
	Jg(2,8) = -g0*t3*t4;
	Jg(2,9) = -g0*t3*t5*t6;
	Jg(2,10) = -g0*t3*t4;
	Jg(2,11) = -g0*t3*t5*t7;
	Jg(2,12) = -g0*t3*t4*t11 - g0*t3*t5*t6*t8;
	Jg(2,13) = -g0*t3*t5*t7;
	Jg(2,14) = t26 - g0*t3*t5*t6*t11;
	Jg(2,15) = -g0*t3*t5*t7*t12 - g0*t3*t4*t11*t13 - g0*t3*t5*t6*t8*t13;
	Jg(2,16) = -t26 + g0*t3*t5*t6*t11;
	Jg(2,17) = t29;
	Jg(2,18) = -g0*t3*t4*t8*t17 + g0*t3*t5*t6*t11*t17 - g0*t3*t5*t7*t12*t18 - g0*t3*t4*t11*t13*t18 - g0*t3*t5*t6*t8*t13*t18;
	Jg(2,19) = -g0*t3*t4*t8*t18 + g0*t3*t5*t6*t11*t18 + g0*t3*t5*t7*t12*t17 + g0*t3*t4*t11*t13*t17 + g0*t3*t5*t6*t8*t13*t17;
	Jg(2,20) = t29;
	Jg(2,25) = g0*t3*t5*t7*(-3.9E1 / 1.0E2);
	Jg(2,26) = g0*t3*t5*t7*(-3.9E1 / 1.0E2);
	Jg(2,27) = g0*t3*t5*t7*(-3.9E1 / 1.0E2);
	Jg(3,9) = -g0*t2*t6 - g0*t3*t4*t7;
	Jg(3,11) = t31;
	Jg(3,12) = -g0*t2*t6*t8 - g0*t3*t4*t7*t8;
	Jg(3,13) = t31;
	Jg(3,14) = -g0*t2*t6*t11 - g0*t3*t4*t7*t11;
	Jg(3,15) = -g0*t2*t7*t12 + g0*t3*t4*t6*t12 - g0*t2*t6*t8*t13 - g0*t3*t4*t7*t8*t13;
	Jg(3,16) = g0*t2*t6*t11 + g0*t3*t4*t7*t11;
	Jg(3,17) = t35;
	Jg(3,18) = g0*t2*t6*t11*t17 - g0*t2*t7*t12*t18 + g0*t3*t4*t7*t11*t17 + g0*t3*t4*t6*t12*t18 - g0*t2*t6*t8*t13*t18 - g0*t3*t4*t7*t8*t13*t18;
	Jg(3,19) = g0*t2*t6*t11*t18 + g0*t2*t7*t12*t17 - g0*t3*t4*t6*t12*t17 + g0*t3*t4*t7*t11*t18 + g0*t2*t6*t8*t13*t17 + g0*t3*t4*t7*t8*t13*t17;
	Jg(3,20) = t35;
	Jg(3,25) = t37;
	Jg(3,26) = t37;
	Jg(3,27) = t37;
	Jg(4,12) = -g0*t3*t5*t8 + g0*t2*t7*t11 - g0*t3*t4*t6*t11;
	Jg(4,14) = t39 - g0*t2*t7*t8 - g0*t3*t5*t11;
	Jg(4,15) = -g0*t3*t5*t8*t13 + g0*t2*t7*t11*t13 - g0*t3*t4*t6*t11*t13;
	Jg(4,16) = -t39 + g0*t2*t7*t8 + g0*t3*t5*t11;
	Jg(4,17) = t42;
	Jg(4,18) = g0*t2*t7*t8*t17 + g0*t3*t5*t11*t17 - g0*t3*t4*t6*t8*t17 - g0*t3*t5*t8*t13*t18 + g0*t2*t7*t11*t13*t18 - g0*t3*t4*t6*t11*t13*t18;
	Jg(4,19) = g0*t2*t7*t8*t18 + g0*t3*t5*t11*t18 - g0*t3*t4*t6*t8*t18 + g0*t3*t5*t8*t13*t17 - g0*t2*t7*t11*t13*t17 + g0*t3*t4*t6*t11*t13*t17;
	Jg(4,20) = t42;
	Jg(5,15) = g0*t2*t6*t13 + g0*t3*t4*t7*t13 + g0*t2*t7*t8*t12 + g0*t3*t5*t11*t12 - g0*t3*t4*t6*t8*t12;
	Jg(5,17) = t45;
	Jg(5,18) = g0*t2*t6*t13*t18 + g0*t3*t4*t7*t13*t18 + g0*t2*t7*t8*t12*t18 + g0*t3*t5*t11*t12*t18 - g0*t3*t4*t6*t8*t12*t18;
	Jg(5,19) = -g0*t2*t6*t13*t17 - g0*t3*t4*t7*t13*t17 - g0*t2*t7*t8*t12*t17 - g0*t3*t5*t11*t12*t17 + g0*t3*t4*t6*t8*t12*t17;
	Jg(5,20) = t45;
	Jg(6,18) = -g0*t3*t5*t8*t18 - g0*t2*t6*t12*t17 + g0*t2*t7*t11*t18 - g0*t3*t4*t6*t11*t18 - g0*t3*t4*t7*t12*t17 + g0*t2*t7*t8*t13*t17 + g0*t3*t5*t11*t13*t17 - g0*t3*t4*t6*t8*t13*t17;
	Jg(6,19) = g0*t3*t5*t8*t17 - g0*t2*t7*t11*t17 - g0*t2*t6*t12*t18 + g0*t3*t4*t6*t11*t17 - g0*t3*t4*t7*t12*t18 + g0*t2*t7*t8*t13*t18 + g0*t3*t5*t11*t13*t18 - g0*t3*t4*t6*t8*t13*t18;

	return Jg;
}//*/

/**
* @brief Delta-gravity computation function
* Compute the delta-gravity contribution with the known joint positions, velocities and parameters of estimated the delta-dynamics
* @param q_k: the joint position vector
* @param fg: the estimated friction/gravity parameters
* @return the resulting delta gravity torques
*/
/*Eigen::VectorXd PANDARobot::computeDeltaGravity(const Eigen::Vector7d& q_k, const Eigen::VectorXd& fg){

	Eigen::VectorXd dg;
	dg.setZero(JOINT_NUM);

	// The vector of the estimating parameters of the delta-dynamics should be arranged as follows
	// fg = [Fs1, Fv1, sigK1, ..., Fs7, Fv7, sigK7, mx1, my1, mz1, ... , mx7, my7, mz7, m1, ..., m7]^T (size = 49)
	int fricOffset = frictionParamsNum * JOINT_NUM; //<-- TODO: from file
	int mxyzNum = SPACE_DIM * JOINT_NUM; //<-- TODO: from file
	float g0 = -9.81; // [m/s^2]

	float MX2 = fg(fricOffset + 3);
	float MZ2 = fg(fricOffset + 5);
	float MX3 = fg(fricOffset + 6);
	float MY3 = fg(fricOffset + 7);
	float MZ3 = fg(fricOffset + 8);

	float MX4 = fg(fricOffset + 9);
	float MY4 = fg(fricOffset + 10);
	float MZ4 = fg(fricOffset + 11);

	float MX5 = fg(fricOffset + 12);
	float MY5 = fg(fricOffset + 13);
	float MZ5 = fg(fricOffset + 14);

	float MX6 = fg(fricOffset + 15);
	float MY6 = fg(fricOffset + 16);
	float MZ6 = fg(fricOffset + 17);

	float MX7 = fg(fricOffset + 18);
	float MY7 = fg(fricOffset + 19);
	float MZ7 = fg(fricOffset + 20);

	float m3 = fg(fricOffset + mxyzNum + 2);
	float m4 = fg(fricOffset + mxyzNum + 3);
	float m5 = fg(fricOffset + mxyzNum + 4);
	float m6 = fg(fricOffset + mxyzNum + 5);
	float m7 = fg(fricOffset + mxyzNum + 6);

	// Build the delta gravity
	float q2 = q_k(1);
	float q3 = q_k(2);
	float q4 = q_k(3);
	float q5 = q_k(4);
	float q6 = q_k(5);
	float q7 = q_k(6);
	float t2 = s2;
	float t3 = cos(q2);
	float t4 = sin(q3);
	float t5 = cos(q4);
	float t6 = cos(q3);
	float t7 = sin(q4);
	float t8 = c5;
	float t9 = cos(q6);
	float t10 = sin(q5);
	float t11 = sin(q6);
	float t12 = cos(q7);
	float t13 = sin(q7);


	dg(1) = MX2*g0*t3 + MY3*g0*t2 - MZ2*g0*t2 - g0*m3*t2*(2.0 / 5.0) - g0*m4*t2*(2.0 / 5.0) - g0*m5*t2*(2.0 / 5.0) - g0*m6*t2*(2.0 / 5.0) - g0*m7*t2*(2.0 / 5.0) - g0*m5*t2*t5*(3.9E1 / 1.0E2) - g0*m6*t2*t5*(3.9E1 / 1.0E2) - g0*m7*t2*t5*(3.9E1 / 1.0E2) + MX3*g0*t3*t6 + MX4*g0*t2*t7 - MY4*g0*t3*t4 - MY5*g0*t2*t5 - MZ3*g0*t3*t4 - MZ4*g0*t2*t5 + MX4*g0*t3*t5*t6 + MX5*g0*t2*t7*t8 - MX5*g0*t3*t4*t10 - MX6*g0*t2*t5*t11 + MY5*g0*t3*t6*t7 - MY6*g0*t3*t4*t8 - MY6*g0*t2*t7*t10 + MZ4*g0*t3*t6*t7 + MZ5*g0*t3*t4*t8 - MZ6*g0*t2*t5*t9 - MZ7*g0*t2*t5*t9 + MZ5*g0*t2*t7*t10 + g0*m5*t3*t6*t7*(3.9E1 / 1.0E2) + g0*m6*t3*t6*t7*(3.9E1 / 1.0E2) + g0*m7*t3*t6*t7*(3.9E1 / 1.0E2) + MX5*g0*t3*t5*t6*t8 + MX6*g0*t2*t7*t8*t9 - MX6*g0*t3*t4*t9*t10 + MX6*g0*t3*t6*t7*t11 - MX7*g0*t3*t4*t8*t13 - MX7*g0*t2*t5*t11*t12 - MX7*g0*t2*t7*t10*t13 - MY6*g0*t3*t5*t6*t10 - MY7*g0*t3*t4*t8*t12 + MY7*g0*t2*t5*t11*t13 - MY7*g0*t2*t7*t10*t12 + MZ5*g0*t3*t5*t6*t10 + MZ6*g0*t3*t6*t7*t9 + MZ7*g0*t3*t6*t7*t9 - MZ6*g0*t2*t7*t8*t11 + MZ6*g0*t3*t4*t10*t11 - MZ7*g0*t2*t7*t8*t11 + MZ7*g0*t3*t4*t10*t11 + MX6*g0*t3*t5*t6*t8*t9 - MX7*g0*t3*t5*t6*t10*t13 + MX7*g0*t2*t7*t8*t9*t12 - MX7*g0*t3*t4*t9*t10*t12 + MX7*g0*t3*t6*t7*t11*t12 - MY7*g0*t3*t5*t6*t10*t12 - MY7*g0*t2*t7*t8*t9*t13 + MY7*g0*t3*t4*t9*t10*t13 - MY7*g0*t3*t6*t7*t11*t13 - MZ6*g0*t3*t5*t6*t8*t11 - MZ7*g0*t3*t5*t6*t8*t11 + MX7*g0*t3*t5*t6*t8*t9*t12 - MY7*g0*t3*t5*t6*t8*t9*t13;
	dg(2) = -MX3*g0*t2*t4 - MY4*g0*t2*t6 - MZ3*g0*t2*t6 - MX4*g0*t2*t4*t5 - MX5*g0*t2*t6*t10 - MY5*g0*t2*t4*t7 - MY6*g0*t2*t6*t8 - MZ4*g0*t2*t4*t7 + MZ5*g0*t2*t6*t8 - g0*m5*t2*t4*t7*(3.9E1 / 1.0E2) - g0*m6*t2*t4*t7*(3.9E1 / 1.0E2) - g0*m7*t2*t4*t7*(3.9E1 / 1.0E2) - MX5*g0*t2*t4*t5*t8 - MX6*g0*t2*t4*t7*t11 - MX6*g0*t2*t6*t9*t10 - MX7*g0*t2*t6*t8*t13 + MY6*g0*t2*t4*t5*t10 - MY7*g0*t2*t6*t8*t12 - MZ5*g0*t2*t4*t5*t10 - MZ6*g0*t2*t4*t7*t9 - MZ7*g0*t2*t4*t7*t9 + MZ6*g0*t2*t6*t10*t11 + MZ7*g0*t2*t6*t10*t11 - MX6*g0*t2*t4*t5*t8*t9 + MX7*g0*t2*t4*t5*t10*t13 - MX7*g0*t2*t4*t7*t11*t12 - MX7*g0*t2*t6*t9*t10*t12 + MY7*g0*t2*t4*t5*t10*t12 + MY7*g0*t2*t4*t7*t11*t13 + MY7*g0*t2*t6*t9*t10*t13 + MZ6*g0*t2*t4*t5*t8*t11 + MZ7*g0*t2*t4*t5*t8*t11 - MX7*g0*t2*t4*t5*t8*t9*t12 + MY7*g0*t2*t4*t5*t8*t9*t13;
	dg(3) = g0*m5*t3*t7*(-3.9E1 / 1.0E2) - g0*m6*t3*t7*(3.9E1 / 1.0E2) - g0*m7*t3*t7*(3.9E1 / 1.0E2) - MX4*g0*t3*t5 - MY5*g0*t3*t7 - MZ4*g0*t3*t7 - MX4*g0*t2*t6*t7 - MX5*g0*t3*t5*t8 - MX6*g0*t3*t7*t11 + MY5*g0*t2*t5*t6 + MY6*g0*t3*t5*t10 + MZ4*g0*t2*t5*t6 - MZ5*g0*t3*t5*t10 - MZ6*g0*t3*t7*t9 - MZ7*g0*t3*t7*t9 + g0*m5*t2*t5*t6*(3.9E1 / 1.0E2) + g0*m6*t2*t5*t6*(3.9E1 / 1.0E2) + g0*m7*t2*t5*t6*(3.9E1 / 1.0E2) - MX5*g0*t2*t6*t7*t8 + MX6*g0*t2*t5*t6*t11 - MX6*g0*t3*t5*t8*t9 + MX7*g0*t3*t5*t10*t13 - MX7*g0*t3*t7*t11*t12 + MY6*g0*t2*t6*t7*t10 + MY7*g0*t3*t5*t10*t12 + MY7*g0*t3*t7*t11*t13 + MZ6*g0*t2*t5*t6*t9 + MZ7*g0*t2*t5*t6*t9 - MZ5*g0*t2*t6*t7*t10 + MZ6*g0*t3*t5*t8*t11 + MZ7*g0*t3*t5*t8*t11 - MX6*g0*t2*t6*t7*t8*t9 + MX7*g0*t2*t5*t6*t11*t12 - MX7*g0*t3*t5*t8*t9*t12 + MX7*g0*t2*t6*t7*t10*t13 - MY7*g0*t2*t5*t6*t11*t13 + MY7*g0*t2*t6*t7*t10*t12 + MY7*g0*t3*t5*t8*t9*t13 + MZ6*g0*t2*t6*t7*t8*t11 + MZ7*g0*t2*t6*t7*t8*t11 - MX7*g0*t2*t6*t7*t8*t9*t12 + MY7*g0*t2*t6*t7*t8*t9*t13;
	dg(4) = -MX5*g0*t2*t4*t8 + MX5*g0*t3*t7*t10 + MY6*g0*t2*t4*t10 + MY6*g0*t3*t7*t8 - MZ5*g0*t2*t4*t10 - MZ5*g0*t3*t7*t8 - MX5*g0*t2*t5*t6*t10 - MX6*g0*t2*t4*t8*t9 + MX6*g0*t3*t7*t9*t10 + MX7*g0*t2*t4*t10*t13 + MX7*g0*t3*t7*t8*t13 - MY6*g0*t2*t5*t6*t8 + MY7*g0*t2*t4*t10*t12 + MY7*g0*t3*t7*t8*t12 + MZ5*g0*t2*t5*t6*t8 + MZ6*g0*t2*t4*t8*t11 + MZ7*g0*t2*t4*t8*t11 - MZ6*g0*t3*t7*t10*t11 - MZ7*g0*t3*t7*t10*t11 - MX6*g0*t2*t5*t6*t9*t10 - MX7*g0*t2*t5*t6*t8*t13 - MX7*g0*t2*t4*t8*t9*t12 + MX7*g0*t3*t7*t9*t10*t12 - MY7*g0*t2*t5*t6*t8*t12 + MY7*g0*t2*t4*t8*t9*t13 - MY7*g0*t3*t7*t9*t10*t13 + MZ6*g0*t2*t5*t6*t10*t11 + MZ7*g0*t2*t5*t6*t10*t11 - MX7*g0*t2*t5*t6*t9*t10*t12 + MY7*g0*t2*t5*t6*t9*t10*t13;
	dg(5) = MX6*g0*t3*t5*t9 - MZ6*g0*t3*t5*t11 - MZ7*g0*t3*t5*t11 + MX6*g0*t2*t6*t7*t9 + MX6*g0*t2*t4*t10*t11 + MX6*g0*t3*t7*t8*t11 + MX7*g0*t3*t5*t9*t12 - MY7*g0*t3*t5*t9*t13 + MZ6*g0*t2*t4*t9*t10 - MZ6*g0*t2*t6*t7*t11 + MZ7*g0*t2*t4*t9*t10 + MZ6*g0*t3*t7*t8*t9 - MZ7*g0*t2*t6*t7*t11 + MZ7*g0*t3*t7*t8*t9 - MX6*g0*t2*t5*t6*t8*t11 + MX7*g0*t2*t6*t7*t9*t12 + MX7*g0*t2*t4*t10*t11*t12 + MX7*g0*t3*t7*t8*t11*t12 - MY7*g0*t2*t6*t7*t9*t13 - MY7*g0*t2*t4*t10*t11*t13 - MY7*g0*t3*t7*t8*t11*t13 - MZ6*g0*t2*t5*t6*t8*t9 - MZ7*g0*t2*t5*t6*t8*t9 - MX7*g0*t2*t5*t6*t8*t11*t12 + MY7*g0*t2*t5*t6*t8*t11*t13;
	dg(6) = -MX7*g0*t2*t4*t8*t12 - MX7*g0*t3*t5*t11*t13 + MX7*g0*t3*t7*t10*t12 + MY7*g0*t2*t4*t8*t13 - MY7*g0*t3*t5*t11*t12 - MY7*g0*t3*t7*t10*t13 - MX7*g0*t2*t5*t6*t10*t12 + MX7*g0*t2*t4*t9*t10*t13 - MX7*g0*t2*t6*t7*t11*t13 + MX7*g0*t3*t7*t8*t9*t13 + MY7*g0*t2*t5*t6*t10*t13 + MY7*g0*t2*t4*t9*t10*t12 - MY7*g0*t2*t6*t7*t11*t12 + MY7*g0*t3*t7*t8*t9*t12 - MX7*g0*t2*t5*t6*t8*t9*t13 - MY7*g0*t2*t5*t6*t8*t9*t12;

	return dg;
}//*/

///**
//* @brief Set function
//* Save the friction parameters on a file
//* @param filename: the name of the file where the friction parameters have to be saved
//*/
//void PANDARobot::saveEstimatedFrictionParameters(const char* filename) {
//
//	std::stringstream fpSS;
//	DBWrapper db(filename);
//	Eigen::vectorXd fp;
//
//	fp = this->frictionGravityParams;
//	fpSS << "# Estimated friction parameters" << std::endl;
//	fpSS << "### Friction Parameters Size" << std::endl;
//	fpSS << frictionParamsNum << std::endl << std::endl;
//	fpSS << "### Friction Parameters Vector" << std::endl;
//	for (int i = 0; i < fp.size(); i++) {
//		fpSS << fp(i) << (i == fp.size() - 1 ? ";" : ",");
//	}
//
//	db.write(fpSS);
//
//}
//
///**
//* @brief Load function
//* Load the friction parameters from a file
//* @param filename: the name of the file where the friction parameters are stored
//*/
//void PANDARobot::loadEstimatedFrictionParameters(const char* filename) {
//
//	DBWrapper db(filename);
//	std::vector < std::pair < std::string, std::string > > content;
//
//	// Read the file
//	content = db.readLabeledFile();
//
//	// Parse the content file
//	for (int i = 0; i < content.size(); i++) {
//		std::string comment = content[i].first;
//		std::string value = content[i].second;
//
//		if (comment.find("Friction Parameters Size") != std::string::npos) {
//			frictionParamsNum = std::stod(value);
//		}
//		else if (comment.find("Friction Parameters Vector") != std::string::npos) {
//			std::vector < double > vec = parseCSVLine(value);
//			this->frictionGravityParams.setZero(vec.size());
//			for (int j = 0; j < vec.size(); j++) {
//				this->frictionGravityParams(j) = vec[j];
//			}
//		}
//	}
//
//	std::cout << "Friction paramters loaded = \n " << this->frictionGravityParams.transpose() << std::endl;
//
//}


