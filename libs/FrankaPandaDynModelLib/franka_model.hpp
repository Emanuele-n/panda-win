/*
 *
 *  Created on: 24 feb 2019
 *     Authors: Oliva Alexander, Gaz Claudio, Cognetti Marco
 *
 *  C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, 'Dynamic
 *  Identification of the Franka Emika Panda Robot With Retrieval of Feasible
 *  Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019.
 *
 */

 // MathLibrary.h - Contains declarations of math functions
#pragma once

#ifdef FRANKAPANDADYNMODELLIBDLL_EXPORTS
#define FRANKAPANDADYNMODELLIBDLL_API __declspec(dllexport)
#else
#define FRANKAPANDADYNMODELLIBDLL_API __declspec(dllimport)
#endif

#include "pch.h"
#include <math.h>

	const unsigned int njoints = 7;

	/*
	 * In order to speed up the code execution, we pre-compute the constant part
	 * of the friction model of each joint i (second term of the right member of
	 * the equation).
	 *
	 * tau_f(i) = FI_1(i)/(1+exp(-FI_2(i)*(dq(i)+FI_3(i)))) -
	 *                                           FI_1(i)/(1+exp(-FI_2(i)*FI_3(i)))
	 *
	 * For further information refer to our paper and relative Supplementary
	 * Material:
	 *      C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca,
	 * 'Dynamic Identification of the Franka Emika Panda Robot With Retrieval of
	 *   Feasible Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019.
	 */

	static const double  FI_11 = 0.54615;
	static const double  FI_12 = 0.87224;
	static const double  FI_13 = 0.64068;
	static const double  FI_14 = 1.2794;
	static const double  FI_15 = 0.83904;
	static const double  FI_16 = 0.30301;
	static const double  FI_17 = 0.56489;

	static const double  FI_21 = 5.1181;
	static const double  FI_22 = 9.0657;
	static const double  FI_23 = 10.136;
	static const double  FI_24 = 5.5903;
	static const double  FI_25 = 8.3469;
	static const double  FI_26 = 17.133;
	static const double  FI_27 = 10.336;

	static const double  FI_31 = 0.039533;
	static const double  FI_32 = 0.025882;
	static const double  FI_33 = -0.04607;
	static const double  FI_34 = 0.036194;
	static const double  FI_35 = 0.026226;
	static const double  FI_36 = -0.021047;
	static const double  FI_37 = 0.0035526;

	static const double TAU_F_CONST_1 = FI_11 / (1 + exp(-FI_21 * FI_31));
	static const double TAU_F_CONST_2 = FI_12 / (1 + exp(-FI_22 * FI_32));
	static const double TAU_F_CONST_3 = FI_13 / (1 + exp(-FI_23 * FI_33));
	static const double TAU_F_CONST_4 = FI_14 / (1 + exp(-FI_24 * FI_34));
	static const double TAU_F_CONST_5 = FI_15 / (1 + exp(-FI_25 * FI_35));
	static const double TAU_F_CONST_6 = FI_16 / (1 + exp(-FI_26 * FI_36));
	static const double TAU_F_CONST_7 = FI_17 / (1 + exp(-FI_27 * FI_37));



extern "C" FRANKAPANDADYNMODELLIBDLL_API void computeFrankaMassMatrix(double** B, double* q, double* dyn_pars_tip = NULL);
/*const double mL = 0,
const Eigen::Matrix4d& fMcom = Eigen::Matrix4d(),
const Eigen::Matrix3d& I_L = Eigen::Matrix3d::Identity());*/


extern "C" FRANKAPANDADYNMODELLIBDLL_API void  computeFrankaGravityVector(double* g, double* q, double* dyn_pars_tip = NULL);
/*const double mL = 0,
const Eigen::Matrix4d& fMcom = Eigen::Matrix4d(),
const Eigen::Vector3d& g0 = Eigen::Vector3d({ 0.0,0.0,-9.80665 }));//*/

extern "C" FRANKAPANDADYNMODELLIBDLL_API void  computeFrankaFrictionVector(double* tauf, double* dq);



extern "C" FRANKAPANDADYNMODELLIBDLL_API void computeFrankaCoriolisMatrix(double** C, double* q, double* dq, double* dyn_pars_tip = NULL);
/*const double mL = 0,
const Eigen::Matrix4d& fMcom = Eigen::Matrix4d(),
const Eigen::Matrix3d& I_L = Eigen::Matrix3d::Identity());*/


extern "C" FRANKAPANDADYNMODELLIBDLL_API void __skew(double** S, double* v);
