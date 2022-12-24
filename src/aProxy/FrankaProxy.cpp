// Standard Header files
#include <iostream>

// Project Header files
#include "FrankaProxy.hpp"

#ifndef PANDA_JOINT_NUM
#define PANDA_JOINT_NUM 7
#endif

#ifndef ESCAPE
#define ESCAPE 27
#endif 


FrankaProxy::FrankaProxy() : RobotProxy() {

	this->rs.initialized = false;
	this->cmdReady = false;
	this->callback_enabled = false;
	this->callback_running = false;
	this->dataNew = false;
}

FrankaProxy::~FrankaProxy() {

	delete this->panda;

	if (this->isGripperMounted()) {
		delete this->gripper;
	}
}

void FrankaProxy::init() {

	std::cout << "Initializing FrankaProxy class ... " << std::endl;
    
    // Connect to robot
	std::cout << "Trying to connect to 172.16.0.2 ... " << std::endl;
	this->panda = new franka::Robot("172.16.0.2");

	setDefaultBehavior(*this->panda);

	// Move the robot autonomously to a starting initial configuration
	std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
	MotionGenerator motion_generator(0.2, q_goal);
	this->panda->control(motion_generator);

    // Load the kinematics and dynamics model
    franka::Model model = this->panda->loadModel();
    franka::RobotState initial_state = this->panda->readOnce();

    // Equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.linear());

    // Set collision behavior
	this->panda->setCollisionBehavior(
		{ {40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0} }, { {40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0} },
		{ {40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0} }, { {40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0} },
		{ {10.0, 10.0, 10.0, 15.0, 15.0, 15.0} }, { {60.0, 60.0, 60.0, 65.0, 65.0, 65.0} },
		{ {10.0, 10.0, 10.0, 15.0, 15.0, 15.0} }, { {60.0, 60.0, 60.0, 65.0, 65.0, 65.0} });

	// Set additional parameters always before the control loop, NEVER in the control loop!
	// Set the joint impedance.
	//this->panda->setJointImpedance({ {10000, 10000, 10000, 10000, 10000, 10000, 10000} });
	//this->panda->setCartesianImpedance({ {100, 100, 10, 100, 100, 100} });
	//this->panda->setCartesianImpedance({ {3000, 3000, 50, 300, 300, 300} });

	// TODO: Call readOnce functions to initialize rs structure here

	// Reload kinematic (why?)
	franka::RobotState init_state = this->panda->readOnce();

	// Get kinematic and dynamic model elements
	std::array<double, 7> gravity_array = model.gravity(init_state);
	std::array<double, 7> coriolis_array = model.coriolis(init_state);
	std::array<double, 49> mass_array = model.mass(init_state);
	
	Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> q_init(init_state.q.data());
	Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> dq_init(init_state.dq.data());
	Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> tau_init(init_state.tau_J.data());
	Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> tau_c_init(init_state.tau_ext_hat_filtered.data());
	Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> gravity(gravity_array.data());
	Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> coriolis(coriolis_array.data());
	Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, PANDA_JOINT_NUM>> mass(mass_array.data());
	Eigen::Map<const Eigen::Vector6d> f_c_init(init_state.K_F_ext_hat_K.data());
	Eigen::Map<const Eigen::Matrix4d> T0ee_init(init_state.O_T_EE.data());

	// Init robot structure parameters
	this->rs.msrJointPosition = q_init.cast<float>();
	this->rs.msrJointVelocity = dq_init.cast<float>();
	this->rs.msrJointTorque = tau_init.cast<float>();
	this->rs.extEstJointTorque = tau_c_init.cast<float>();
	this->rs.extEstCartForce = f_c_init.cast<float>();
	this->rs.g_lib = gravity.cast<float>();
	this->rs.c_lib = coriolis.cast<float>();
	this->rs.m_lib = mass.cast<float>();
	this->rs.T0ee = T0ee_init.cast<float>();
	this->rs.initialized = true;
	this->rs.cmdJointTorque.setZero(PANDA_JOINT_NUM);
	this->rs.cmdJointVelocity.setZero(PANDA_JOINT_NUM);

	// Default Gripper initialization is false
	this->gripperMounted = false;

    std::cout << "Franka robot initialization completed. " << std::endl;

}

void FrankaProxy::initGripper() {

	this->gripper = new franka::Gripper("172.16.0.2");

	// Put the gripper on homing
	this->gripper->homing();

	// Keep the gripper closed
	this->gripper->move(0.0, 0.01);

	// Should I update the Franka Panda kinematics and dynamics

}

void FrankaProxy::mainLoop() {

	Eigen::Vector7d tau_cmd, qd_cmd;
	tau_cmd.setZero();
	qd_cmd.setZero();
	franka::Torques zero_torques{ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0} };
	franka::JointVelocities zero_velocities{ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0} };

	std::cout << "FrankaProxy::mainLoop() started " << std::endl;

	// Load the kinematics and dynamics model
	franka::Model model = this->panda->loadModel();

	// Define the callback
	double time = 0.0;
	double time0;
	
	// Get robot parameters
	franka::RobotState init_state = this->panda->readOnce();
	std::array<double, 7> gravity_array = model.gravity(init_state);
	std::array<double, 7> coriolis_array = model.coriolis(init_state);
	std::array<double, 49> mass_array = model.mass(init_state);
	std::array<double, 16> stiffness_frame;
	std::array<double, 42> Jee_arr_init = model.zeroJacobian(franka::Frame::kEndEffector, init_state);
	Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> q_init(init_state.q.data());
	Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> dq_init(init_state.dq.data());
	Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> tau_init(init_state.tau_J.data());
	Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> tau_c_init(init_state.tau_ext_hat_filtered.data());
	Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> gravity(gravity_array.data());
	Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> coriolis(coriolis_array.data());
	Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, PANDA_JOINT_NUM>> mass(mass_array.data());
	Eigen::Map<const Eigen::Matrix<double, 6, 7>> J_init(Jee_arr_init.data());
	Eigen::Map<const Eigen::Vector6d> f_c_init(init_state.K_F_ext_hat_K.data());
	Eigen::Map<const Eigen::Matrix4d> T0ee_init(init_state.O_T_EE.data());

	// Stiffness frame seems to be not used
	for (int i = 0; i < 16; i++) {
		stiffness_frame[i] = 0.0;
	}
	stiffness_frame[0] = 1.0;
	stiffness_frame[5] = 1.0;
	stiffness_frame[10] = 1.0;
	stiffness_frame[15] = 1.0;
	stiffness_frame[14] = 0.05;
	//this->panda->setK(stiffness_frame);

	// Set robot parameters
	this->rs.msrJointPosition = q_init.cast<float>();
	this->rs.msrJointVelocity = dq_init.cast<float>();
	this->rs.msrJointTorque = tau_init.cast<float>();
	this->rs.extEstJointTorque = tau_c_init.cast<float>();
	this->rs.extEstCartForce = f_c_init.cast<float>();
	this->rs.g_lib = gravity.cast<float>();
	this->rs.c_lib = coriolis.cast<float>();
	this->rs.m_lib = mass.cast<float>();
	this->rs.T0ee = T0ee_init.cast<float>();
	this->rs.T0ee_des = T0ee_init.cast<float>();
	this->rs.initialized = true;
	this->rs.Jac_lib.setZero(6, 7);
	this->rs.Jac_lib = J_init.cast<float>();

	time0 = init_state.time.toSec();
	this->rs.stamp = time0;

	// Set running on true
	this->running = true;
	this->callback_enabled = true;

	// [PRINT THREAD]
	// Initialize data fields for the print thread.
	const double print_rate = 1000.0;
	struct {
		std::mutex mutex;
		bool has_data;
		Eigen::Vector7d tau_d_limited;
		Eigen::Vector7d tau_d_computed;
		franka::RobotState robot_state;
		Eigen::Vector7d gravity;
		Eigen::Vector7d coriolis;
		Eigen::Matrix<double, 6, 1> err_;
		Eigen::Matrix<double, 6, 1> err_v_;
		Eigen::Vector3d pdes;
		Eigen::Vector3d p;
		Eigen::Matrix<double, 6, 1> PIDaction;
		Eigen::Quaterniond orientation;
		Eigen::Quaterniond orientation_d;
		Eigen::Vector6d velocity;
		Eigen::Matrix<double, 7, 1> qdot;
		double time;
		double rate;
	} print_data{};

	print_data.time = time0;
	double print_time = 0.0;

	std::thread print_thread([print_rate, &print_data, &print_time, this]() {

		auto now_ = std::chrono::steady_clock::now();
		auto last_ = now_;

		while (this->running) {
			// Sleep to achieve the desired print rate.
			//std::this_thread::sleep_for(
			//std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));

			// Update time
			now_ = std::chrono::steady_clock::now();
			auto tic = now_;
			auto dt = round<std::chrono::microseconds>(now_ - last_).count() * 1e-6;
			last_ = now_;
			t_ += dt;

			// Try to lock data to avoid read write collisions.
			if (print_data.mutex.try_lock()) {
				if (print_data.has_data) {
					std::array<double, 7> tau_error{};
					double error_rms(0.0);
					std::array<double, 7> tau_d_actual{};
					std::array<double, 7> tau_d_actual_no_ratelimit{};
					for (size_t i = 0; i < 7; ++i) {
						tau_d_actual[i] = print_data.tau_d_limited[i] + print_data.gravity[i];
						tau_d_actual_no_ratelimit[i] = print_data.tau_d_computed[i] + print_data.gravity[i];
						tau_error[i] = tau_d_actual[i] - print_data.robot_state.tau_J[i]; //* tau_J: Measured link-side joint torque sensor signals. Unit: \f$[Nm]\f$
						error_rms += std::pow(tau_error[i], 2.0) / tau_error.size();
					}
					error_rms = std::sqrt(error_rms);

					//std::cout << "time = " << print_data.time << std::endl;
					//std::cout << "ctrl stamp = " << this->rs.stamp << std::endl;
					//std::cout << "tau = " << print_data.tau_d_computed.transpose() << std::endl;
					//std::cout << "qdot = " << print_data.qdot.transpose() << std::endl;
					//std::cout << "pdes = " << print_data.pdes.transpose() << std::endl;
					//std::cout << "p = " << print_data.p.transpose() << std::endl;
					//std::cout << "PID action = " << print_data.PIDaction.transpose() << std::endl;
					//std::cout << "velocity = " << print_data.velocity.transpose() << std::endl;
					//std::cout << "orientation = " << print_data.orientation.coeffs().transpose() << std::endl;
					//std::cout << "qdot = " << print_data.qdot.transpose() << std::endl;
					//std::cout << "err = " << print_data.err_.transpose() << std::endl;
					//std::cout << "tau = " << print_data.tau_d_computed.transpose() << std::endl;
					//std::cout << std::endl;

					// Print data to console
					//std::cout << print_data.err_.transpose() << std::endl;
					//std::cout << "error (position, orientation) = " << print_data.err_.transpose() << std::endl;
					print_data.has_data = false;
				}
				else {
					//std::cout << "No new data." << std::endl;
				}
				print_data.mutex.unlock();
			}

			auto toc = std::chrono::steady_clock::now();
			auto tictoc = round<std::chrono::microseconds>(toc - tic).count() * 1e-6;
			std::this_thread::sleep_for(
				std::chrono::milliseconds(static_cast<int>(((1.0 / print_rate - tictoc)* 1000.0))));
			auto tac = std::chrono::steady_clock::now();
			print_time += round<std::chrono::microseconds>(tac - tic).count() * 1e-6;
		}
	});

	// Initial 0_T_ee
	std::cout << "Initial 0_T_ee = \n " << T0ee_init << "\n" << std::endl;
	std::array<double, 16> init_pose = { T0ee_init(0,0), T0ee_init(1,0), T0ee_init(2,0), T0ee_init(3,0),
								   T0ee_init(0,1), T0ee_init(1,1), T0ee_init(2,1), T0ee_init(3,1),
								   T0ee_init(0,2), T0ee_init(1,2), T0ee_init(2,2), T0ee_init(3,2),
								   T0ee_init(0,3), T0ee_init(1,3), T0ee_init(2,3), T0ee_init(3,3) };


	auto set_cartpose_callback = [&, this](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {

		Eigen::Matrix4f Tref = this->rs.T0ee_des;
		std::array<double, 16> new_pose = { Tref(0,0), Tref(1,0), Tref(2,0), Tref(3,0),
											Tref(0,1), Tref(1,1), Tref(2,1), Tref(3,1),
											Tref(0,2), Tref(1,2), Tref(2,2), Tref(3,2),
											Tref(0,3), Tref(1,3), Tref(2,3), Tref(3,3) };

		if (!this->callback_enabled) {
			std::cout << std::endl << "Finished test, shutting down example" << std::endl;
			return franka::MotionFinished(init_pose);
		}

		return new_pose;
	
	};

	auto set_qdot_commands_callback = [&, this](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities {

		// Get robot state
		std::array<double, 7> gravity_array = model.gravity(robot_state);
		std::array<double, 7> coriolis_array = model.coriolis(robot_state);
		std::array<double, 49> mass_array = model.mass(robot_state);
		std::array<double, 42> Jee_arr = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
		Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> q(robot_state.q.data());
		Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> dq(robot_state.dq.data());
		Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> tau(robot_state.tau_J.data());
		Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> tau_c(robot_state.tau_ext_hat_filtered.data());
		Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> gravity(gravity_array.data());
		Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> coriolis(coriolis_array.data());
		Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, PANDA_JOINT_NUM>> mass(mass_array.data());
		Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(Jee_arr.data());
		Eigen::Map<const Eigen::Vector6d> f_c(robot_state.K_F_ext_hat_K.data());
		Eigen::Map<const Eigen::Matrix4d> T0ee(robot_state.O_T_EE.data());

		this->rs.msrJointPosition = q.cast<float>();
		this->rs.msrJointVelocity = dq.cast<float>();
		this->rs.msrJointTorque = tau.cast<float>();
		this->rs.extEstJointTorque = tau_c.cast<float>();
		this->rs.extEstCartForce = f_c.cast<float>();
		this->rs.g_lib = gravity.cast<float>();
		this->rs.c_lib = coriolis.cast<float>();
		this->rs.m_lib = mass.cast<float>();
		this->rs.T0ee = T0ee.cast<float>();
		this->rs.stamp = robot_state.time.toSec() - time0;
		this->rs.Jac_lib = J.cast<float>();

		qd_cmd = this->rs.cmdJointVelocity.cast<double>();
		//qd_cmd.setZero();
		std::array<double, 7> qd_d_array{};
		Eigen::VectorXd::Map(&qd_d_array[0], 7) = qd_cmd;
		franka::JointVelocities velocities(qd_d_array);

		if (!this->callback_enabled) {
			std::cout << std::endl << "Finished test, shutting down example" << std::endl;
			return franka::MotionFinished(zero_velocities);
		}

		return velocities;
	};

	auto set_tau_commands_callback = [&, this](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {

		// Get robot state
		std::array<double, 7> gravity_array = model.gravity(robot_state);
		std::array<double, 7> coriolis_array = model.coriolis(robot_state);
		std::array<double, 49> mass_array = model.mass(robot_state);
		Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> q(robot_state.q.data());
		Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> dq(robot_state.dq.data());
		Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> tau(robot_state.tau_J.data());
		Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> tau_c(robot_state.tau_ext_hat_filtered.data());
		Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> gravity(gravity_array.data());
		Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> coriolis(coriolis_array.data());
		Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, PANDA_JOINT_NUM>> mass(mass_array.data());
		Eigen::Map<const Eigen::Vector6d> f_c(robot_state.K_F_ext_hat_K.data());
		Eigen::Map<const Eigen::Matrix4d> T0ee(robot_state.O_T_EE.data());
		std::array<double, 42> Jee_arr = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
		Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(Jee_arr.data());
		Eigen::Vector6d vel = J * dq;
		Eigen::Matrix3d Ree = T0ee.topLeftCorner(3, 3);
		Eigen::Vector3d pos = T0ee.block<3, 1>(0, 3);

		//this->callback_running = true;
		this->rs.msrJointPosition = q.cast<float>();
		this->rs.msrJointVelocity = dq.cast<float>();
		this->rs.msrJointTorque = tau.cast<float>();
		this->rs.extEstJointTorque = tau_c.cast<float>();
		this->rs.extEstCartForce = f_c.cast<float>();
		this->rs.g_lib = gravity.cast<float>();
		this->rs.c_lib = coriolis.cast<float>();
		this->rs.m_lib = mass.cast<float>();
		this->rs.T0ee = T0ee.cast<float>();

		//tau_cmd = (this->getSafeCmdJointTorques()).cast<double>();
		tau_cmd = this->rs.cmdJointTorque.cast<double>();

		// Update data to print.
		if (print_data.mutex.try_lock()) {
			print_data.has_data = true;
			print_data.time = robot_state.time.toSec() - time0;
			print_data.robot_state = robot_state;
			print_data.tau_d_limited = tau_cmd;
			print_data.tau_d_computed = tau_cmd;
			print_data.gravity = gravity;
			print_data.coriolis = coriolis;
			print_data.rate = 1.0 / period.toSec();
			print_data.orientation = Eigen::Quaterniond(Ree);
			print_data.velocity = vel;
			print_data.qdot = dq;
			print_data.PIDaction = this->rs.PIDaction.cast<double>();
			print_data.pdes = this->rs.pdes.cast<double>();
			print_data.p = pos;
			print_data.err_ = this->rs.ctrlErr.cast<double>();
			print_data.mutex.unlock();
		}//*/

		//tau_cmd.setZero();
		std::array<double, 7> tau_d_array{};
		Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_cmd;

		if (!this->callback_enabled) {
			std::cout << "Exiting from set_tau_commands_callback function ... " << std::endl;
			return franka::MotionFinished(zero_torques);
		}

		return tau_d_array;

	};

	auto set_zero_tau_commands_callback = [&, this](const franka::RobotState& robot_state,	franka::Duration period) -> franka::Torques {
		return franka::MotionFinished(zero_torques);
	};

	while (this->running) {
	
		try {
			// Test Callback function
			if (this->ctrlLevel == ROBOT_CONTROL_TYPE::KINEMATIC_CONTROL) {
				if (this->callback_enabled) {
					this->callback_running = true;
					std::cout << "Running the kinematic control callback on Franka robot ... " << std::endl;
					std::cout << "\nTask execution started\nPress any key to stop" << std::endl;
					this->panda->control(set_qdot_commands_callback, franka::ControllerMode::kCartesianImpedance, true, 2000.0);
				}
			}
			else if (this->ctrlLevel == ROBOT_CONTROL_TYPE::TORQUE_CONTROL) {
				if (this->callback_enabled) {
					this->callback_running = true;
					std::cout << "Running the torque control callback on Franka robot ... " << std::endl;
					std::cout << "\nTask execution started\nPress any key to stop\n" << std::endl;
					this->panda->control(set_tau_commands_callback);
					std::cout << "Exited from set_tau_commands_callback. " << std::endl;
				}
			}
			/*else if (this->ctrlLevel == ROBOT_CONTROL_TYPE::IDLE_CONTROL) {
				Timer clock;
				clock.timeSleep(0.01);
			}*/
			//this->panda->control(set_cartpose_callback, franka::ControllerMode::kCartesianImpedance, true, 10.0);
			this->callback_running = false;
			std::cout << "[FP] this->callback_running = " << this->callback_running << std::endl;
		}
		catch (franka::Exception const& e) {
			std::cout << e.what() << std::endl;

			// This line below is FUNDAMENTAL, because it allows to automatically handle the 
			// violation of communication real time constraints and to recover the program without
			// interrupting anything
			if (std::string(e.what()).find("communication") != std::string::npos){
				this->panda->automaticErrorRecovery();
			}
			else {

				std::cout << "Stopping the Franka Panda robot ... " << std::endl;
				this->panda->stop();
			}

		}

	}

	// Turn off motors if the control is kinematic with a dedicated dummy callback acting on the torques
	if (this->ctrlLevel == ROBOT_CONTROL_TYPE::KINEMATIC_CONTROL) {
		this->panda->control(set_zero_tau_commands_callback, false, 10.0);
	}

	if (print_thread.joinable()) {
		print_thread.join();
	}

	// Apparently, I cannot use both a read() and a control() callback function, but only one of them.
	// On the other hand, when calling the control() function the information about the current robot state is available, 
	// since it has been designed so as to implement the control law in the control() callback itself. However, since I would like to 
	// lighten as much as possible the callback to keep the real-time rate guaranteed, I would compute the control law separately in a different
	// class with a different rate, and leave to the control() callback only the responsibility of forwarding the torque inputs to the robot. 
	// With this setup, the state retrieved in the callback will always be one step behind the applied control input. This may not be a big deal,
	// since the control thread will likely run slower than the Franka thread (100Hz).
	/*this->panda->read([&, this](const franka::RobotState& robot_state) {
		// Printing to std::cout adds a delay. This is acceptable for a read loop such as this,
		// but should not be done in a control loop.
		

		Eigen::Map<const Eigen::Matrix<double, PANDA_JOINT_NUM, 1>> q(robot_state.q.data());

		// TODO: get the rest of the read function
		this->rs.msrJointPosition = q.cast<float>();
		this->rs.stamp = robot_state.time.toSec() - time0;
		

		return this->running;
	});*/
}

void FrankaProxy::moveJoint(){
    
	// TODO: get configuration from input/file

    // Connect to robot
	std::cout << "Trying to connect to 172.16.0.2 ... " << std::endl;
	this->panda = new franka::Robot("172.16.0.2");

	setDefaultBehavior(*this->panda);
	std::cout << "Franka robot initialization completed. " << std::endl;

	// TEST to move the robot autonomously to a starting initial configuration
	std::array<double, 7> q_goal = { {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4} };
	MotionGenerator motion_generator(0.2, q_goal);
	this->panda->control(motion_generator);
}


// Get functions
Eigen::VectorXf FrankaProxy::getMsrJointPosition() {

	Eigen::VectorXf qmsr(this->jointDOFs);
	return qmsr;

}

Eigen::VectorXf FrankaProxy::getMsrJointTorque() {

	Eigen::VectorXf tau(this->jointDOFs);
	return tau;

}

Eigen::VectorXf FrankaProxy::getEstExtJointTorque() {

	Eigen::VectorXf ext_tau(this->jointDOFs);

	return ext_tau;

}

Eigen::Vector6f FrankaProxy::getEstExtCartForce() {
	
	Eigen::Vector6f f_ext;

	return f_ext;

}

// Set functions
void FrankaProxy::setStiffnessFrame(const Eigen::Matrix4d& T) {

	std::array<double, 16> T0ee = { T(0,0), T(1,0), T(2,0), T(3,0),
									T(0,1), T(1,1), T(2,1), T(3,1),
									T(0,2), T(1,2), T(2,2), T(3,2),
									T(0,3), T(1,3), T(2,3), T(3,3) };

	for (int i = 0; i < 16; i++) {
		std::cout << T0ee[i] << ", ";
	}
	std::cout << std::endl;
	//this->panda->setK(T0ee);

}

void FrankaProxy::setJointTorqueCommands(const Eigen::VectorXf& taucmd) {this->rs.cmdJointTorque = taucmd;}

void FrankaProxy::setJointVelocityCommands(const Eigen::VectorXf& qdcmd) {this->rs.cmdJointVelocity = qdcmd;}

void FrankaProxy::setControlInputPIDAction(const Eigen::Vector6f& pid) {this->rs.PIDaction = pid;}

void FrankaProxy::setControlError(const Eigen::Vector6f& err) {this->rs.ctrlErr = err;}

void FrankaProxy::setPdes(const Eigen::Vector3f& pdes) {this->rs.pdes = pdes;}

// Empty body functions
void FrankaProxy::clear() {}

void FrankaProxy::setEstExtCartForce(const Eigen::Vector6f& f) {}

void FrankaProxy::setEstExtJointTorque(const Eigen::VectorXf& tau) {}

void FrankaProxy::setJointPositionCommands(const Eigen::VectorXf& qcmd) {}

void FrankaProxy::setJointRefs(const Eigen::VectorXf& jref) {}

void FrankaProxy::setMsrJointPosition(const Eigen::VectorXf& q) {}

void FrankaProxy::setMsrJointTorque(const Eigen::VectorXf& tau){}



