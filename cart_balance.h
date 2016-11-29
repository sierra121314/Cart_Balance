// Cart_Balance-Code.cpp : Defines the entry point for the console application.
// Min-Max Evolutionary Controllers Physics Portion

#ifndef _CART_BALANCE_CODE_
#define _CART_BALANCE_CODE_

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <vector>
#include <iostream>

// Establish boundaries
// Table boundaries
// gravity = 9.81; // m/s

// Initialize knowns
const float g = 9.81f; // m/s^2
const int n = 1; // iterations
static const double dt = 0.1; //time step

////Struct has function 
//Class has public, private, function and inheritance

namespace CB {

	// Cart
	// Establish cart boundaries
	// cart movements - left and right
	// Cart starting position
	// cart position - x
	// cart velocity - xdot 
	// forces on the cart
	// weight of cart
	struct Cart {

		const int mass_c; // Mass of the Cart
		const int y; // Y position of Cart - Irrelevant due to gravity
		const int y_dot; // Velocity of y component of Cart - Irrelevant due to gravity
		const int y_dd; // Y double dot - Y component of acceleration of Cart

				// Changing Variables
		double x; // X Position of Cart
		double x_dot; // Velocity of x component of Cart
		double x_dd; // Acceleration of x component of Cart
		// Cart cart;
		// cart.mass_c = 100;
	};

	///////////////////////////////////
	// Establish Pendulum boundaries //
	///////////////////////////////////
	struct Pend_state {
		// ball position - theta
		double theta; // ever changing theta - main objective to keep theta around 90*
		// ball position
		double Px; // x coordinate of Pendulum;
		double Py; // y coordinate of Pendulum;{
			
		double theta_dot; // ever changing velocity of theta.
		double theta_dd; // acceleration of theta.
	};


	class Pendulum {
	private:
		// static variables
		double mass_p; // Mass of Pendulum
		double length; // Length of the Pendulum
		
		// collection of states
		std::vector <Pend_state> pend;

		// Changing variables
		double torq;
		double I;
		double reward; //last reward for the last action

		// public functions
		double determine_reward();
	public:
		Pendulum();
		std::vector <double> give_reward();
		std::vector <double> give_state();
		void get_action(std::vector <double>);
		void cycle();
	};

	Pendulum::Pendulum()
	{
		Pend_state initial;
		mass_p = 50;
		length = 10;
		initial.theta = 45 * M_PI / 180;
		initial.Px = length*cos(initial.theta);
		initial.Py = length*cos(initial.theta);
		// initialize theta_dot=0 and theta double dot= little less that 90 degrees
		initial.theta_dot = 0; // rad/s // theta dot of this specific pendulum
		initial.theta_dd = 0;

		pend.push_back(initial); //push_back pushes it to the back of the vector

	}



	void Pendulum::cycle() {

		// variables
		Pend_state nextState;
		//note

		// does all necessary calculations, given an action (already set from set_action), to arrive at the next state at the next timestep.
		//torque to theta dd
		nextState.theta_dd = -g*cos(pend.at(pend.size() - 1).theta) / (mass_p*length) + torq; //rad/s^2   // define theta_dd with t variable 
		//thetat_dd to theta_dot
		nextState.theta_dot = pend.at(pend.size() - 1).theta_dot + nextState.theta_dd*dt;
		//theta_dot to theta
		nextState.theta = pend.at(pend.size() - 1).theta + nextState.theta_dot*dt;
		//theta to xy
		nextState.Px = length*cos(nextState.theta);
		nextState.Py = length*sin(nextState.theta);

		std::cout << nextState.theta << "," << nextState.theta_dot << "," << nextState.theta_dd << "," << nextState.Px << "," << nextState.Py << std::endl;

		pend.push_back(nextState); //update state

		reward = determine_reward();

		std::ofstream fout;
		fout.clear();
		fout.open("positiondata2.csv", std::ofstream::out | std::ofstream::app);
		for (int i = 0; i < n; i++) { // for i number of iterations 
									  //calculate xy
									  //use theta
									  //output xy
			//fout << pend.at(pend.end()-1).Px << "," << pend.at(pend.end()-1).Py << "," << dt*i << "," \
				<< pend.give_state(pend.end()-1).theta << "," << pend.give_state(pend.end()-1).theta_dot << "," \
				<< pend.give_state(pend.end()-1).theta_dd << std::endl;
		}  // replace give_state with .at
		fout.close();

	}

	double Pendulum::determine_reward() {
		double temp_reward;
		if theta == 90 && theta_dot == 0 {
			temp_reward = 0.0;
			else 
				temp_reward = 1;
			end
		}
		return temp_reward;
	}

	void Pendulum::get_action(std::vector <double> in_action) { //receives "action vection", which in the first case will just consist of the torque at the joint

		//name vector
		// torq=t@0
		torq = in_action.at(0); 
		cycle();
	}

	std::vector <double> Pendulum::give_state() {
		//gives the state of the pendulum at the given timestep
		std::vector <double> temp_state;
		temp_state.push_back(pend.at(pend.size()-1).theta);
		temp_state.push_back(pend.at(pend.size()-1).theta_dot);
		return temp_state;
	}
	std::vector <double> Pendulum::give_reward() {
		std::vector <double> temp_reward;
		temp_reward.push_back(reward);

		return temp_reward;
	}
}

#endif // !