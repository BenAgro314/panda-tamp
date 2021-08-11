#pragma once

#include <array>
#include <franka/robot.h>
#define DOF 7

extern const std::array<double, DOF> GRASP_SIG = {99, 99, 99, 99, 99, 99, 99};
extern const std::array<double, DOF> RELEASE_SIG = {-99, -99, -99, -99, -99, -99, -99};

/**
 * Sets a default collision behavior, joint impedance, Cartesian impedance, and filter frequency.
 *
 * @param[in] robot Robot instance to set behavior on.
 */
void setDefaultBehavior(franka::Robot& robot) {
  robot.setCollisionBehavior(
      {{40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0}},
      {{40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{40.0, 40.0, 40.0, 40.0, 40.0, 40.0}},
      {{40.0, 40.0, 40.0, 40.0, 40.0, 40.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, 
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

bool isConfValid(std::array<double, DOF> q){
	std::array<double, DOF> q_max = {{2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973}};
	std::array<double, DOF> q_min = {{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}};
	for (int i = 0; i < DOF; i++){
		if ((q[i] > q_max[i]) || (q[i] < q_min[i])){
			return false;
		}
	}
	return true;
}

void printConf(std::array<double, DOF> q){
    for (int i = 0; i < DOF; i++){
        std::cout << q[i];
        if (i == q.size() - 1) {
            std::cout << std::endl;
        } else {
            std::cout << " ";
        }
    }
}