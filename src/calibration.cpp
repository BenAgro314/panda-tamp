#include <iostream>
#include <array>

#include <franka/exception.h>
#include <franka/robot.h>

#include "position_controller.h"
#include "parse_confs.h"


int main(int argc, char** argv){
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
    double percent_max_speed = 0.2;

    try {
        franka::Robot robot(argv[1]);
        franka::Gripper gripper(argv[1]);
        setDefaultBehavior(robot);

        std::array<double, 7> q_goal = {{0.0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(percent_max_speed, q_goal);
        std::cout << "WARNING: This example will move the robot! "
                << "Please make sure to have the user stop button at hand!" << std::endl
                << "Press Enter to continue..." << std::endl;
        std::cin.ignore();

        gripper.homing();
        robot.control(motion_generator);
        std::cout << "Finished moving to initial joint configuration." << std::endl;

        if (!gripper.move(0, 0.1)) {
            std::cout << "Failed to close gripper" << std::endl;
            return -1;
        }

        auto calibration_list = parseConfsFile("./shorter_calibration_confs.txt");
        assert(calibration_list.size() % 2 == 0);
        for (int i = 0; i < calibration_list.size(); i += 2){
            auto pos = calibration_list[i];
            auto conf = calibration_list[i+1];
            std::cout << pos.size() << std::endl;
            std::cout << "(" << pos[0] << ", " << pos[1] << ")" << std::endl;
            MotionGenerator motion_generator(percent_max_speed, conf);
            robot.control(motion_generator);
            std::cout << "Press Enter For Next Configuration" << std::endl;
            std::cin.ignore();
        }
    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }
}