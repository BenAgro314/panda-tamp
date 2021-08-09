#include <iostream>
#include <array>

#include <franka/exception.h>
#include <franka/robot.h>

#include "position_controller.h"
#include "parse_confs.h"

int main(int argc, char** argv){
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname> <homing> <object-width>" << std::endl;
        return -1;
    }
    auto confs_list = parseConfsFile("./confs.txt");
    //PositionController position_controller = PositionController(0.2);
    //position_controller.setGoal({{0, 0, 0, 0, 0, 0, 0}});

    try {
        franka::Robot robot(argv[1]);
        franka::Gripper gripper(argv[1]);

        double grasping_width = std::stod(argv[3]);

        std::stringstream ss(argv[2]);
        bool homing;
        if (!(ss >> homing)) {
            std::cerr << "<homing> can be 0 or 1." << std::endl;
            return -1;
        }
        franka::GripperState gripper_state = gripper.readOnce();
        if (gripper_state.max_width < grasping_width) {
            std::cout << "Object is too large for the current fingers on the gripper." << std::endl;
            return -1;
        }

        setDefaultBehavior(robot);

        std::array<double, 7> q_goal = {{0.0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);

        std::cout << "WARNING: This example will move the robot! "
                << "Please make sure to have the user stop button at hand!" << std::endl
                << "Press Enter to continue..." << std::endl;
        std::cin.ignore();

        robot.control(motion_generator);
        std::cout << "Finished moving to initial joint configuration." << std::endl;

        if (homing){
            std::cout << "Starting homing to estimate finger width" << std::endl;
            gripper.homing();
        }

        if (gripper_state.max_width < grasping_width) {
            std::cout << "Object is too large for the current fingers on the gripper." << std::endl;
            return -1;
        }

        int j = 0;
        for (auto conf: confs_list){
            j += 1;
            std::cout << "Starting motion #" << j << std::endl;
            if (conf == GRASP_SIG) {
                // Grasp the object.
                if (!gripper.grasp(grasping_width, 0.1, 60)) {
                    std::cout << "Failed to grasp object." << std::endl;
                    return -1;
                }
            } else if (conf == RELEASE_SIG) {
                if (!gripper.move(0.08, 0.1)) {
                    std::cout << "Failed to open gripper" << std::endl;
                    return -1;
                }
            } else {
                MotionGenerator motion_generator(0.5, conf);
                robot.control(motion_generator);
            }
            std::cout << "Ending motion #" << j << std::endl;
        }

    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}