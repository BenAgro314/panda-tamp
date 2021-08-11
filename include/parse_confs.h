#include <iostream>
#include <fstream>
#include <array>
#include <string>
#include <vector>
#include <regex>
#include <assert.h>

#include "position_control_utils.h"
#include <franka/gripper.h>

#define NONE -10000


std::string removeWhitespace(const std::string& cell){
    std::cout << cell << std::endl;
    return std::regex_replace(cell, std::regex("^ +| +$|( ) +"), "$1");
}

std::vector<std::array<double, 7>> parseConfsFile(std::string filepath){

    std::string grasp_action = "grasp";
    std::string release_action = "release";

    std::ifstream file(filepath);
    std::string cell;
    std::string line;

    std::vector<std::array<double, 7>> out;

    if (file.is_open()){
        while (std::getline(file, line)){
            if (line == grasp_action){
                std::cout << "GRASP" << std::endl;
                out.push_back(GRASP_SIG);
                continue;
            }
            if (line == release_action){
                std::cout << "RELEASE" << std::endl;
                out.push_back(RELEASE_SIG);
                continue;
            }
            std::stringstream lineStream(line);
            int i = 0;
            std::array<double, 7> q = {{NONE, NONE, NONE, NONE, NONE, NONE, NONE}};
            while (std::getline(lineStream, cell, ',')){
                double num = std::stod(cell) ;
                //std::cout << num << " ";
                q[i] = num;
                i+=1;
            }
            // it could also be an x,y value used for calibration
            if (i == 7) {
                assert(isConfValid(q));
            }
            out.push_back(q);
            //std::cout << std::endl;
        }
        file.close();
    } else{
        std::cout << "Unable to open file" << std::endl;
    }

    return out;
}

