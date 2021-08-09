#include "parse_confs.h"

int main(int argc, char** argv){
    //std::string s = " a bcde ";
    //s = removeWhitespace("   123.0   ");
    //std::cout << s <<std::endl;
    auto out = parseConfsFile("./confs.txt");
    for (auto q: out) {
        printConf(q);
    }
    return 0;
}