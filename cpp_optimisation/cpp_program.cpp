#include <iostream>
#include <string>
#include <vector>
#include "json.hpp"
#include "optimise.h"


using json = nlohmann::json;

int main() {
    // Read input JSON from standard input
    std::string input_json;
    std::getline(std::cin, input_json);

    // Parse input JSON
    json input_data = json::parse(input_json);

    // Access input list and matrix
    std::vector<int> path = input_data["list"];
    std::vector<std::vector<int>> matrix = input_data["matrix"];
    int type_of_opti = input_data["type"];

//   Example of path and matrix and type of opti:
//    std::vector<int> path = {0, 1, 2, 3, 4};
//    std::vector<std::vector<int>> matrix = {{0, 10, 15, 20, 25},
//                                            {10, 0, 35, 40, 45},
//                                            {15, 35, 0, 55, 60},
//                                            {20, 40, 55, 0, 75},
//                                            {25, 45, 60, 75, 0}};
//    int type_of_opti = 2;

    //     Call the optimise function with your variables.
    std::vector<int> output_list  = optimise(path, matrix, type_of_opti);

    // Prepare output data
    json output_data = {{"output_list", output_list}};

    // Serialize output data to JSON and write to standard output
    std::cout << output_data.dump() << std::endl;

    return 0;
}