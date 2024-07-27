#ifndef UNTITLED_OPTMISE_H
#define UNTITLED_OPTMISE_H

#include <string>
#include <vector>

using namespace std;

std::vector<int> optimise(std::vector<int>& path, std::vector<std::vector<int>>& matrix, std::vector<std::pair<double,
 double>>& points, int type_of_optimisation, int turning_radius);

#endif //UNTITLED_OPTMISE_H
