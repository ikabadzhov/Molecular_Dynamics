#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Dense>
#include <string>
#include <vector>

using Positions_t = Eigen::Array3Xd;
using Velocities_t = Eigen::Array3Xd;
using Forces_t = Eigen::Array3Xd;
using Names_t = std::vector<std::string>;

#endif /* TYPES_H */
