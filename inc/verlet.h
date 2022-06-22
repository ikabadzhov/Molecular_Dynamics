#ifndef __VERLET_H
#define __VERLET_H

#include <Eigen/Dense>

/// \brief Naive Verlet integration step 1
/// \param[in] <x, y, z> position
/// \param[in] <vx, vy, vz> velocity
/// \param[in] <fx, fy, fz> forces
/// \param[in] timestamp
/// \param[in] mass
void naive_verlet_step1(double &x, double &y, double &z, double &vx, double &vy,
                        double &vz, double fx, double fy, double fz,
                        double timestep, double mass);
void naive_verlet_step2(double &vx, double &vy, double &vz, double fx,
                        double fy, double fz, double timestep, double mass);

/// \brief Eigen Verlet integration step 1
/// \param[in] Eigen::Array3Xd <x, y, z> position
/// \param[in] Eigen::Array3Xd <vx, vy, vz> velocity
/// \param[in] Eigen::Array3Xd <fx, fy, fz> forces
/// \param[in] timestamp
/// \param[in] mass
void eigen_verlet_step1(Eigen::Array3Xd &positions, Eigen::Array3Xd &velocities,
                        const Eigen::Array3Xd &forces, double timestep,
                        double mass);
void eigen_verlet_step2(Eigen::Array3Xd &velocities,
                        const Eigen::Array3Xd &forces, double timestep,
                        double mass);

#endif // __VERLET_H