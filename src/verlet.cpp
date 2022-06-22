#include "../inc/verlet.h" // TODO: write smarter paths

#include <stdexcept> // runtime_error

void naive_verlet_step1(double &x, double &y, double &z, double &vx, double &vy,
                        double &vz, double fx, double fy, double fz,
                        double timestep, double mass) {
  if (!mass)
    throw std::runtime_error(
        "<naive_verlet_step1> mass=0 leads illegal division by 0.");
  vx += .5 * fx * timestep / mass;
  vy += .5 * fy * timestep / mass;
  vz += .5 * fz * timestep / mass;
  x += vx * timestep;
  y += vy * timestep;
  z += vz * timestep;
}

void naive_verlet_step2(double &vx, double &vy, double &vz, double fx,
                        double fy, double fz, double timestep, double mass) {
  if (!mass)
    throw std::runtime_error(
        "<naive_verlet_step2> mass=0 leads illegal division by 0.");
  vx += .5 * fx * timestep / mass;
  vy += .5 * fy * timestep / mass;
  vz += .5 * fz * timestep / mass;
}

void eigen_verlet_step1(Eigen::Array3Xd &positions, Eigen::Array3Xd &velocities,
                        const Eigen::Array3Xd &forces, double timestep,
                        double mass) {
  if (!mass)
    throw std::runtime_error(
        "<eigen_verlet_step1> mass=0 leads illegal division by 0.");
  velocities += .5 * forces * timestep / mass;
  positions += velocities * timestep;
}

void eigen_verlet_step2(Eigen::Array3Xd &velocities,
                        const Eigen::Array3Xd &forces, double timestep,
                        double mass) {
  if (!mass)
    throw std::runtime_error(
        "<eigen_verlet_step2> mass=0 leads illegal division by 0.");
  velocities += .5 * forces * timestep / mass;
}
