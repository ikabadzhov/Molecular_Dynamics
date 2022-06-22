#include "inc/verlet.h"
#include <gtest/gtest.h>
#include <vector>

TEST(NaiveVerlet, AllZeros) {
  double x{0.}, y{0.}, z{0.};
  double vx{0.}, vy{0.}, vz{0.};
  double fx{0.}, fy{0.}, fz{0.};
  double t{0.}, m{0.};

  EXPECT_THROW(
      try {
        naive_verlet_step1(x, y, z, vx, vy, vz, fx, fy, fz, t, m);
      } catch (const std::runtime_error &err) {
        EXPECT_EQ(std::string(err.what()),
                  "<naive_verlet_step1> mass=0 leads illegal division by 0.");
        throw;
      },
      std::runtime_error);

  EXPECT_THROW(
      try {
        naive_verlet_step2(vx, vy, vz, fx, fy, fz, t, m);
      } catch (const std::runtime_error &err) {
        EXPECT_EQ(std::string(err.what()),
                  "<naive_verlet_step2> mass=0 leads illegal division by 0.");
        throw;
      },
      std::runtime_error);

  // ensure none of the values has changed
  EXPECT_DOUBLE_EQ(x, 0.);
  EXPECT_DOUBLE_EQ(y, 0.);
  EXPECT_DOUBLE_EQ(z, 0.);
  EXPECT_DOUBLE_EQ(vx, 0.);
  EXPECT_DOUBLE_EQ(vy, 0.);
  EXPECT_DOUBLE_EQ(vz, 0.);
  EXPECT_DOUBLE_EQ(fx, 0.);
  EXPECT_DOUBLE_EQ(fy, 0.);
  EXPECT_DOUBLE_EQ(fz, 0.);
  EXPECT_DOUBLE_EQ(t, 0.);
  EXPECT_DOUBLE_EQ(m, 0.);
}

TEST(NaiveVerlet, AllOnes) {
  double x{1.}, y{1.}, z{1.};
  double vx{1.}, vy{1.}, vz{1.};
  double fx{1.}, fy{1.}, fz{1.};
  double t{1.}, m{1.};

  naive_verlet_step1(x, y, z, vx, vy, vz, fx, fy, fz, t, m);

  // check only variables passed by reference
  EXPECT_DOUBLE_EQ(x, 2.5);
  EXPECT_DOUBLE_EQ(y, 2.5);
  EXPECT_DOUBLE_EQ(z, 2.5);
  EXPECT_DOUBLE_EQ(vx, 1.5);
  EXPECT_DOUBLE_EQ(vy, 1.5);
  EXPECT_DOUBLE_EQ(vz, 1.5);

  std::vector<double> A(11, 1.);
  naive_verlet_step1(A[0], A[1], A[2], A[3], A[4], A[5], A[6], A[7], A[8], A[9],
                     A[10]);
  EXPECT_DOUBLE_EQ(A[0], x);
  EXPECT_DOUBLE_EQ(A[1], y);
  EXPECT_DOUBLE_EQ(A[2], z);
  EXPECT_DOUBLE_EQ(A[3], vx);
  EXPECT_DOUBLE_EQ(A[4], vy);
  EXPECT_DOUBLE_EQ(A[5], vz);

  naive_verlet_step2(vx, vy, vz, fx, fy, fz, t, m);
  EXPECT_DOUBLE_EQ(vx, 2.);
  EXPECT_DOUBLE_EQ(vy, 2.);
  EXPECT_DOUBLE_EQ(vz, 2.);

  naive_verlet_step2(A[3], A[4], A[5], A[6], A[7], A[8], A[9], A[10]);
  EXPECT_DOUBLE_EQ(A[3], vx);
  EXPECT_DOUBLE_EQ(A[4], vy);
  EXPECT_DOUBLE_EQ(A[5], vz);
}

TEST(EigenVerlet, SingleVectorAllOnes) {
  // reference values
  double x{1.}, y{1.}, z{1.};
  double vx{1.}, vy{1.}, vz{1.};
  double fx{1.}, fy{1.}, fz{1.};
  double t{1.}, m{1.};
  naive_verlet_step1(x, y, z, vx, vy, vz, fx, fy, fz, t, m);

  size_t n_vectors = 1;
  Eigen::Array3Xd positions(3, n_vectors);
  Eigen::Array3Xd velocities(3, n_vectors);
  Eigen::Array3Xd forces(3, n_vectors);

  for (auto i = 0u; i < 3u; ++i) {
    positions(i, 0) = 1;
    velocities(i, 0) = 1;
    forces(i, 0) = 1;
  }

  eigen_verlet_step1(positions, velocities, forces, t, m);
  EXPECT_DOUBLE_EQ(positions(0, 0), x);
  EXPECT_DOUBLE_EQ(positions(1, 0), y);
  EXPECT_DOUBLE_EQ(positions(2, 0), z);
  EXPECT_DOUBLE_EQ(velocities(0, 0), vx);
  EXPECT_DOUBLE_EQ(velocities(1, 0), vy);
  EXPECT_DOUBLE_EQ(velocities(2, 0), vz);

  naive_verlet_step2(vx, vy, vz, fx, fy, fz, t, m);
  eigen_verlet_step2(velocities, forces, t, m);

  EXPECT_DOUBLE_EQ(positions(0, 0), x);
  EXPECT_DOUBLE_EQ(positions(1, 0), y);
  EXPECT_DOUBLE_EQ(positions(2, 0), z);
}

TEST(EigenVerlet, ManyRandomVectors) {
  double t{1.}, m{1.};
  size_t n_vectors = 1000;
  Eigen::Array3Xd positions = Eigen::Array3Xd::Random(3, n_vectors);
  Eigen::Array3Xd velocities = Eigen::Array3Xd::Random(3, n_vectors);
  Eigen::Array3Xd forces = Eigen::Array3Xd::Random(3, n_vectors);

  std::vector<std::vector<double>> entries(n_vectors, std::vector<double>(9));
  for (size_t i = 0u; i < 3; ++i) {
    for (size_t j = 0u; n_vectors < 3; ++j) {
      entries[i][j] = positions(i, j);
      entries[i + 3][j] = velocities(i, j);
      entries[i + 6][j] = forces(i, j);
    }
    naive_verlet_step1(entries[i][0], entries[i][1], entries[i][2],
                       entries[i][3], entries[i][4], entries[i][5],
                       entries[i][6], entries[i][7], entries[i][8], t, m);
  }
  eigen_verlet_step1(positions, velocities, forces, t, m);

  for (size_t i = 0u; i < 3; ++i) {
    for (size_t j = 0u; n_vectors < 3; ++j) {
      EXPECT_DOUBLE_EQ(entries[i][j], positions(i, j));
      EXPECT_DOUBLE_EQ(entries[i + 3][j], velocities(i, j));
    }
    naive_verlet_step2(entries[i][3], entries[i][4], entries[i][5],
                       entries[i][6], entries[i][7], entries[i][8], t, m);
  }

  eigen_verlet_step2(velocities, forces, t, m);

  for (size_t i = 0u; i < 3; ++i)
    for (size_t j = 0u; n_vectors < 3; ++j)
      EXPECT_DOUBLE_EQ(entries[i + 3][j], velocities(i, j));
}
