#include "inc/atoms.h"
#include <gtest/gtest.h>

TEST(AtomsTest, Constructors) {
  auto nb_Atoms = 100u;

  Positions_t pos = Eigen::Array3Xd(3, nb_Atoms);
  pos.setRandom();
  Atoms atoms0(pos);

  Positions_t vel = Eigen::Array3Xd(3, nb_Atoms);
  vel.setRandom();
  Atoms atoms1(pos, vel);

  EXPECT_EQ(atoms0.nb_atoms(), nb_Atoms);
  EXPECT_EQ(atoms1.nb_atoms(), nb_Atoms);

  for (auto i{0u}; i < nb_Atoms; ++i) {
    for (auto j{0u}; j < 3; ++j) {
      EXPECT_DOUBLE_EQ(atoms0.positions(j, i), pos(j, i));
      EXPECT_DOUBLE_EQ(atoms0.velocities(j, i), 0.);
      EXPECT_DOUBLE_EQ(atoms0.forces(j, i), 0.);

      EXPECT_DOUBLE_EQ(atoms1.positions(j, i), pos(j, i));
      EXPECT_DOUBLE_EQ(atoms1.velocities(j, i), vel(j, i));
      EXPECT_DOUBLE_EQ(atoms1.forces(j, i), 0.);
    }
  }
}