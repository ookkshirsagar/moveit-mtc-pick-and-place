#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "ur5e_rh56dfx_manipulation/config/placement.h"

using ur5e_rh56dfx_manipulation::cubeCentreZ;
using ur5e_rh56dfx_manipulation::Region;
using ur5e_rh56dfx_manipulation::regionsSeparated;
using ur5e_rh56dfx_manipulation::samplePosition;

// values from config/pick_place.yaml
const Region kCubeRegion{-0.35, -0.10, 0.05, 0.35};
const Region kPlaceRegion{-0.32, -0.10, -0.22, -0.06};
constexpr double kTable = 0.855, kCube = 0.075, kReach = 0.5;

TEST(Placement, CubeCentreHeights)
{
  EXPECT_NEAR(cubeCentreZ(kTable, kCube), 0.8925, 1e-9);
  EXPECT_NEAR(cubeCentreZ(kTable, kCube, 2) - cubeCentreZ(kTable, kCube, 1), kCube, 1e-9);
}

TEST(Placement, SamplesStayInRegionAndReach)
{
  for (unsigned seed = 1; seed <= 500; ++seed) {
    auto p = samplePosition(kCubeRegion, seed, kReach);
    EXPECT_TRUE(kCubeRegion.contains(p.x, p.y)) << "seed " << seed;
    EXPECT_LE(std::hypot(p.x, p.y), kReach) << "seed " << seed;
  }
}

TEST(Placement, SeedIsReproducible)
{
  for (unsigned seed : {1u, 7u, 42u}) {
    auto a = samplePosition(kPlaceRegion, seed), b = samplePosition(kPlaceRegion, seed);
    EXPECT_EQ(a.x, b.x);
    EXPECT_EQ(a.y, b.y);
  }
  auto a = samplePosition(kPlaceRegion, 1), b = samplePosition(kPlaceRegion, 2);
  EXPECT_TRUE(a.x != b.x || a.y != b.y);
}

TEST(Placement, RandomPlaceNeverHitsRandomCube)
{
  EXPECT_TRUE(regionsSeparated(kCubeRegion, kPlaceRegion, kCube));
  EXPECT_FALSE(regionsSeparated(kCubeRegion, kCubeRegion, kCube));
}

TEST(Placement, UnreachableRegionThrows)
{
  EXPECT_THROW(samplePosition({2.0, 3.0, 2.0, 3.0}, 1, kReach), std::runtime_error);
}

TEST(Placement, RegionFromVectorValidates)
{
  auto r = Region::fromVector({-0.3, -0.1, 0.0, 0.2});
  EXPECT_DOUBLE_EQ(r.x_min, -0.3);
  EXPECT_DOUBLE_EQ(r.y_max, 0.2);
  EXPECT_THROW(Region::fromVector({0.1, -0.1, 0.0, 0.2}), std::invalid_argument);
  EXPECT_THROW(Region::fromVector({0.0, 0.1}), std::invalid_argument);
}
