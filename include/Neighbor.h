#pragma once

#include "Helpers.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/gtx/matrix_operation.hpp>
#include <glm/gtx/string_cast.hpp>

#include <iostream>
#include <vector>

namespace distanc3d
{

/**
 * @brief A neighbor in a 3D cubic neighborhood of any radius.
 *
 * Given a voxel u = (x, y, z), this struct specifies a neighbor v = (x', y', z') of u within
 * a full cubic neighborhood of radius R, where the cube side length is 2*R + 1.
 *
 * Each point in a regular 3D lattice (i.e. voxels in a 3D medical image) has a neighborhood of
 * points (voxels) surrounding it. For example, with a 3x3x3 cube neighborhood (radius 1),
 * there are 27 different configrations of voxel neighborhoods:
 * - 1 configuration for voxels in the interior of the lattice. These voxels have 26 neighbors.
 * - 6 configurations for voxels on the faces of the lattice. These voxels have 17 neighbors.
 * - 12 configurations for voxels on the edges of the lattice. These voxels have 11 neighbors.
 * - 8 configurations for voxels on the corners of the lattice. These voxels have 7 neighbors.
 */
struct Neighbor
{
  /// Direction vector of v relative to u, which is
  /// direction = v - u = (x' - x, y' - y, z' - z),
  /// where the direction vector components are in {-R, ..., R} and R is the cube radius.
  glm::ivec3 direction;

  /// The linear (signed) direction offset relative of this neighbor relative to the cube center.
  /// The offset is a function of the image dimensions.
  int offset;

  /// Euclidean distance (in mm) between u and v, accounting for voxel spacing.
  double distance;
};

/**
 * @brief A cubic neighborhood/stencil, where the cube length is 2*radius + 1, so that:
 * -radius 1: 3x3x3 cube
 * -radius 2: 5x5x5 cube
 */
struct Neighborhood
{
  int radius;
  std::vector<Neighbor> neighbors;
};

std::ostream& operator<<(std::ostream& os, const Neighborhood& N)
{
  for (const Neighbor& neighbor : N.neighbors)
  {
    os << "\n-Direction offset: " << glm::to_string(neighbor.direction)
       << "\n-Linear offset: " << neighbor.offset << "\n-Distance: " << neighbor.distance
       << std::endl;
  }
  return os;
}

/**
 * @brief Make a cubic neighborhood for an image
 * @param[in] imageDims Image dimensions
 * @param[in] voxelSize Voxel size
 * @param[in] radius Neighborhood cube radius, such that the neighborhood cube has side length
 * 2*radius + 1.
 * @return Cubic neighborhood
 */
Neighborhood makeNeighborhood(const glm::ivec3& imageDims, const glm::dvec3& voxelSize, int radius)
{
  const int cubeLength = 2 * radius + 1;
  const int numNeighbors = std::pow(cubeLength, 3);

  /// @note There are 27 neighborhood configurations, each consisting
  /// of up to 26 neighbors. For simplicity, create each neighborhood
  /// with 27 neighbors, but assign invalid flag to invalid neighbors.
  Neighborhood neighborhood;
  neighborhood.radius = radius;
  neighborhood.neighbors.resize(numNeighbors);

  glm::ivec3 neighborDir;
  for (neighborDir.z = -radius; neighborDir.z <= radius; ++neighborDir.z)
  {
    for (neighborDir.y = -radius; neighborDir.y <= radius; ++neighborDir.y)
    {
      for (neighborDir.x = -radius; neighborDir.x <= radius; ++neighborDir.x)
      {
        const int neighborConfig = cubeIndex(neighborDir, radius);

        Neighbor& neighbor = neighborhood.neighbors[neighborConfig];
        neighbor.direction = neighborDir;
        neighbor.offset = coordToIndex(neighborDir, imageDims);
        neighbor.distance = glm::length(glm::diagonal3x3(voxelSize) * neighborDir);
      }
    }
  }
  return neighborhood;
}

} // namespace distanc3d
