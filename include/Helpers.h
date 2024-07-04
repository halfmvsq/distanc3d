#pragma once

#include <cstdlib>
#include <glm/glm.hpp>

namespace distanc3d
{

/**
 * @brief Convert 3D image voxel coordinates to a linear voxel index
 * @param[in] coord 3D voxel coordinates
 * @param[in] imageDims Image dimensions
 * @return Linear voxel index
 */
int coordToIndex(const glm::ivec3& coord, const glm::ivec3& imageDims)
{
  return (coord.z * imageDims.y + coord.y) * imageDims.x + coord.x;
}

/**
 * @brief Convert 3D coordinates in a cube of size [-(2*cubeRadius+1), 2*cubeRadius+1]^3
 * to a linear index into the cube
 * @param[in] coord 3D coordinates in the cube
 * @param[in] cubeRadius
 * @return Linear index into the cube
 */
int cubeIndex(const glm::ivec3& coord, int cubeRadius)
{
  const glm::ivec3 offset{cubeRadius};
  const glm::ivec3 cubeDims{2 * cubeRadius + 1};
  return coordToIndex(coord + offset, cubeDims);
}

/**
 * @brief Convert a linear index in a 3D image to 3D voxel coordinates
 * @param[in] index Linear index
 * @param[in] imageDims Image dimensions
 * @return 3D voxel coordinates
 */
glm::ivec3 indexToCoord(int index, const glm::ivec3& imageDims)
{
  const std::div_t zQuotRem = std::div(index, imageDims.x * imageDims.y);
  const std::div_t xyQuotRem = std::div(zQuotRem.rem, imageDims.x);
  return glm::ivec3{xyQuotRem.rem, xyQuotRem.quot, zQuotRem.quot};
}

} // namespace distanc3d
