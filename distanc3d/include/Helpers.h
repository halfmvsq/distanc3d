#pragma once

#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

#include <cstdlib>
#include <iomanip>
#include <ostream>
#include <vector>

#ifdef USE_STD_SPAN
#include <span>
using std::span;
#else
#include <tcb/span.hpp>
using tcb::span;
#endif

#define SIZET(i) (static_cast<std::size_t>((i)))

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

template<typename IndexType, typename DistanceType, typename ImageCompType>
void printShortestPath(
  std::ostream& os,
  const std::vector<IndexType>& path,
  const span<DistanceType> imageDistance,
  const span<DistanceType> euclideanDistance,
  const span<ImageCompType> image,
  const glm::ivec3& imageDims
)
{
  os << std::fixed << std::setprecision(6);
  os << "Step, voxel coordinate, total image distance, total Euclidean distance, image value" << std::endl;

  std::size_t pathCounter = 0;

  for (auto riter = path.rbegin(); riter != path.rend(); ++riter)
  {
    const std::size_t i = SIZET(*riter);
    os << pathCounter << ", "
       << glm::to_string(indexToCoord(*riter, imageDims)) << ", "
       << imageDistance[i] << ", "
       << euclideanDistance[i] << ", "
       << image[i] << std::endl;

    ++pathCounter;
  }
}

} // namespace distanc3d
