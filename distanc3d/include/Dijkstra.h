#pragma once

#include "IPriorityQueue.h"
#include "Helpers.h"
#include "Neighbor.h"

#include <functional>
#include <ostream>
#include <utility>
#include <vector>

#ifdef USE_STD_SPAN
#include <span>
using std::span;
#else
#include <tcb/span.hpp>
using tcb::span;
#endif

namespace distanc3d
{

/// Signed integers are used to represent image indices
using IndexType = int;

/// Priority queue entries for the Dijkstra algorithm are pairs, where:
/// -first element: voxel index
/// -second element: voxel distance
template<typename DistanceType>
using QueueItem = std::pair<IndexType, DistanceType>;

/**
 * @brief Validate that all Dijkstra algorithm inputs are non-null and have the same data size.
 * @param[in] imageDims Image dimensions
 * @param[in] sourceLabel
 * @param[in] imageDistance
 * @param[in] euclideanDistance
 * @param[in] sourceIndex
 * @param[in] parentIndex
 * @return True iff validation passes.
 */
template<typename LabelType, typename DistanceType>
bool validate(
  const glm::ivec3& imageDims,
  const span<LabelType> sourceLabel,
  const span<DistanceType> imageDistance,
  const span<DistanceType> euclideanDistance,
  const span<IndexType> sourceIndex,
  const span<IndexType> parentIndex,
  std::ostream& os
)
{
  if (!sourceLabel.data())
  {
    os << "Source labels are null!" << std::endl;
    return false;
  }

  const std::size_t N = sourceLabel.size();

  if (N != static_cast<std::size_t>(imageDims.x * imageDims.y * imageDims.z))
  {
    os << "Image dimensions do not match data sizes!" << std::endl;
    return false;
  }

  if (!imageDistance.data())
  {
    os << "Image distances are null!" << std::endl;
    return false;
  }
  else if (N != imageDistance.size())
  {
    os << "Source label and image distance data sizes do not match!" << std::endl;
    return false;
  }

  if (!euclideanDistance.data())
  {
    os << "Euclidean distances are null!" << std::endl;
    return false;
  }
  else if (N != euclideanDistance.size())
  {
    os << "Source label and Euclidean distance data sizes do not match!" << std::endl;
    return false;
  }

  if (!sourceIndex.data())
  {
    os << "Source indices are null!" << std::endl;
    return false;
  }
  else if (N != sourceIndex.size())
  {
    os << "Source label and source index data sizes do not match!" << std::endl;
    return false;
  }

  if (!parentIndex.data())
  {
    os << "Parent indices are null!" << std::endl;
    return false;
  }
  else if (N != parentIndex.size())
  {
    os << "Source label and parent index data sizes do not match!" << std::endl;
    return false;
  }

  return true;
}

/**
 * @brief Insert source voxels in a priority queue and initialize their distances to zero.
 *
 * @tparam LabelType Seed image voxel component type
 * @tparam DistaneType Type used to represent distances of entries in the priority queue and
 * also the component type of voxels in the distance image
 *
 * @param[in] sourceLabel Labeled sources
 * @param[out] distance Distance image: source voxel distance set to zero; all other distances set to
 * \c std::numeric_limits<DistanceType>::max()
 * @param[out] source Source index image: each source voxel i set to i (itself as source);
 * all other voxels set to -1
 * @param[out] parent Parent index image: each voxel i set to i (itself as parent)
 * @param[out] pqueue Priority queue
 *
 * @return True iff successful initialization
 */
template<typename LabelType, typename DistanceType>
bool initialize(
  const span<LabelType> sourceLabel,
  span<DistanceType> imageDistance,
  span<DistanceType> euclideanDistance,
  span<IndexType> sourceIndex,
  span<IndexType> parentIndex,
  IPriorityQueue<QueueItem<DistanceType>>& pqueue
)
{
  static constexpr DistanceType MAX_DIST = std::numeric_limits<DistanceType>::max();

  pqueue.clear();

  for (std::size_t i = 0; i < sourceLabel.size(); ++i)
  {
    if (sourceLabel[i] > 0)
    {
      imageDistance[i] = 0;
      euclideanDistance[i] = 0;
      sourceIndex[i] = static_cast<IndexType>(i);
      pqueue.push(std::make_pair(static_cast<IndexType>(i), 0));
    }
    else
    {
      imageDistance[i] = MAX_DIST;
      euclideanDistance[i] = MAX_DIST;
      sourceIndex[i] = -1;
    }

    if (parentIndex.data())
    {
      parentIndex[i] = static_cast<IndexType>(i);
    }
  }

  return true;
}

/**
 * @brief Execute Dijkstra's shortest path algorithm on a 3D image to completion.
 * Graph nodes correspond to voxel centers of the 3D image and graph edges connect
 * neighboring voxels together.
 *
 * @tparam DistanceType Type used to represent distances of priority queue entries and
 * also the component type of voxels in the distance image.
 *
 * @param[in] imageDims Image dimensions
 * @param[in] voxelSpacing Voxel spacing (mm)
 * @param[in, out] sourceIndex
 * Input: The value of each source (seed) voxel i is its own index i. All non-source voxels
 * have negative index.
 * Output: Each voxel is assigned the index of its nearest source, thereby creating a
 * segmentation of the image.
 * @param[in, out] parentIndex Parent index image
 * @param[in, out] imageDistance Distances based on image voxel values
 * @param[in, out] euclidDistance Distances based on Euclidean distance from sources
 * @param[in] euclideanWeight Weight of the Euclidean distance
 * @param[in, out] pqueue Priority queue
 * @param[in] computeImageDistance Function to compute distances between two voxels of an image
 * (optional: ignored if set to nullptr)
 * @param[in] isValidVoxel Function to determine whether v is a valid voxel node in the graph
 * @param[in] debugPrintInterval Number of iterations between successive debug print statements
 * (optional: ignored if set to zero)
 *
 * @return Number of priority queue updates
 */
template<typename DistanceType>
std::size_t dijkstra(
  const glm::ivec3& imageDims,
  const glm::dvec3& voxelSpacing,
  span<IndexType> sourceIndex,
  span<IndexType> parentIndex,
  span<DistanceType> imageDistance,
  span<DistanceType> euclidDistance,
  DistanceType euclidWeight,
  IPriorityQueue<QueueItem<DistanceType>>& pqueue,
  std::function<DistanceType(IndexType u, IndexType v)> computeImageDistance,
  std::function<bool (IndexType v)> isValidVoxel,
  std::size_t debugPrintInterval,
  std::ostream& os
)
{
  static constexpr DistanceType maxDist = std::numeric_limits<DistanceType>::max();

  const Neighborhood neighborhood = makeNeighborhood(imageDims, voxelSpacing, 1);

  std::size_t numIterations = 0;
  std::size_t numQueueUpdates = 0;

  if (debugPrintInterval)
  {
    os << "Iteration, queue size, queue updates" << std::endl;
  }

  const glm::ivec3 zero{0};

  while (!pqueue.empty())
  {
    if (debugPrintInterval && (numIterations % debugPrintInterval == 0))
    {
      os << numIterations << ", " << pqueue.size() << ", " << numQueueUpdates << std::endl;
    }
    ++numIterations;

    const IndexType u = pqueue.top().first;
    pqueue.pop();

    const glm::ivec3 uCoord = indexToCoord(u, imageDims);
    const glm::ivec3 sourceCoord = indexToCoord(sourceIndex[SIZET(u)], imageDims);
    const glm::dvec3 sourcePos = voxelSpacing * glm::dvec3{sourceCoord};

    for (const Neighbor& neighbor : neighborhood.neighbors)
    {
      const glm::ivec3 vCoord = uCoord + neighbor.direction;

      if (glm::any(glm::lessThan(vCoord, zero)) || glm::any(glm::greaterThanEqual(vCoord, imageDims)))
      {
        continue;
      }

      const glm::dvec3 vPos = voxelSpacing * glm::dvec3{vCoord};
      const IndexType v = u + neighbor.offset;

      if (isValidVoxel && !isValidVoxel(v))
      {
        continue;
      }

      const DistanceType uImageDist_old = computeImageDistance ? imageDistance[SIZET(u)] : 0;
      const DistanceType vImageDist_old = computeImageDistance ? imageDistance[SIZET(v)] : 0;
      const DistanceType uvImageDist = computeImageDistance ? computeImageDistance(u, v) : 0;

      const DistanceType vImageDist_new = uImageDist_old + uvImageDist;
      const DistanceType vEuclideanDist_new = static_cast<DistanceType>(glm::distance(sourcePos, vPos));
      const DistanceType vTotalDist_new = vImageDist_new + euclidWeight * vEuclideanDist_new;
      const DistanceType vTotalDist_old = vImageDist_old + euclidWeight * euclidDistance[SIZET(v)];

      if (vTotalDist_new < vTotalDist_old)
      {
        sourceIndex[SIZET(v)] = sourceIndex[SIZET(u)];
        euclidDistance[SIZET(v)] = vEuclideanDist_new;

        if (computeImageDistance)
        {
          imageDistance[SIZET(v)] = vImageDist_new;
        }

        if (parentIndex.data())
        {
          parentIndex[SIZET(v)] = u;
        }

        const bool enqueued = (euclidDistance[SIZET(v)] != maxDist);
        if (pqueue.mutableKeys() && !enqueued)
        {
          pqueue.decreaseKey(std::make_pair(SIZET(v), vTotalDist_new));
        }
        else
        {
          pqueue.push(std::make_pair(SIZET(v), vTotalDist_new));
        }
        ++numQueueUpdates;
      }
    }
  }

  return numQueueUpdates;
}

/**
 * @brief createSegmentation
 * @param source
 * @param seed
 * @param seg
 * @return
 */
template<typename LabelType>
bool createSegmentation(
  const span<IndexType> sourceIndex,
  const span<LabelType> sourceLabel,
  span<LabelType> segLabel
)
{
  if (!sourceIndex.data() || !sourceLabel.data() || !segLabel.data())
  {
    return false;
  }

  if (sourceIndex.size() != sourceLabel.size() || sourceIndex.size() != segLabel.size())
  {
    return false;
  }

  for (std::size_t i = 0; i < sourceIndex.size(); ++i)
  {
    segLabel[i] = sourceLabel[SIZET(sourceIndex[i])];
  }

  return true;
}

/**
 * @brief Compute the shortest path from a voxel to its closest source voxel
 *
 * @tparam LabelType Seed image voxel component type
 *
 * @param[in] parentIndex Parent index image (computed from \c dijkstra)
 * @param[in] destIndex Index of destination voxel of path
 * @param[out] pathImage Binary path image, with path voxels set to 1 (optional: ignored if nullptr)
 *
 * @return Vector of voxel indices along the path, beginning at the destination voxel
 * and ending at the closest source voxel.
 */
template<typename LabelType>
std::vector<IndexType> shortestPath(
  const span<IndexType> parentIndex, IndexType destIndex, span<LabelType> pathImage
)
{
  std::vector<IndexType> path;

  if (!parentIndex.data())
  {
    return path;
  }

  IndexType i = destIndex;
  path.push_back(i);

  if (pathImage.data())
  {
    for (LabelType& l : pathImage)
    {
      l = 0;
    }

    pathImage[SIZET(i)] = 1;
  }

  while (i != parentIndex[SIZET(i)])
  {
    i = parentIndex[SIZET(i)];
    path.push_back(i);

    if (pathImage.data())
    {
      pathImage[SIZET(i)] = 1;
    }
  }

  return path;
}

} // namespace distanc3d
