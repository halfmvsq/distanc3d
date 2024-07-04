#include "Dijkstra.h"
#include "Util.h"

// Wrapper around C++ std::priority_queue
#include "queues/StdPriorityQueue.h"

// Uncomment these to try other priority queue implementations:
// #include "queues/GPriorityQueue.h"
// #include "queues/IndexedPriorityQueue.h"
// #include "queues/JkdsPriorityQueue.h"
// #include "queues/UpdatablePriorityQueue.h"

#include <glm/glm.hpp>

#include <itkImage.h>
#include <itkImageFileReader.h>

#include <chrono>
#include <iostream>
#include <vector>

namespace
{
/// Type for representing distances and edge weights between image voxels
using DistanceType = float;

/// Less-than comparison on queue items -- used only for the \c GPriorityQueue
struct LessComparer
{
  bool operator()(
    const distanc3d::QueueItem<DistanceType>& a, const distanc3d::QueueItem<DistanceType>& b
  ) const
  {
    return b.second < a.second;
  }
};

} // namespace

int main(int argc, char* argv[])
{
  using std::chrono::duration_cast;
  using std::chrono::milliseconds;
  using std::chrono::steady_clock;
  using std::chrono::time_point;

  constexpr unsigned int Dim = 3;
  using ImageCompType = short;
  using LabelType = unsigned short; // Segmentation and seed image component type
  using IndexType = distanc3d::IndexType;

  using ImageType = itk::Image<ImageCompType, Dim>;
  using LabelImageType = itk::Image<LabelType, Dim>;
  using DistanceImageType = itk::Image<DistanceType, Dim>;

  // Flag to use image values when computing weights between voxels
  constexpr bool useImageForDistance = false;

  // Coefficient that weights the relative contributions of image distance and Euclidean distance
  // when computing the edge distance between two adjacent voxels (nodes):
  // edge weight = imageDistance + euclideanWeight * euclideanDistance
  constexpr DistanceType euclideanWeight = 1.0f;

  // Number of intervals between debug print statements (use 0 to disable debug prints)
  constexpr std::size_t debugPrintInterval = 10000;

  if (argc != 5)
  {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << " <image_in> <source_in> <seg_out> <distance_out>\n"
              << "- image_in: image on which to compute distances (input)\n"
              << "- source_in: source label image (input)\n"
              << "- seg_out: segmentation label image (output)\n"
              << "- distance_out: distance image (output)\n"
              << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "Image (in): " << argv[1] << "\nSource label image (in): " << argv[2]
            << "\nSegmentation label image (out): " << argv[3]
            << "\nDistance image (out): " << argv[4] << std::endl;

  const ImageType::Pointer image = itk::ReadImage<ImageType>(argv[1]);
  LabelImageType::Pointer sourceLabel = itk::ReadImage<LabelImageType>(argv[2]);

  if (!image)
  {
    std::cerr << "Null input image!" << std::endl;
    return EXIT_FAILURE;
  }

  if (!sourceLabel)
  {
    std::cerr << "Null input seed image!" << std::endl;
    return EXIT_FAILURE;
  }

  const ImageType::SizeType imageSize = image->GetLargestPossibleRegion().GetSize();
  const LabelImageType::SizeType size = sourceLabel->GetLargestPossibleRegion().GetSize();

  if (imageSize[0] != size[0] || imageSize[1] != size[1] || imageSize[2] != size[2])
  {
    std::cerr << "Image and source dimensions do not match!" << std::endl;
    return EXIT_FAILURE;
  }

  const glm::ivec3 imageDims(size[0], size[1], size[2]);
  const glm::dvec3 voxelSpacing(
    sourceLabel->GetSpacing()[0], sourceLabel->GetSpacing()[1], sourceLabel->GetSpacing()[2]
  );
  const std::size_t N = imageDims.x * imageDims.y * imageDims.z;

  std::vector<IndexType> sourceIndex(N, -1);         // Source index
  std::vector<IndexType> parentIndex(N, -1);         // Parent index
  std::vector<DistanceType> imageDistance(N, 0);     // Image distance
  std::vector<DistanceType> euclideanDistance(N, 0); // Euclidean distance

  // Distance function on the image -- when null, it is not used
  std::function<DistanceType(IndexType u, IndexType v)> computeImageDistance = nullptr;

  if (useImageForDistance)
  {
    // Define a sample distance function between voxels with indices u and v:
    const ImageCompType* buffer = image->GetBufferPointer();
    computeImageDistance = [buffer](IndexType u, IndexType v) -> DistanceType
    { return static_cast<DistanceType>(std::abs(buffer[u] - buffer[v])); };
  }

  // Choose your favorite priority queue:
  StdPriorityQueue<distanc3d::QueueItem<DistanceType>> pqueue; // PQ from the C++ Standard Library

  // constexpr bool MutableKeys = true;
  // MyIndexedPriorityQueue<distanc3d::QueueItem<DistanceType>, MutableKeys> queue(N);
  // UpdatablePriorityQueue<distanc3d::QueueItem<DistanceType>, MutableKeys> queue;
  // GPriorityQueue<distanc3d::QueueItem<DistanceType>, LessComparer> queue;
  // JkdsPriorityQueue<distanc3d::QueueItem<DistanceType>, MutableKeys> queue;

  if (!distanc3d::validate(
        imageDims,
        std::span{sourceLabel->GetBufferPointer(), N},
        std::span{imageDistance},
        std::span{euclideanDistance},
        std::span{sourceIndex},
        std::span{parentIndex}
      ))
  {
    std::cerr << "Failed to validate data!" << std::endl;
    return EXIT_FAILURE;
  }

  if (!distanc3d::initialize(
        std::span{sourceLabel->GetBufferPointer(), N},
        std::span{imageDistance},
        std::span{euclideanDistance},
        std::span{sourceIndex},
        std::span{parentIndex},
        pqueue
      ))
  {
    std::cerr << "Failed to initialize priority queue!" << std::endl;
    return EXIT_FAILURE;
  }

  time_point<steady_clock> tick = steady_clock::now();

  // Run the Dijkstra algorithm on the image using seed
  const std::size_t updates = distanc3d::dijkstra(
    imageDims,
    voxelSpacing,
    std::span{sourceIndex},
    std::span{parentIndex},
    std::span{imageDistance},
    std::span{euclideanDistance},
    euclideanWeight,
    pqueue,
    computeImageDistance,
    debugPrintInterval
  );

  time_point<steady_clock> tock = steady_clock::now();

  std::cout << "\nTotal execution time (ms): " << duration_cast<milliseconds>(tock - tick).count()
            << "\nTotal queue updates/inserts: " << updates << std::endl;

  // In-place replacement of seed image with segmentation labels:
  if (!distanc3d::createSegmentation(
        sourceIndex,
        std::span{sourceLabel->GetBufferPointer(), N},
        std::span{sourceLabel->GetBufferPointer(), N}
      ))
  {
    std::cerr << "Error creating segmentation!" << std::endl;
    return EXIT_FAILURE;
  }

  std::vector<DistanceType> totalDistance(N);
  for (std::size_t i = 0; i < N; ++i)
  {
    const DistanceType imgDist = useImageForDistance ? imageDistance[i] : 0;
    totalDistance[i] = imgDist + euclideanWeight * euclideanDistance[i];
  }

  const DistanceImageType::Pointer distanceImage = createImage(sourceLabel, std::span{totalDistance});

  writeImage<LabelImageType>(sourceLabel, argv[3]);
  writeImage<DistanceImageType>(distanceImage, argv[4]);

  return EXIT_SUCCESS;
}
