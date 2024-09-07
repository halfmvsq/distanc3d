#pragma once

#include <itkImage.h>
#include <itkImageFileWriter.h>
#include <itkImportImageFilter.h>

#include <filesystem>
#include <iostream>

/**
 * @brief Create a 3D ITK image from raw image data
 * @tparam T Image component type
 * @param[in] refImage Reference image
 * @param[in] imageData Raw image data
 * @return ITK image aligned with refImage with voxel values from imageData
 */
template<typename T>
typename itk::Image<T, 3>::Pointer createImage(
    const typename itk::ImageBase<3>::Pointer refImage, const T* imageData, std::size_t imageSize
)
{
  using ImportFilterType = itk::ImportImageFilter<T, 3>;

  // This filter will not free the memory in its destructor and the application providing the
  // buffer retains the responsibility of freeing the memory for this image data
  constexpr bool filterOwnsBuffer = false;

  if (!refImage)
  {
    std::cerr << "Null data array provided when creating new scalar image!" << std::endl;
    return nullptr;
  }

  if (!imageData)
  {
    return nullptr;
  }

  if (imageSize != refImage->GetLargestPossibleRegion().GetNumberOfPixels())
  {
    return nullptr;
  }

  typename ImportFilterType::Pointer importer = ImportFilterType::New();
  importer->SetRegion(refImage->GetLargestPossibleRegion());
  importer->SetOrigin(refImage->GetOrigin());
  importer->SetSpacing(refImage->GetSpacing());
  importer->SetDirection(refImage->GetDirection());
  importer->SetImportPointer(const_cast<T*>(imageData), imageSize, filterOwnsBuffer);

  try
  {
    importer->Update();
    return importer->GetOutput();
  }
  catch (const itk::ExceptionObject& e)
  {
    std::cerr << "Exception creating ITK scalar image from data array: " << e.what() << std::endl;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception creating ITK scalar image from data array: " << e.what() << std::endl;
  }
  return nullptr;
}

/**
 * @brief Write an ITK image to disk
 * @param[in] image ITK image
 * @param[in] outFilename File name
 */
template<class ImageType>
void writeImage(const typename ImageType::Pointer image, const std::filesystem::path& outFilename)
{
  using WriterType = itk::ImageFileWriter<ImageType>;
  auto writer = WriterType::New();
  writer->SetFileName(outFilename);
  writer->SetInput(image);

  try
  {
    writer->Update();
  }
  catch (const itk::ExceptionObject& e)
  {
    std::cerr << "Exception writing ITK image: " << e.what() << std::endl;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception writing ITK image: " << e.what() << std::endl;
  }
}
