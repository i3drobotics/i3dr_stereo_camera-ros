#include <stereoMatcher/jrsgm.h>

//Initialise matcher
void jrsgm::init(std::string &sConfigFile)
{
  std::cout << sConfigFile << std::endl;
  JR::Phobos::ReadIniFile(params, sConfigFile);
  createMatcher();
}

int jrsgm::getErrorDisparity(void)
{
  return -10000;
}

int jrsgm::round_up_to_32(int val){
  return round(val / 32) * 32;
}

int jrsgm::checkMemoryCensus(int image_width, int image_height){
  int num_of_gpus = params.oGPUs.size();
  int window_size_x = params.oPyramidParams[0].oMetricParams.nWindowSizeX;
  int window_size_y = params.oPyramidParams[0].oMetricParams.nWindowSizeY;
  int num_of_slanted_window_scales = params.oPyramidParams[0].oMetricParams.nNumberOfScales;
  int num_of_slanted_window_shifts = params.oPyramidParams[0].oMetricParams.nNumberOfSlants;
  int have_no_data = params.bHasNodata;

  bool enMeanCensus = true;
  int NumberOfNeededCensusBytes;
  if (enMeanCensus){
    NumberOfNeededCensusBytes = round_up_to_32(window_size_x*window_size_y * (1+have_no_data))/4 * (num_of_slanted_window_scales + num_of_slanted_window_shifts*2);
  } else {
    NumberOfNeededCensusBytes = round_up_to_32((window_size_x*window_size_y -1)* (1+have_no_data))/4 * (num_of_slanted_window_scales + num_of_slanted_window_shifts*2);
  }
  int MemoryCensus_PerGPU = 2*image_width*image_height*NumberOfNeededCensusBytes / num_of_gpus;
  return MemoryCensus_PerGPU;
}

int jrsgm::checkMemoryDSI(int image_width, int image_height){
  int num_of_disparities = params.oPyramidParams[0].nMaximumNumberOfDisparities;
  bool enTextureDSI = params.oPyramidParams[0].oSGMParams.bUseDSITexture;
  int texture_dsi;
  if (enTextureDSI){
    texture_dsi = 2;
  } else {
    texture_dsi = 1;
  }
  int MemoryDSI_PerGPU = 2*image_width*image_height*num_of_disparities;
  return MemoryDSI_PerGPU;
}

//compute disparity
void jrsgm::compute(cv::Mat left_image, cv::Mat right_image, cv::Mat &disp)
{
  int memoryDSI = checkMemoryDSI(left_image.size().width,left_image.size().height);
  int memoryCensus = checkMemoryCensus(left_image.size().width,left_image.size().height);
  std::cout << "GPU MEMORY DSI: " << memoryDSI << std::endl;
  std::cout << "GPU MEMORY CENSUS: " << memoryCensus << std::endl;
  std::string sgm_log = "./sgm_log.txt";
  try
  {
    JR::Phobos::MatchStereo(matcher_handle,
                            left_image,
                            right_image,
                            cv::Mat(),
                            cv::Mat(),
                            disp,
                            sgm_log,
                            JR::Phobos::e_logError);
  }
  catch (const std::exception &ex)
  {
    std::cerr << ex.what() << std::endl;
  }
}

//backward match disparity
void jrsgm::backwardMatch(cv::Mat left_image, cv::Mat right_image, cv::Mat &disp)
{
  cv::Mat left_joint, right_joint;
  std::string sgm_log = "./sgm_log.txt";

  try
  {
    JR::Phobos::MatchStereo(matcher_handle,
                            right_image,
                            left_image,
                            cv::Mat(),
                            cv::Mat(),
                            disp,
                            sgm_log,
                            JR::Phobos::e_logError);
  }
  catch (const std::exception &ex)
  {
    std::cerr << ex.what() << std::endl;
  }
}

void jrsgm::setP1(float P1)
{
  float P1_scaled = P1 / 100;
  for (auto &pyramid : params.oPyramidParams)
  {
    pyramid.oSGMParams.fP1_E_W = P1_scaled;
    pyramid.oSGMParams.fP1_SE_NW = P1_scaled;
    pyramid.oSGMParams.fP1_SW_NE = P1_scaled;
    pyramid.oSGMParams.fP1_S_N = P1_scaled;
  }
  createMatcher();
}

void jrsgm::setP2(float P2)
{
  float P2_scaled = P2 / 100;
  for (auto &pyramid : params.oPyramidParams)
  {
    pyramid.oSGMParams.fP2_E_W = P2_scaled;
    pyramid.oSGMParams.fP2_SE_NW = P2_scaled;
    pyramid.oSGMParams.fP2_SW_NE = P2_scaled;
    pyramid.oSGMParams.fP2_S_N = P2_scaled;
  }
  createMatcher();
}

void jrsgm::setWindowSize(int census_size)
{
  if (census_size > 13){
    std::cout << "census size " << census_size << " is too large. Will use maximum 13" << std::endl;
    census_size = 13;
  }
  for (auto &pyramid : params.oPyramidParams)
  {
    pyramid.oMetricParams.nWindowSizeX = census_size;
    pyramid.oMetricParams.nWindowSizeY = census_size;
  }

  createMatcher();
}

void jrsgm::setDisparityShift(int shift)
{
  params.fTopPredictionShift = shift / pow(2, params.nNumberOfPyramids - 1);

  createMatcher();
}

void jrsgm::enableSubpixel(bool enable)
{
  params.oFinalSubPixelParameters.bCompute = enable;

  createMatcher();
}

void jrsgm::setDisparityRange(int n)
{
  /* Set disparity range for all pyramids */
  if (n % 2)
  {
    disparity_range = n;
    params.oPyramidParams[0].nMaximumNumberOfDisparities = n;
    params.oPyramidParams[1].nMaximumNumberOfDisparities = n;
    params.oPyramidParams[2].nMaximumNumberOfDisparities = n;
    params.oPyramidParams[3].nMaximumNumberOfDisparities = n;
    params.oPyramidParams[4].nMaximumNumberOfDisparities = n;

    params.oFinalSubPixelParameters.nMaximumNumberOfDisparities = n;
  }

  createMatcher();
}

void jrsgm::enableTextureDSI(bool enable){
  for (auto &pyramid : params.oPyramidParams)
  {
    pyramid.oSGMParams.bUseDSITexture = enable;
  }
}

void jrsgm::enableInterpolation(bool enable)
{
  /* Toggle interpolation */
  for (auto &pyramid : params.oPyramidParams)
  {
    pyramid.bInterpol = enable;
  }

  params.oFinalSubPixelParameters.bInterpol = enable;
  createMatcher();
}

void jrsgm::enableOcclusionDetection(bool enable)
{
  /* Toggle occlusion detection */
  for (auto &pyramid : params.oPyramidParams)
  {
    pyramid.bOcclusionDetection = enable;
  }

  params.oFinalSubPixelParameters.bOcclusionDetection = enable;

  createMatcher();
}

void jrsgm::createMatcher()
{
  if (matcher_handle != nullptr)
  {
    JR::Phobos::DestroyMatchStereoHandle(matcher_handle);
  }
  try
  {
    matcher_handle = JR::Phobos::CreateMatchStereoHandle(params);
  } catch  (const std::exception &ex)
  {
    std::cerr << "Failed to create I3DR matcher with chosen parameters" << std::endl;
    std::cerr << ex.what() << std::endl;
  }
}