#include <stereoMatcher/jrsgm.h>

//Initialise matcher
void jrsgm::init(std::string &sConfigFile, cv::Size _image_size)
{
  this->image_size = _image_size;
  this->param_file = sConfigFile;

  std::cout << this->param_file << std::endl;
  this->params_raw = ReadFileRaw(this->param_file);

  createMatcher();
}

bool jrsgm::EditParamRaw(std::vector<std::string> *lines, std::string param_name, std::string param_value)
{
  // find location of pyramid in file
  std::string full_parameters_string = param_name + " = " + param_value;
  bool param_found = false;
  for (std::vector<std::string>::iterator it = lines->begin(); it != lines->end(); ++it)
  {
    std::size_t found = it->find(param_name);
    if (found != std::string::npos)
    {
      param_found = true;
      int index = std::distance(lines->begin(), it);
      std::cerr << "!!!!Parameter found @ " << index << "!!!" << std::endl;
      EditLineRaw(lines, full_parameters_string, index);
      break;
    }
  }
  return param_found;
}

bool jrsgm::EditPyramidParamRaw(std::vector<std::string> *lines, int pyramid_num, std::string param_name, std::string param_value, bool is_subpix)
{
  // find location of pyramid in file
  std::string pyramid_string;
  if (is_subpix)
  {
    pyramid_string = "[Pyramid " + std::to_string(pyramid_num) + " Subpix]";
  }
  else
  {
    pyramid_string = "[Pyramid " + std::to_string(pyramid_num) + "]";
  }
  std::string full_parameters_string = param_name + " = " + param_value;
  bool pyramid_found = false;
  bool param_found = false;
  for (std::vector<std::string>::iterator it = lines->begin(); it != lines->end(); ++it)
  {
    if (pyramid_found)
    {
      std::size_t found = it->find(param_name);
      if (found != std::string::npos)
      {
        param_found = true;
        int index = std::distance(lines->begin(), it);
        EditLineRaw(lines, full_parameters_string, index);
        break;
      }
    }
    else
    {
      if (it->find(pyramid_string) != std::string::npos)
      {
        pyramid_found = true;
      }
    }
  }
  return pyramid_found && param_found;
}

void jrsgm::EditLineRaw(std::vector<std::string> *lines, std::string value, int line_num)
{
  lines->at(line_num) = value + "\n";
}

std::vector<std::string> jrsgm::ReadFileRaw(std::string &filename)
{
  std::ifstream file(filename);
  std::vector<std::string> lines;
  if (file.is_open())
  {
    std::string line;
    while (getline(file, line))
    {
      lines.push_back(line);
    }
    file.close();
  }
  else
  {
    std::cerr << "Unable to open file";
  }
  return lines;
}

void jrsgm::WriteIniFileRaw(std::string &filename, std::vector<std::string> lines)
{
  std::ofstream file(filename);
  if (file.is_open())
  {
    for (std::vector<std::string>::iterator it = lines.begin(); it != lines.end(); ++it)
    {
      std::string line = *it;
      file << line << "\n";
    }
    file.close();
  }
  else
  {
    std::cerr << "Unable to open file";
  }
}

int jrsgm::getErrorDisparity(void)
{
  return -10000;
}

int jrsgm::round_up_to_32(int val)
{
  return round(val / 32) * 32;
}

size_t jrsgm::checkMemoryAvailable()
{
  //re-calculate current CUDA GPU memory
  cudaMemory.calcMem();
  size_t memFree = cudaMemory.getMemFree();
  //size_t memUsed = cudaMemory.getMemUsed();
  //size_t memTotal = cudaMemory.getMemTotal();
  /*
  std::cout << "GPU MEMORY FREE: " << float(memFree) / 1073741824. << std::endl;
  std::cout << "GPU MEMORY USED: " << float(memUsed) / 1073741824. << std::endl;
  std::cout << "GPU MEMORY TOTAL: " << float(memTotal) / 1073741824. << std::endl;
  */
  return (memFree);
}

int jrsgm::checkMemoryCensus(int image_width, int image_height)
{
  int num_of_gpus = params.oGPUs.size();
  int num_of_pyramids = params.nNumberOfPyramids;
  int window_size_x = params.oPyramidParams[0].oMetricParams.nWindowSizeX;
  int window_size_y = params.oPyramidParams[0].oMetricParams.nWindowSizeY;
  int num_of_slanted_window_scales = params.oPyramidParams[0].oMetricParams.nNumberOfScales;
  int num_of_slanted_window_shifts = params.oPyramidParams[0].oMetricParams.nNumberOfSlants;
  int have_no_data = params.bHasNodata;

  bool enMeanCensus = true;
  int NumberOfNeededCensusBytes;
  if (enMeanCensus)
  {
    NumberOfNeededCensusBytes = round_up_to_32(window_size_x * window_size_y * (1 + have_no_data)) / 4 * (num_of_slanted_window_scales + num_of_slanted_window_shifts * 2);
  }
  else
  {
    NumberOfNeededCensusBytes = round_up_to_32((window_size_x * window_size_y - 1) * (1 + have_no_data)) / 4 * (num_of_slanted_window_scales + num_of_slanted_window_shifts * 2);
  }
  int MemoryCensus_PerGPU = (num_of_pyramids * (2 * image_width * image_height * NumberOfNeededCensusBytes)) / num_of_gpus;
  return MemoryCensus_PerGPU;
}

int jrsgm::checkMemoryDSI(int image_width, int image_height)
{
  int num_of_pyramids = params.nNumberOfPyramids;
  int num_of_disparities = params.oPyramidParams[0].nMaximumNumberOfDisparities;
  bool enTextureDSI = params.oPyramidParams[0].oSGMParams.bUseDSITexture;
  int texture_dsi;
  if (enTextureDSI)
  {
    texture_dsi = 2;
  }
  else
  {
    texture_dsi = 1;
  }
  int MemoryDSI_PerGPU = num_of_pyramids * (2 * image_width * image_height * num_of_disparities);
  return MemoryDSI_PerGPU;
}

int jrsgm::checkMemoryExtra(int image_width, int image_height)
{
  int num_of_pyramids = params.nNumberOfPyramids;
  return (num_of_pyramids * (image_width * image_height * 4 * 10));
}

bool jrsgm::checkMemoryValid(int image_width, int image_height)
{
  int memoryDSI = checkMemoryDSI(image_width, image_height);
  int memoryCensus = checkMemoryCensus(image_width, image_height);
  int memoryExtra = checkMemoryExtra(image_width, image_height);
  int memoryUsed = memoryDSI + memoryCensus + memoryExtra;
  size_t memoryAvaiable = checkMemoryAvailable();
  std::cout << "Image size: (" << image_width << "," << image_height << ")" << std::endl;
  std::cout << "GPU MEMORY USED: " << float(memoryUsed) / 1073741824. << std::endl;
  std::cout << "GPU MEMORY AVAILABLE: " << float(memoryAvaiable) / 1073741824. << std::endl;
  if ((memoryUsed) < memoryAvaiable)
  {
    isMemoryValid = true;
    return true;
  }
  else
  {
    isMemoryValid = false;
    return false;
  }
}

void jrsgm::setImages(cv::Mat left_image, cv::Mat right_image)
{
  left_image.copyTo(this->image_left);
  right_image.copyTo(this->image_right);
  image_size = left_image.size();
}

//compute disparity
int jrsgm::compute(cv::Mat &disp)
{
  std::cout << "[jrsgm] Starting match..." << std::endl;
  if (isMemoryValid)
  {
    if (matcher_handle != nullptr)
    {
      std::string sgm_log = "./sgm_log.txt";
      try
      {
        JR::Phobos::MatchStereo(matcher_handle,
                                image_left,
                                image_right,
                                cv::Mat(),
                                cv::Mat(),
                                disp,
                                sgm_log,
                                JR::Phobos::e_logError);
      }
      catch (...)
      {
        std::cerr << "FAILED TO COMPUTE" << std::endl;
        return -1;
      }
    }
    else
    {
      std::cerr << "Matcher handle not found" << std::endl;
      createMatcher();
      return -3;
    }
  }
  else
  {
    return -2;
    std::cerr << "Invalid GPU memory for stereo match" << std::endl;
  }
  std::cout << "[jrsgm] Match complete." << std::endl;
  return 0;
}

//backward match disparity
int jrsgm::backwardMatch(cv::Mat &disp)
{
  std::cout << "[jrsgm] Starting match..." << std::endl;
  if (isMemoryValid)
  {
    if (matcher_handle != nullptr)
    {
      std::string sgm_log = "./sgm_log.txt";
      try
      {
        JR::Phobos::MatchStereo(matcher_handle,
                                image_right,
                                image_left,
                                cv::Mat(),
                                cv::Mat(),
                                disp,
                                sgm_log,
                                JR::Phobos::e_logError);
      }
      catch (...)
      {
        std::cerr << "FAILED TO COMPUTE" << std::endl;
        return -1;
      }
    }
    else
    {
      std::cerr << "Matcher handle not found" << std::endl;
      createMatcher();
      return -3;
    }
  }
  else
  {
    return -2;
    std::cerr << "Invalid GPU memory for stereo match" << std::endl;
  }
  std::cout << "[jrsgm] Match complete." << std::endl;
  return 0;
}

void jrsgm::setSpeckleDifference(float diff)
{
  std::cout << "Speckle difference: " << diff << std::endl;
  diff = diff / 10;
  /*
  for (auto &pyramid : params.oPyramidParams)
  {
    pyramid.fSpeckleMaxDiff = diff;
  }
  */
  int i = 0;
  std::string param_name = "Disparity Speckle Filter Max Difference";
  std::string param_val = std::to_string(diff);
  for (auto &pyramid : params.oPyramidParams)
  {
    EditPyramidParamRaw(&this->params_raw, i, param_name, param_val);
    i++;
  }
  EditPyramidParamRaw(&this->params_raw, 0, param_name, param_val, true);
  createMatcher();
  createMatcher();
}

void jrsgm::setSpeckleSize(int size)
{
  std::cout << "Speckle size: " << size << std::endl;
  size = size / 10;
  /*
  for (auto &pyramid : params.oPyramidParams)
  {
    pyramid.iSpeckleMaxSize = size;
  }
  */
  int i = 0;
  std::string param_name = "Disparity Speckle Filter Max Region Size";
  std::string param_val = std::to_string(size);
  for (auto &pyramid : params.oPyramidParams)
  {
    EditPyramidParamRaw(&this->params_raw, i, param_name, param_val);
    i++;
  }
  EditPyramidParamRaw(&this->params_raw, 0, param_name, param_val, true);
  createMatcher();
}

void jrsgm::setP1(float P1)
{
  float P1_scaled = P1 / 1000;
  float P1_subpix = P1_scaled / 10;
  /*
  for (auto &pyramid : params.oPyramidParams)
  {
    pyramid.oSGMParams.fP1_E_W = P1_scaled;
    pyramid.oSGMParams.fP1_SE_NW = P1_scaled;
    pyramid.oSGMParams.fP1_SW_NE = P1_scaled;
    pyramid.oSGMParams.fP1_S_N = P1_scaled;
  }
  */
  int i = 0;
  std::string param_name_1 = "SE-NW Penalty 1";
  std::string param_name_2 = "SN Penalty 1";
  std::string param_name_3 = "SW-NE Penalty 1";
  std::string param_name_4 = "WE Penalty 1";
  std::string param_val = std::to_string(P1_scaled);
  for (auto &pyramid : params.oPyramidParams)
  {
    EditPyramidParamRaw(&this->params_raw, i, param_name_1, param_val);
    EditPyramidParamRaw(&this->params_raw, i, param_name_2, param_val);
    EditPyramidParamRaw(&this->params_raw, i, param_name_3, param_val);
    EditPyramidParamRaw(&this->params_raw, i, param_name_4, param_val);
    i++;
  }
  std::string param_subpix_val = std::to_string(P1_subpix);
  EditPyramidParamRaw(&this->params_raw, 0, param_name_1, param_subpix_val, true);
  EditPyramidParamRaw(&this->params_raw, 0, param_name_2, param_subpix_val, true);
  EditPyramidParamRaw(&this->params_raw, 0, param_name_3, param_subpix_val, true);
  EditPyramidParamRaw(&this->params_raw, 0, param_name_4, param_subpix_val, true);
  createMatcher();
}

void jrsgm::setP2(float P2)
{
  float P2_scaled = P2 / 1000;
  float P2_subpix = P2_scaled / 10;
  /*
  for (auto &pyramid : params.oPyramidParams)
  {
    pyramid.oSGMParams.fP2_E_W = P2_scaled;
    pyramid.oSGMParams.fP2_SE_NW = P2_scaled;
    pyramid.oSGMParams.fP2_SW_NE = P2_scaled;
    pyramid.oSGMParams.fP2_S_N = P2_scaled;
  }
  */

  int i = 0;
  std::string param_name_1 = "SE-NW Penalty 2";
  std::string param_name_2 = "SN Penalty 2";
  std::string param_name_3 = "SW-NE Penalty 2";
  std::string param_name_4 = "WE Penalty 2";
  std::string param_val = std::to_string(P2_scaled);
  for (auto &pyramid : params.oPyramidParams)
  {
    EditPyramidParamRaw(&this->params_raw, i, param_name_1, param_val);
    EditPyramidParamRaw(&this->params_raw, i, param_name_2, param_val);
    EditPyramidParamRaw(&this->params_raw, i, param_name_3, param_val);
    EditPyramidParamRaw(&this->params_raw, i, param_name_4, param_val);
    i++;
  }
  std::string param_subpix_val = std::to_string(P2_subpix);
  EditPyramidParamRaw(&this->params_raw, 0, param_name_1, param_subpix_val, true);
  EditPyramidParamRaw(&this->params_raw, 0, param_name_2, param_subpix_val, true);
  EditPyramidParamRaw(&this->params_raw, 0, param_name_3, param_subpix_val, true);
  EditPyramidParamRaw(&this->params_raw, 0, param_name_4, param_subpix_val, true);
  createMatcher();
}

void jrsgm::setWindowSize(int census_size)
{
  if (census_size % 2 == 0)
  {
    census_size++;
  }

  /*
  for (auto &pyramid : params.oPyramidParams)
  {
    pyramid.oMetricParams.nWindowSizeX = census_size;
    pyramid.oMetricParams.nWindowSizeY = census_size;
  }
  
  //params.oPyramidParams[0].oMetricParams.nWindowSizeX = census_size;
  //params.oPyramidParams[0].oMetricParams.nWindowSizeY = census_size;
  */

  int i = 0;
  std::string x_param_name = "Feature Set Size X";
  std::string y_param_name = "Feature Set Size Y";
  std::string param_val = std::to_string(census_size);
  for (auto &pyramid : params.oPyramidParams)
  {
    EditPyramidParamRaw(&this->params_raw, i, x_param_name, param_val);
    EditPyramidParamRaw(&this->params_raw, i, y_param_name, param_val);
    i++;
  }
  createMatcher();
}

void jrsgm::setDisparityShift(int shift)
{
  //params.fTopPredictionShift = shift_p;

  std::cout << "shift: " << shift << std::endl;
  double shift_p = (double)shift / 20;
  std::cout << "shift_p: " << shift_p << std::endl;

  std::string param_val = std::to_string(int(shift_p));

  std::string param_name = "Top Prediction Shift";
  EditParamRaw(&this->params_raw, param_name, param_val);
  createMatcher();
}

void jrsgm::maxPyramid(int pyramid_num){
  std::string param_val_true = "true";
  std::string param_val_false = "false";
  std::string param_name = "Process This Pyramid";

  if (pyramid_num > params.oPyramidParams.size()){
    pyramid_num = params.oPyramidParams.size();
  }

  int i = 0;
  int j = params.oPyramidParams.size() - 1;
  for (auto &pyramid : params.oPyramidParams)
  {
    if (i < pyramid_num){
      EditPyramidParamRaw(&this->params_raw, j, param_name, param_val_true);
      std::cerr << "p: " << j << std::endl;
    } else {
      EditPyramidParamRaw(&this->params_raw, j, param_name, param_val_false);
    }
    std::cerr << "i: " << i << " j: " << j << std::endl;
    i++;
    j--;
  }

  createMatcher();
}

void jrsgm::enablePyramid(bool enable, int pyramid_num){
  std::string param_val;
  if (enable)
  {
    param_val = "true";
  }
  else
  {
    param_val = "false";
  }

  std::string param_name = "Process This Pyramid";
  EditPyramidParamRaw(&this->params_raw, pyramid_num, param_name, param_val);

  createMatcher();
}

void jrsgm::enableSubpixel(bool enable)
{
  //params.oFinalSubPixelParameters.bCompute = enable;

  std::string param_val;
  if (enable)
  {
    param_val = "true";
  }
  else
  {
    param_val = "false";
  }

  std::string param_name = "Process This Pyramid";
  EditPyramidParamRaw(&this->params_raw, 0, param_name, param_val, true);

  createMatcher();
}

void jrsgm::setDisparityRange(int n)
{
  disparity_range = n / 10;
  //force odd number
  if (disparity_range % 2 == 0)
  {
    disparity_range++;
  }
  /* Set disparity range for all pyramids */
  int i = 0;
  std::string param_name = "Number Of Disparities";
  std::string param_val = std::to_string(disparity_range);
  for (auto &pyramid : params.oPyramidParams)
  {
    EditPyramidParamRaw(&this->params_raw, i, param_name, param_val);
    i++;
  }
  EditPyramidParamRaw(&this->params_raw, 0, param_name, param_val, true);
  createMatcher();
}

void jrsgm::enableTextureDSI(bool enable)
{
  /*
  for (auto &pyramid : params.oPyramidParams)
  {
    pyramid.oSGMParams.bUseDSITexture = enable;
  }
  */

  std::string param_val;
  if (enable)
  {
    param_val = "true";
  }
  else
  {
    param_val = "false";
  }


  std::string param_name = "Use DSI  Texture Memory";

  int i = 0;
  for (auto &pyramid : params.oPyramidParams)
  {
    EditPyramidParamRaw(&this->params_raw, i, param_name, param_val);
    i++;
  }
  EditPyramidParamRaw(&this->params_raw, 0, param_name, param_val, true);
  createMatcher();

  createMatcher();
}

void jrsgm::enableInterpolation(bool enable)
{
  /* Toggle interpolation */
  //params.oPyramidParams[1].bInterpol = enable

  std::string param_val;
  if (enable)
  {
    param_val = "true";
  }
  else
  {
    param_val = "false";
  }


  std::string param_name = "Interpolate Disparity";

  int i = 0;
  for (auto &pyramid : params.oPyramidParams)
  {
    EditPyramidParamRaw(&this->params_raw, i, param_name, param_val);
    i++;
  }
  EditPyramidParamRaw(&this->params_raw, 0, param_name, param_val, true);
  createMatcher();
}

void jrsgm::enableOcclusionDetection(bool enable)
{
  /* Toggle occlusion detection */
  /*
  for (auto &pyramid : params.oPyramidParams)
  {
    pyramid.bOcclusionDetection = enable;
  }

  params.oFinalSubPixelParameters.bOcclusionDetection = enable;
  */

  std::string param_val;
  if (enable)
  {
    param_val = "true";
  }
  else
  {
    param_val = "false";
  }


  std::string param_name = "Occlusion Detection";

  int i = 0;
  for (auto &pyramid : params.oPyramidParams)
  {
    EditPyramidParamRaw(&this->params_raw, i, param_name, param_val);
    i++;
  }
  EditPyramidParamRaw(&this->params_raw, 0, param_name, param_val, true);
  createMatcher();
}

void jrsgm::enableOccInterpol(bool enable)
{
  /* Toggle occlusion interpolation */
  /*
  for (auto &pyramid : params.oPyramidParams)
  {
    pyramid.bOccInterpol = enable;
  }

  params.oFinalSubPixelParameters.bOccInterpol = enable;
  */

  std::string param_val;
  if (enable)
  {
    param_val = "true";
  }
  else
  {
    param_val = "false";
  }


  std::string param_name = "Interpolate Occlusions";

  int i = 0;
  for (auto &pyramid : params.oPyramidParams)
  {
    EditPyramidParamRaw(&this->params_raw, i, param_name, param_val);
    i++;
  }
  EditPyramidParamRaw(&this->params_raw, 0, param_name, param_val, true);
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
    //checkMemoryValid(image_size.width, image_size.height);
    isMemoryValid = true;
    if (isMemoryValid)
    {
      std::cout << "Re-creating matcher with new paramters..." << std::endl;
      WriteIniFileRaw(this->tmp_param_file, this->params_raw);
      this->params = JR::Phobos::SMatchingParametersInput();
      JR::Phobos::ReadIniFile(this->params, this->tmp_param_file);
      matcher_handle = JR::Phobos::CreateMatchStereoHandle(params);
      std::cout << "Re-created matcher with new paramters." << std::endl;
    }
    else
    {
      std::cerr << "NOT ENOUGH GPU MEMORY AVAILABLE" << std::endl;
    }
  }
  catch (...)
  {
    std::cerr << "Failed to create I3DR matcher with chosen parameters" << std::endl;
    //std::cerr << ex.what() << std::endl;
  }
}