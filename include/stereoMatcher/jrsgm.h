#ifndef JRSGM_H
#define JRSGM_H

#include <PhobosIntegration/PhobosIntegration.hpp>
#include <iostream>

class jrsgm {
    public:
        jrsgm(std::string &sConfigFile){
            init(sConfigFile);
        }
    
        ~jrsgm(void){
            if (matcher_handle != nullptr){
                JR::Phobos::DestroyMatchStereoHandle(matcher_handle);
            }
        }

        void parseConfig(std::string input_file);
        int getErrorDisparity();
        void createMatcher();

        void setDisparityShift(int shift);
        void setDisparityRange(int n);
        void enableSubpixel(bool enable);
        void setP1(float P1);
        void setP2(float P2);
        void setWindowSize(int census_size);
        void enableInterpolation(bool enable);
        void enableOcclusionDetection(bool enable);
        void enableTextureDSI(bool enable);

        float getP1(void){ return params.oPyramidParams[0].oSGMParams.fP1_E_W; }
        float getP2(void){ return params.oPyramidParams[0].oSGMParams.fP2_E_W; }
        int getDisparityRange(void){ return params.oPyramidParams[0].nMaximumNumberOfDisparities; }
        int getCensusSize(void){ return params.oPyramidParams[0].oMetricParams.nWindowSizeX; }
        bool getInterpolate(void){return params.oFinalSubPixelParameters.bInterpol; }
        bool getOcclusionDetect(void){ return params.oFinalSubPixelParameters.bOcclusionDetection; }
        bool getSubpixel(void){return params.oFinalSubPixelParameters.bCompute; }
        int getDisparityShift(void){return params.fTopPredictionShift * pow(2, params.nNumberOfPyramids-1) ; }

        void compute(cv::Mat left_image, cv::Mat right_image, cv::Mat &disp);
        void backwardMatch(cv::Mat left_image, cv::Mat right_image, cv::Mat &disp);

    private:
        JR::Phobos::TSTEREOHANDLE matcher_handle = nullptr;
        JR::Phobos::SMatchingParametersInput params;
        int min_disparity, disparity_range;

        void init(std::string &sConfigFile);
        int round_up_to_32(int val);
        int checkMemoryDSI(int image_width, int image_height);
        int checkMemoryCensus(int image_width, int image_height);
};

#endif // JRSGM_H