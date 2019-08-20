#ifndef MATCHERJRSGM_H
#define MATCHERJRSGM_H

#include <stereoMatcher/abstractStereoMatcher.h>
#include <PhobosIntegration/PhobosIntegration.hpp>

class MatcherJRSGM : public AbstractStereoMatcher
{
public:
    explicit MatcherJRSGM(std::string param_file)
        : AbstractStereoMatcher(param_file)
    {
        param_file_ = param_file;
        init();
    }

    void forwardMatch(void);
    void backwardMatch(void);

private:

    void init(void);
    void setupDefaultMatcher(void);

    std::string param_file_;

    JR::Phobos::TSTEREOHANDLE matcher_handle = nullptr;
    JR::Phobos::SMatchingParametersInput params;
};

#endif // MATCHERJRSGM_H