#ifndef IS_HPP
#define IS_HPP

#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include "CImg.h"
#include "Sift.h"

using namespace std;
using namespace cimg_library;

#define DATA_COUNT 2
#define RANSAC_INLIER_FRAC_EST 0.25
#define p_badxform 0.005
#define RATIO 0.7

class ImageStitching {
public:
    ImageStitching() = default;
    ~ImageStitching() = default;
    void initial();
    void readImg();
    void getGray();
    void beginSIFT();
    void drawSIFTpoint();
    void run();
    void match();
    void drawOneImage();
    void drawOneImage2();
    void RANSAC();
    void computeHomography();
    int findInliner();
    int updateRuntime(double ep, int runtime);
    void computeReprojError();
    void randomPickSample();
    bool checkSample(const SiftDescriptor& x, const SiftDescriptor& y, const SiftDescriptor& z);
    float siftDistance(const SiftDescriptor& x, const SiftDescriptor& y);
    void test();
    
private:
    vector<CImg<unsigned char>> srcs;
    vector<CImg<float>> grays;
    vector<vector<SiftDescriptor>> descriptors;
    vector<SiftDescriptor> match1, match2;
    vector<CImg<float>> marks;
    CImg<float> matchResult;
    vector<vector<float>> Hmatrix, tmatrix;
    vector<SiftDescriptor*> sample1, sample2;
    float threshold;
    vector<float> err;
    vector<int> inlinerIndex;
};

#endif