#ifndef IS_HPP
#define IS_HPP

#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include "CImg.h"
#include "Sift.h"

using namespace std;
using namespace cimg_library;

#define RANSAC_INLIER_FRAC_EST 0.25
#define p_badxform 0.005
#define RATIO 0.7
#define FUSION_SWITCH true

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
    void computeAllHomography();
    int findInliner();
    int updateRuntime(double ep, int runtime);
    void computeReprojError();
    void randomPickSample();
    bool checkSample(const SiftDescriptor& x, const SiftDescriptor& y, const SiftDescriptor& z);
    double siftDistance(const SiftDescriptor& x, const SiftDescriptor& y);
    void test();

    void imageFusion();
    void saveImg(const CImg<float>& img, const string& s, int i);
    void over();
    void readAgain();
    void getGrayAgain();
    void beginSIFTAgain();
    void drawSIFTpointAgain();
    void computeHT();
    void refine(CImg<float>& img);
    
private:
    vector<CImg<float>> srcs;
    vector<CImg<float>> grays;
    vector<vector<SiftDescriptor>> descriptors;
    vector<SiftDescriptor> match1, match2;
    vector<CImg<float>> marks;
    vector<vector<double>> Hmatrix, tmatrix, HTmatrix;
    vector<SiftDescriptor*> sample1, sample2;
    double threshold;
    vector<double> err;
    vector<int> inlinerIndex;
    vector<int> maxInlinerIndex;
    vector<int> randomIndex;
    CImg<float> fusion;
    int times;
    int total;
};

#endif