#include "ImageStitching.hpp"

void ImageStitching::initial() {
    readImg();
    getGray();
    beginSIFT();
    drawSIFTpoint();
    threshold = 9;
}

void ImageStitching::readImg() {
    char c1, c2;
    cin >> c1 >> c2;
    // string path1 = "./data/1.bmp";
    // string path2 = "./data/2.bmp";
    string path1 = "./data/";
    string path2 = "./data/";
    path1 += c1; path1 += ".bmp";
    path2 += c2; path2 += ".bmp";

    srcs.push_back(CImg<unsigned char>(path1.c_str()));
    srcs.push_back(CImg<unsigned char>(path2.c_str()));
}

void ImageStitching::getGray() {
    CImg<float> temp1(srcs[0].width(), srcs[0].height(), 1, 1);
    CImg<float> temp2(srcs[1].width(), srcs[1].height(), 1, 1);

    float r,g,b;

    for(int i = 0; i < srcs[0].width(); i++) {
        for(int j = 0; j < srcs[0].height(); j++) {
            r = srcs[0](i, j, 0, 0);
            g = srcs[0](i, j, 0, 1);
            b = srcs[0](i, j, 0, 2);

            temp1(i, j, 0, 0) = 0.299*r + 0.587*g + 0.114*b;
        }
    }
    for(int i = 0; i < srcs[1].width(); i++) {
        for(int j = 0; j < srcs[1].height(); j++) {
            r = srcs[1](i, j, 0, 0);
            g = srcs[1](i, j, 0, 1);
            b = srcs[1](i, j, 0, 2);

            temp2(i, j, 0, 0) = 0.299*r + 0.587*g + 0.114*b;
        }
    }

    // temp1.display("1");
    // temp2.display("2");
    grays.push_back(temp1);
    grays.push_back(temp2);
}

void ImageStitching::beginSIFT() {
    for (int i = 0; i < grays.size(); ++i) {
        descriptors.push_back(Sift::compute_sift(grays[i]));
        marks.push_back(srcs[i]);
    }
}

void ImageStitching::drawSIFTpoint() {
    unsigned char red[3] = {255, 0, 0};
    for (int i = 0; i < grays.size(); ++i) {
        for (auto k : descriptors[i]) {
            marks[i].draw_circle(k.col, k.row, 3, red, 5.0f, 1);
        }
        // marks[i].display("SIFT");
        // marks[i].save("./result/marks");
    }
    marks[0].save("./result/marks1.bmp");
    marks[1].save("./result/marks2.bmp");
}

void ImageStitching::test() {
    cout << endl << descriptors[0].size() << endl;
    float a = descriptors[0][0].sigma;
    for (int i = 0; i < descriptors[0].size(); ++i) {
        if (a == descriptors[0][i].sigma) {
            cout << "(" << descriptors[0][i].col << ", " << descriptors[0][i].row << ", " << descriptors[0][i].sigma << ")   ";
        }
    }
}

void ImageStitching::run() {
    // test();
    match();
    drawOneImage();
    RANSAC();
    drawOneImage2();
}

void ImageStitching::match() {
    match1.clear();
    match2.clear();
    for (int i = 0; i < descriptors[0].size(); ++i) {
        float fclose = 99998;
        int SIFTfirstIndex = 0;
        float sclose = 99999;
        int SIFTsecondIndex = 0;

        for (int j = 0; j < descriptors[1].size(); ++j) {
            float dis = siftDistance(descriptors[0][i], descriptors[1][j]);
            if (dis < fclose) {
                sclose = fclose;
                SIFTsecondIndex = SIFTfirstIndex;
                fclose = dis;
                SIFTfirstIndex = j;
            } else if (dis < sclose) {
                sclose = dis;
                SIFTsecondIndex = j;
            }
        }

        if (fclose / sclose < RATIO) {
            match1.push_back(descriptors[0][i]);
            match2.push_back(descriptors[1][SIFTfirstIndex]);
        }
    }
}

void ImageStitching::drawOneImage() {
    int w = srcs[0].width() + srcs[1].width();
    int h = srcs[0].height() > srcs[1].height() ? srcs[0].height() : srcs[1].height();
    CImg<float> bigImage(w, h, 1, 3);
    int offset = srcs[1].width();
    unsigned char red[3] = {255, 0, 0};

    cimg_forXY(marks[0], x, y) {
        bigImage(x, y, 0) = marks[0](x, y, 0);
        bigImage(x, y, 1) = marks[0](x, y, 1);
        bigImage(x, y, 2) = marks[0](x, y, 2);
    }

    cimg_forXY(marks[1], x, y) {
        bigImage(x + offset, y, 0) = marks[1](x, y, 0);
        bigImage(x + offset, y, 1) = marks[1](x, y, 1);
        bigImage(x + offset, y, 2) = marks[1](x, y, 2);
    }

    for (int i = 0; i < match1.size(); ++i) {
        bigImage.draw_line(match1[i].col, match1[i].row, match2[i].col + offset, match2[i].row, red);
    }

    // bigImage.display("try1");
    bigImage.save("./result/bigImage1.bmp");
}

void ImageStitching::drawOneImage2() {
    int w = srcs[0].width() + srcs[1].width();
    int h = srcs[0].height() > srcs[1].height() ? srcs[0].height() : srcs[1].height();
    CImg<float> bigImage(w, h, 1, 3);
    int offset = srcs[1].width();
    unsigned char red[3] = {0, 0, 255};

    cimg_forXY(marks[0], x, y) {
        bigImage(x, y, 0) = marks[0](x, y, 0);
        bigImage(x, y, 1) = marks[0](x, y, 1);
        bigImage(x, y, 2) = marks[0](x, y, 2);
    }

    cimg_forXY(marks[1], x, y) {
        bigImage(x + offset, y, 0) = marks[1](x, y, 0);
        bigImage(x + offset, y, 1) = marks[1](x, y, 1);
        bigImage(x + offset, y, 2) = marks[1](x, y, 2);
    }

    for (int i = 0; i < match1.size(); ++i) {
        if (inlinerIndex[i] == 1) {
            bigImage.draw_line(match1[i].col, match1[i].row, match2[i].col + offset, match2[i].row, red);
        }
    }

    bigImage.save("./result/bigImage2.bmp");
    bigImage.display("try2");
}

void ImageStitching::RANSAC() {
    int k = 0, maxGoodCount = 0, goodCount = 0, m = 4, runtime = 9000, run = 0;
    double in_frac = RANSAC_INLIER_FRAC_EST;
    double p = pow( 1.0 - pow( in_frac, m ), k );

    while (p > p_badxform && runtime > run) {
        randomPickSample();
        computeHomography();
        goodCount = findInliner();

        if (goodCount > maxGoodCount) {
            Hmatrix = tmatrix;
            maxGoodCount = goodCount;
            in_frac = (double)maxGoodCount / match1.size();
        }

        p = pow( 1.0 - pow( in_frac, m ), ++k );
        runtime = updateRuntime(1 - in_frac, runtime);
        run++;

        cout << "p: " << p << ", runtime: " << runtime << ", run: " << run << ", goodCount: " << goodCount << endl;
    }
}

void ImageStitching::randomPickSample() {
    srand(time(nullptr));
    int r = rand() % match1.size();
    int stack[5] = {0, 0, 0, 0, 0};
    stack[1] = r;
    sample1.clear();
    sample2.clear();
    // the first
    sample1.push_back(&(match1[r]));
    sample2.push_back(&(match2[r]));

    r = rand() % match1.size();
    while (r == stack[1]) {
        r = rand() % match1.size();
    }
    // the second
    sample1.push_back(&(match1[r]));
    sample2.push_back(&(match2[r]));

    r = rand() % match1.size();
    while (r == stack[1] || r == stack[2]) {
        r = rand() % match1.size();
    }
    // the third
    if (checkSample(*sample1[0], *sample1[1], match1[r]) && checkSample(*sample2[0], *sample2[1], match2[r])) {
        sample1.push_back(&(match1[r]));
        sample2.push_back(&(match2[r]));
    }

    r = rand() % match1.size();
    while (r == stack[1] || r == stack[2] || r == stack[3]) {
        r = rand() % match1.size();
    }
    // the last
    if (checkSample(*sample1[0], *sample1[1], match1[r]) && checkSample(*sample2[0], *sample2[1], match2[r]) &&
        checkSample(*sample1[0], *sample1[2], match1[r]) && checkSample(*sample2[0], *sample2[2], match2[r]) &&
        checkSample(*sample1[2], *sample1[1], match1[r]) && checkSample(*sample2[2], *sample2[1], match2[r])) {
        sample1.push_back(&(match1[r]));
        sample2.push_back(&(match2[r]));
    }
}

bool ImageStitching::checkSample(const SiftDescriptor& x, const SiftDescriptor& y, const SiftDescriptor& z) {
    double e = abs((x.col - y.col) * (z.row - y.row) - (y.col - z.col) * (y.row - x.row));
    return !(e < 0.05);
} 

void ImageStitching::computeHomography() {
    CImg<float> A(8, 8, 1, 1, 0), B(1, 8, 1, 1, 0), X(1, 8, 1, 1, 0);

    /* set up matrices so we can unstack homography into X; AX = B */
    tmatrix.push_back(vector<float>(3, 0));
    tmatrix.push_back(vector<float>(3, 0));
    tmatrix.push_back(vector<float>(3, 0));
    // for(int i = 0; i < 4; i++) {
    //     A(i  , 0) = sample1[i]->col;
    //     A(i+4, 3) = sample1[i]->col;
    //     A(i  , 1) = sample1[i]->row;
    //     A(i+4, 4) = sample1[i]->row;
    //     A(i  , 2) = 1.0;
    //     A(i+4, 5) = 1.0;
    //     A(i  , 6) = -sample1[i]->col * sample2[i]->col;
    //     A(i  , 7) = -sample1[i]->row * sample2[i]->col;
    //     A(i+4, 6) = -sample1[i]->col * sample2[i]->row;
    //     A(i+4, 7) = -sample1[i]->row * sample2[i]->row;
    //     B(i  , 0) = sample2[i]->col;
    //     B(i+4, 0) = sample2[i]->row;
    // }
    for(int i = 0; i < 4; i++) {
        A(0, i  ) = sample1[i]->col;
        A(3, i+4) = sample1[i]->col;
        A(1, i  ) = sample1[i]->row;
        A(4, i+4) = sample1[i]->row;
        A(2, i  ) = 1.0;
        A(5, i+4) = 1.0;
        A(6, i  ) = -sample1[i]->col * sample2[i]->col;
        A(7, i  ) = -sample1[i]->row * sample2[i]->col;
        A(6, i+4) = -sample1[i]->col * sample2[i]->row;
        A(7, i+4) = -sample1[i]->row * sample2[i]->row;
        B(0, i  ) = sample2[i]->col;
        B(0, i+4) = sample2[i]->row;
    }
    X = B.solve(A);
    // cout << "A:\n";
    // for (int i = 0; i < 8; ++i)
    // {
    //     for (int j = 0; j < 8; ++j)
    //     {
    //         cout << A(i, j) << ", ";
    //     }
    //     cout << endl;
    // }
    // cout << "------\nB:\n";
    // for (int i = 0; i < 8; ++i)
    // {
    //     cout << B(0, i) << ", ";
    // }
    // cout << "------\nm:\n";
    int index = 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            tmatrix[i][j] = X(0, index++);
            // cout << tmatrix[i][j] << ", ";
        }
        // cout << endl;
    }
    tmatrix[2][2] = 1.0;
}

int ImageStitching::findInliner() {
    inlinerIndex.clear();
    int i, count = match1.size(), goodCount = 0;  
    err.clear();
    computeReprojError();  //err里面是计算后的矩阵的大小，用于与阈值比较  

    for( i = 0; i < count; i++ ) {
        inlinerIndex.push_back((err[i] <= threshold));
        goodCount += inlinerIndex[i]; //goodCount为计算出的内点的个数  
    }
    return goodCount;
} 

void ImageStitching::computeReprojError() {
    int i, count = match1.size();

    for (i = 0; i < count; i++) {
        double ww = 1./(tmatrix[2][0]*match1[i].col + tmatrix[2][1]*match1[i].row + 1.);
        double dx = (tmatrix[0][0]*match1[i].col + tmatrix[0][1]*match1[i].row + tmatrix[0][2])*ww - match2[i].col;
        double dy = (tmatrix[1][0]*match1[i].col + tmatrix[1][1]*match1[i].row + tmatrix[1][2])*ww - match2[i].row;
        err.push_back((float)(dx*dx + dy*dy));
    }
}

int ImageStitching::updateRuntime(double ep, int runtime) {
    // p 置信概率, ep outliner比例
    double p = 1 - p_badxform;

    // avoid inf's & nan's
    double num = max(1. - p, DBL_MIN);  
    double denom = 1. - pow(1. - ep, 4);  
    if ( denom < DBL_MIN ) {
        return 0;  
    }

    num = log(num);
    denom = log(denom);

    return denom >= 0 || -num >= runtime*(-denom) ?  
        runtime : (int)(num/denom); 
}

float ImageStitching::siftDistance(const SiftDescriptor& x, const SiftDescriptor& y) {
    float dis = 0;
    for(int i = 0; i < 128; i++) {
        dis += pow((x.descriptor[i] - y.descriptor[i]), 2.0);
    }
    return sqrt(dis);
}