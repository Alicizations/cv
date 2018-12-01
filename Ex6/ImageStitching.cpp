#include "ImageStitching.hpp"

void ImageStitching::initial() {
    readImg();
    getGray();
    beginSIFT();
    drawSIFTpoint();
    threshold = 9;
}

void ImageStitching::readImg() {
    // string path1 = "./data/1.bmp";
    // string path2 = "./data/2.bmp";
    cout << "input the numbers of images\n";
    cin >> total;
    times = 2;
    cout << "stitching image " << times-1 << " and " << times << endl;
    string path1 = "./data/";
    string path2 = "./data/";
    path1 += '0'+times-1; path1 += ".bmp";
    path2 += '0'+times; path2 += ".bmp";

    srcs.push_back(CImg<float>(path1.c_str()));
    srcs.push_back(CImg<float>(path2.c_str()));
    // refine(srcs[0]);
    // refine(srcs[1]);
}

void ImageStitching::getGray() {
    grays.clear();

    CImg<float> temp1(srcs[0].width(), srcs[0].height(), 1, 1);
    CImg<float> temp2(srcs[1].width(), srcs[1].height(), 1, 1);

    double r,g,b;

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
    descriptors.clear();
    marks.clear();

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
    }

    saveImg(marks[0], "marks1_", times);
    saveImg(marks[1], "marks2_", times);
}

void ImageStitching::test() {
    randomIndex.clear();
    for (int i = 0; i < match1.size(); ++i) {
        randomIndex.push_back(i);
    }
    random_shuffle(randomIndex.begin(), randomIndex.end());
    random_shuffle(randomIndex.begin(), randomIndex.end());
}

void ImageStitching::run() {
    match();
    test();
    drawOneImage();
    RANSAC();
    drawOneImage2();
    // computeHT();
    // computeAllHomography();
    imageFusion();
    over();
}

void ImageStitching::match() {
    match1.clear();
    match2.clear();
    for (int i = 0; i < descriptors[0].size(); ++i) {
        double fclose = 99998;
        int SIFTfirstIndex = 0;
        double sclose = 99999;
        int SIFTsecondIndex = 0;

        for (int j = 0; j < descriptors[1].size(); ++j) {
            double dis = siftDistance(descriptors[0][i], descriptors[1][j]);
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
    int offset = srcs[0].width();
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

    saveImg(bigImage, "bigImage1_", times);
}

void ImageStitching::drawOneImage2() {
    int w = srcs[0].width() + srcs[1].width();
    int h = srcs[0].height() > srcs[1].height() ? srcs[0].height() : srcs[1].height();
    CImg<float> bigImage(w, h, 1, 3);
    int offset = srcs[0].width();
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
        if (maxInlinerIndex[i] == 1) {
            bigImage.draw_line(match1[i].col, match1[i].row, match2[i].col + offset, match2[i].row, red);
        }
    }

    saveImg(bigImage, "bigImage2_", times);
}

void ImageStitching::RANSAC() {
    int k = 0, maxGoodCount = 0, goodCount = 0, m = 4, runtime = 15000, run = 0;
    double in_frac = RANSAC_INLIER_FRAC_EST;
    double p = pow( 1.0 - pow( in_frac, m ), k );
    maxInlinerIndex.clear();

    while (p > p_badxform && (runtime > run || run < 2500)) {
    // while (p > p_badxform) {
        randomPickSample();
        if (sample1.size() != 4) {
            continue;
        }
        computeHomography();
        goodCount = findInliner();

        if (goodCount > maxGoodCount) {
            Hmatrix = tmatrix;
            maxGoodCount = goodCount;
            in_frac = (double)maxGoodCount / match1.size();
            maxInlinerIndex = inlinerIndex;
            // computeHT();
        }

        p = pow( 1.0 - pow( in_frac, m ), ++k );
        runtime = updateRuntime(1 - in_frac, runtime);
        run++;

    }
    cout << "p: " << p << ", runtime: " << runtime << ", run: " << run << ", maxGoodCount: " << maxGoodCount << endl; 
}

void ImageStitching::randomPickSample() {
    sample1.clear();
    sample2.clear();

    for (int i = 0; i < 4; ++i) {
        sample1.push_back(&(match1[randomIndex[i]]));
        sample2.push_back(&(match2[randomIndex[i]]));
    }

    random_shuffle(randomIndex.begin(), randomIndex.end());

    return;

    srand(time(nullptr));
    int r = rand() % match1.size();
    int stack[5] = {0, 0, 0, 0, 0};
    stack[1] = r;
    
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
    CImg<double> A(8, 8, 1, 1, 0), B(1, 8, 1, 1, 0), X(1, 8, 1, 1, 0);

    /* set up matrices so we can unstack homography into X; AX = B */
    tmatrix.clear();
    tmatrix.push_back(vector<double>(3, 0));
    tmatrix.push_back(vector<double>(3, 0));
    tmatrix.push_back(vector<double>(3, 0));
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
        err.push_back((double)(dx*dx + dy*dy));
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

double ImageStitching::siftDistance(const SiftDescriptor& x, const SiftDescriptor& y) {
    double dis = 0;
    for(int i = 0; i < 128; i++) {
        dis += pow((x.descriptor[i] - y.descriptor[i]), 2.0);
    }
    return sqrt(dis);
}

void ImageStitching::computeHT() {
    CImg<double> A(8, 8, 1, 1, 0), B(1, 8, 1, 1, 0), X(1, 8, 1, 1, 0);

    /* set up matrices so we can unstack homography into X; AX = B */
    HTmatrix.clear();
    HTmatrix.push_back(vector<double>(3, 0));
    HTmatrix.push_back(vector<double>(3, 0));
    HTmatrix.push_back(vector<double>(3, 0));

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
    
    int index = 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            HTmatrix[i][j] = X(0, index++);
            // cout << HTmatrix[i][j] << ", ";
        }
        // cout << endl;
    }
    HTmatrix[2][2] = 1.0;
}

void ImageStitching::imageFusion() {
    int dw = srcs[0].width(), dh = srcs[0].height();
    double w1 = (dw * Hmatrix[0][0] + 0  * Hmatrix[0][1] + 1 * Hmatrix[0][2])/(Hmatrix[2][0]*dw + Hmatrix[2][1]*0 + 1.);
    double w2 = (dw * Hmatrix[0][0] + dh * Hmatrix[0][1] + 1 * Hmatrix[0][2])/(Hmatrix[2][0]*dw + Hmatrix[2][1]*dh + 1.);
    while (w1 < srcs[1].width() && w2 < srcs[1].width()) {
        dw++;
        w1 = (dw * Hmatrix[0][0] + 0  * Hmatrix[0][1] + 1 * Hmatrix[0][2])/(Hmatrix[2][0]*dw + Hmatrix[2][1]*0  + 1.);
        w2 = (dw * Hmatrix[0][0] + dh * Hmatrix[0][1] + 1 * Hmatrix[0][2])/(Hmatrix[2][0]*dw + Hmatrix[2][1]*dh + 1.);
    }
    w1 = w1 > w2 ? w1 : w2;

    int tw = 0, th = srcs[0].height();
    double tw1 = (tw * Hmatrix[0][0] + 0  * Hmatrix[0][1] + 1 * Hmatrix[0][2])/(Hmatrix[2][0]*tw + Hmatrix[2][1]*0 + 1.);
    double tw2 = (tw * Hmatrix[0][0] + th * Hmatrix[0][1] + 1 * Hmatrix[0][2])/(Hmatrix[2][0]*tw + Hmatrix[2][1]*th + 1.);
    while (tw1 > 0 && tw2 > 0) {
        tw--;
        tw1 = (tw * Hmatrix[0][0] + 0  * Hmatrix[0][1] + 1 * Hmatrix[0][2])/(Hmatrix[2][0]*tw + Hmatrix[2][1]*0  + 1.);
        tw2 = (tw * Hmatrix[0][0] + th * Hmatrix[0][1] + 1 * Hmatrix[0][2])/(Hmatrix[2][0]*tw + Hmatrix[2][1]*th + 1.);
    }
    tw1 = tw1 < tw2 ? tw1 : tw2;


    int w = srcs[0].width() > dw  ? srcs[0].width()  : dw;
    w -= tw;
    int h = srcs[0].height();
    fusion = CImg<float>(w, h, 1, 3);

    cimg_forXY(srcs[0], x, y) {
        fusion(x-tw, y, 0) = srcs[0](x, y, 0);
        fusion(x-tw, y, 1) = srcs[0](x, y, 1);
        fusion(x-tw, y, 2) = srcs[0](x, y, 2);
    }

    int maxl = 0, maxr = 0;

    if (tw < 0) {
        for (int i = 0; i < fusion.width(); ++i) {
            double x1 = (i * Hmatrix[0][0] + srcs[0].height() * Hmatrix[0][1] + 1 * Hmatrix[0][2]) / (Hmatrix[2][0] * i + Hmatrix[2][1] * srcs[0].height() + 1.);
            double x2 = (i * Hmatrix[0][0] + 0 * Hmatrix[0][1] + 1 * Hmatrix[0][2]) / (Hmatrix[2][0] * i + Hmatrix[2][1] * 0 + 1.);
            if (x1 < srcs[1].width() && i > maxr) {
                maxr = i;
            }
            if (x2 < srcs[1].width() && i > maxr) {
                maxr = i;
            }
        }
    } else {
        for (int i = fusion.width(); i > 0; --i) {
            double x1 = (i * Hmatrix[0][0] + srcs[0].height() * Hmatrix[0][1] + 1 * Hmatrix[0][2]) / (Hmatrix[2][0] * i + Hmatrix[2][1] * srcs[0].height() + 1.);
            double x2 = (i * Hmatrix[0][0] + 0 * Hmatrix[0][1] + 1 * Hmatrix[0][2]) / (Hmatrix[2][0] * i + Hmatrix[2][1] * 0 + 1.);
            if (x1 > 0 && i < maxl) {
                maxl = i;
            }
            if (x2 > 0 && i < maxl) {
                maxl = i;
            }
        }
    }

    //cimg_forXY(fusion, x, y) {
    for (int x = 0+tw; x < fusion.width(); x++) {
        for (int y = 0; y < fusion.height(); y++) {
            double ox = (x * Hmatrix[0][0] + (y) * Hmatrix[0][1] + 1 * Hmatrix[0][2]) / (Hmatrix[2][0] * x + Hmatrix[2][1] * (y) + 1.);
            double oy = (x * Hmatrix[1][0] + (y) * Hmatrix[1][1] + 1 * Hmatrix[1][2]) / (Hmatrix[2][0] * x + Hmatrix[2][1] * (y) + 1.);
            if (ox > 0 && oy > 0 && ox < srcs[1].width() && oy < srcs[1].height()) {
                int px = floor(ox);
                int py = floor(oy);
                double a = ox - px;
                double b = oy - py;
                float f1 = (1-a)*(1-b)*srcs[1](px, py, 0) + (a)*(1-b)*srcs[1](1+px, py, 0) + 
                            (1-a)*(b)*srcs[1](px, 1+py, 0) + (a)*(b)*srcs[1](1+px, 1+py, 0);
                float f2 = (1-a)*(1-b)*srcs[1](px, py, 1) + (a)*(1-b)*srcs[1](1+px, py, 1) + 
                            (1-a)*(b)*srcs[1](px, 1+py, 1) + (a)*(b)*srcs[1](1+px, 1+py, 1);
                float f3 = (1-a)*(1-b)*srcs[1](px, py, 2) + (a)*(1-b)*srcs[1](1+px, py, 2) + 
                            (1-a)*(b)*srcs[1](px, 1+py, 2) + (a)*(b)*srcs[1](1+px, 1+py, 2);
                fusion(x-tw, y, 0) = f1!=0?f1:fusion(x-tw, y, 0);
                fusion(x-tw, y, 1) = f2!=0?f2:fusion(x-tw, y, 1);
                fusion(x-tw, y, 2) = f3!=0?f3:fusion(x-tw, y, 2);
                if (tw < 0) {
                    if (x > 0 && x < (maxr+tw)) {
                        double a = x/1.0/(maxr+tw);
                        fusion(x, y, 0) *= (1-a);
                        fusion(x, y, 1) *= (1-a);
                        fusion(x, y, 2) *= (1-a);
                        fusion(x, y, 0) += a * srcs[0](x, y, 0);
                        fusion(x, y, 1) += a * srcs[0](x, y, 1);
                        fusion(x, y, 2) += a * srcs[0](x, y, 2);
                    }
                } else {
                    if (x > maxl && x < srcs[0].width()) {
                        double a = (x-maxl)/1.0/(srcs[0].width()-maxl);
                        fusion(x, y, 0) *= (1-a);
                        fusion(x, y, 1) *= (1-a);
                        fusion(x, y, 2) *= (1-a);
                        fusion(x, y, 0) += a * srcs[0](x, y, 0);
                        fusion(x, y, 1) += a * srcs[0](x, y, 1);
                        fusion(x, y, 2) += a * srcs[0](x, y, 2);
                    }
                }
            }
        }
    }

    // cimg_forXY(srcs[1], x, y) {
    //     int ox = (x * Hmatrix[0][0] + y * Hmatrix[0][1] + 1 * Hmatrix[0][2]) / (Hmatrix[2][0] * x + Hmatrix[2][1] * y + 1.);
    //     int oy = (x * Hmatrix[1][0] + y * Hmatrix[1][1] + 1 * Hmatrix[1][2]) / (Hmatrix[2][0] * x + Hmatrix[2][1] * y + 1.);
    //     if (ox > 0 && ox < w && oy > 0 && oy < h) {
    //         if (FUSION_SWITCH || fusion(ox, oy, 0) == fusion(ox, oy, 1) && fusion(ox, oy, 0) == fusion(ox, oy, 2) && fusion(ox, oy, 0) == 0.) {
    //             fusion(ox, oy, 0) = srcs[1](x, y, 0);
    //             fusion(ox, oy, 1) = srcs[1](x, y, 1);
    //             fusion(ox, oy, 2) = srcs[1](x, y, 2);
    //         } else {
    //             fusion(ox, oy, 0) = 0.5 * srcs[1](x, y, 0) + 0.5 * fusion(ox, oy, 0);
    //             fusion(ox, oy, 1) = 0.5 * srcs[1](x, y, 1) + 0.5 * fusion(ox, oy, 1);
    //             fusion(ox, oy, 2) = 0.5 * srcs[1](x, y, 2) + 0.5 * fusion(ox, oy, 2);
    //         }
    //     }
    // }

    saveImg(fusion, "fusion_", times);
}

void ImageStitching::saveImg(const CImg<float>& img, const string& str, int i) {
    char s[100];
    sprintf(s,"./result/%s%d.bmp", str.c_str(), i);
    img.save(s);
}

void ImageStitching::over() {
    times++;
    if (times > total) {
        return;
    }
    readAgain();
    getGray();
    beginSIFT();
    drawSIFTpoint();
    run();
}

void ImageStitching::computeAllHomography() {
    sample1.clear();
    sample2.clear();
    int n = 0;
    for (int i = 0; i < maxInlinerIndex.size(); ++i) {
        if (maxInlinerIndex[i]) {
            sample1.push_back(&(match1[i]));
            sample2.push_back(&(match2[i]));
            n++;
        }
    }


    CImg<double> A(8, 2*n, 1, 1, 0), B(1, 2*n, 1, 1, 0), X(1, 8, 1, 1, 0);

    /* set up matrices so we can unstack homography into X; AX = B */
    Hmatrix.clear();
    Hmatrix.push_back(vector<double>(3, 0));
    Hmatrix.push_back(vector<double>(3, 0));
    Hmatrix.push_back(vector<double>(3, 0));
    for(int i = 0; i < 4; i++) {
        A(0, i  ) = sample2[i]->col;
        A(3, i+4) = sample2[i]->col;
        A(1, i  ) = sample2[i]->row;
        A(4, i+4) = sample2[i]->row;
        A(2, i  ) = 1.0;
        A(5, i+4) = 1.0;
        A(6, i  ) = -sample2[i]->col * sample1[i]->col;
        A(7, i  ) = -sample2[i]->row * sample1[i]->col;
        A(6, i+4) = -sample2[i]->col * sample1[i]->row;
        A(7, i+4) = -sample2[i]->row * sample1[i]->row;
        B(0, i  ) = sample1[i]->col;
        B(0, i+4) = sample1[i]->row;
    }
    X = B.solve(A);

    int index = 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Hmatrix[i][j] = X(0, index++);
            // cout << Hmatrix[i][j] << ", ";
        }
        // cout << endl;
    }
    Hmatrix[2][2] = 1.0;

    A = CImg<double>(8, 2*n, 1, 1, 0); B = CImg<double>(1, 2*n, 1, 1, 0); X = CImg<double>(1, 8, 1, 1, 0);

    /* set up matrices so we can unstack homography into X; AX = B */
    HTmatrix.clear();
    HTmatrix.push_back(vector<double>(3, 0));
    HTmatrix.push_back(vector<double>(3, 0));
    HTmatrix.push_back(vector<double>(3, 0));
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

    index = 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            HTmatrix[i][j] = X(0, index++);
            // cout << HTmatrix[i][j] << ", ";
        }
        // cout << endl;
    }
    HTmatrix[2][2] = 1.0;
}

void ImageStitching::readAgain() {
    srcs.clear();

    srcs.push_back(fusion);

    cout << "stitching image " << times << endl;

    string path1 = "./data/";
    path1 += '0' + times; path1 += ".bmp";

    srcs.push_back(CImg<float>(path1.c_str()));

    // fusion.display("f");
}

void ImageStitching::getGrayAgain() {
    grays.clear();

    CImg<float> temp2(srcs[0].width(), srcs[0].height(), 1, 1);

    double r,g,b;

    for(int i = 0; i < srcs[0].width(); i++) {
        for(int j = 0; j < srcs[0].height(); j++) {
            r = srcs[0](i, j, 0, 0);
            g = srcs[0](i, j, 0, 1);
            b = srcs[0](i, j, 0, 2);

            temp2(i, j, 0, 0) = 0.299*r + 0.587*g + 0.114*b;
        }
    }

    grays.push_back(temp2);
}

void ImageStitching::beginSIFTAgain() {

    vector<SiftDescriptor> temp = descriptors[1];
    // int w = srcs[0].width();
    // int h = srcs[0].height();

    // for (int i = 0; i < descriptors[1].size(); ++i) {
    //     int ox = descriptors[1][i].col * Hmatrix[0][0] + descriptors[1][i].row * Hmatrix[0][1] + 1 * Hmatrix[0][2];
    //     int oy = descriptors[1][i].col * Hmatrix[1][0] + descriptors[1][i].row * Hmatrix[1][1] + 1 * Hmatrix[1][2];
    //     if (ox > 0 && ox < w && oy > 0 && oy < h) {
    //         auto t = descriptors[1][i];
    //         t.col = ox;
    //         t.row = oy;
    //         temp.push_back(t);
    //     }
    // }

    descriptors.clear();
    marks.clear();

    descriptors.push_back(Sift::compute_sift(grays[0]));
    marks.push_back(srcs[0]);

    descriptors.push_back(temp);
    marks.push_back(srcs[1]);

}

void ImageStitching::drawSIFTpointAgain() {
    unsigned char red[3] = {255, 0, 0};
    for (int i = 0; i < marks.size(); ++i) {
        for (auto k : descriptors[i]) {
            marks[i].draw_circle(k.col, k.row, 3, red, 5.0f, 1);
        }
    }

    saveImg(marks[0], "marks1_", times);
    saveImg(marks[1], "marks2_", times);
}

void ImageStitching::refine(CImg<float>& img) {
    double k1 = -0.0000002;   // 形变参数，根据实际情况调整  
    double k2 = k1;
    
    CImg<float> temp(img.width(), img.height(), 1, 3);

    cimg_forXY(temp, x, y) {
        double ox = x - img.width()/2;
        double oy = y - img.height()/2;
        int x1 = ox * (1 + k1 * ox * ox + k2 * oy * oy);
        int y1 = oy * (1 + k1 * ox * ox + k2 * oy * oy);
        x1 += img.width()/2;
        y1 += img.height()/2;
        if (x1 > 0 && x1 < img.width() && y1 > 0 && y1 < img.height()) {
            temp(x, y, 0) = img(x1, y1, 0);
            temp(x, y, 1) = img(x1, y1, 1);
            temp(x, y, 2) = img(x1, y1, 2);
        } else {
            temp(x, y, 0) = 0;
            temp(x, y, 1) = 0;
            temp(x, y, 2) = 0;
        }
        
    }

    img = temp;
    
    // temp.display("refine");
}
