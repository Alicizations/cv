#include "ImageStitching.hpp"

int main() {
    ImageStitching *is = new ImageStitching();
    is->initial();
    is->run();
    delete is;
    return 0;
}