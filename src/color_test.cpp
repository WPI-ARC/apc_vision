#include "ColorDetector.h"
// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>


int main(int argc, char** argv) {
    if(argc <= 1) {
        std::cout << "Please specify image to test" << std::endl;
        return -1;
    }

    std::string object = argv[1];
    std::string color = argv[2];
    std::string file = argv[3];

    std::vector<std::string> object_names;
    object_names.push_back(object);

    ColorDetector detector;
    detector.Init(object_names);
    cv::Mat image = cv::imread(file);
    cv::imshow("image", image);
    cv::waitKey(0);
    cv::Mat mask = detector.detect(object, image, color);

    cv::imshow("mask", mask);
    cv::waitKey(0);
}
