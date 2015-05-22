#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <map>
#include <vector>

struct HSV
{
    int iLowH;
    int iHighH;

    int iLowS;
    int iHighS;

    int iLowV;
    int iHighV;

};

struct Object
{
    std::map<std::string,HSV> colorToHSV;
};

class ColorDetector
{
private:
    std::map<std::string, Object> objectHSVs;
    int total_success;

public:
    void Init(std::vector<std::string> objectNames);
    ColorDetector();
    cv::Mat detect(std::string objectName, cv::Mat src, std::string color);
    std::map<std::string, HSV> HSVGenerator(std::string objectName);
    std::map<std::string, Object> getObjectHSVs();
};
