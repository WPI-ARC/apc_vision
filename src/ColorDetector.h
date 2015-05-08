#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <map>
#include <vector>
using namespace cv;
using namespace std;

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
    map<string,HSV> colorToHSV;
};

class ColorDetector
{
private:
    map<string, Object> objectHSVs;
    int total_success;

public:
    ColorDetector(std::vector<std::string> objectNames);
    ColorDetector();
    bool detect(std::string objectName, Mat src, Mat contoursrc, string color);
    map<string, HSV> HSVGenerator(std::string objectName);
};