#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

class ObjectRecognizer {
private:
    // Feature detector
    cv::Ptr<cv::FeatureDetector> detector;
    // Feature descriptor extractor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    // Feature matcher
    cv::Ptr<cv::DescriptorMatcher> matcher;

    // Object calibration image
    std::vector<cv::Mat> obj_imgs;
    // Object feature keypoints
    std::vector<std::vector<cv::KeyPoint> > obj_keypoints;
    // Object feature descriptors
    std::vector<cv::Mat> obj_descriptors;

    int min_matches;
public:
    ObjectRecognizer();
    ObjectRecognizer(std::vector<std::string> calib_image_names);
    float detect(cv::Mat img);
};
