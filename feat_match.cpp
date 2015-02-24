#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <math.h>
#include <iostream>

class ObjDetector {
private:
    // Feature detector
    cv::Ptr<cv::FeatureDetector> detector;
    // Feature descriptor extractor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    // Feature matcher
    cv::Ptr<cv::DescriptorMatcher> matcher;

    // Object calibration image
    cv::Mat obj_img;
    // Object feature keypoints
    std::vector<cv::KeyPoint> obj_keypoints;
    // Object feature descriptors
    cv::Mat obj_descriptors;

    int min_matches;

public:
    ObjDetector(std::string calib_image_name)
    {
        // Setup feature detection/extraction/matching objects
        //detector = cv::FeatureDetector::create("ORB");
        //extractor = cv::DescriptorExtractor::create("ORB");
        detector = cv::FeatureDetector::create("SURF");
        extractor = cv::DescriptorExtractor::create("SURF");

        //matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
        matcher = cv::DescriptorMatcher::create("FlannBased");

        // Read calibration image
        obj_img = cv::imread(calib_image_name, CV_LOAD_IMAGE_COLOR);
        cv::Mat_<unsigned char> obj_gray(obj_img.size());
        cv::cvtColor(obj_img, obj_gray, CV_BGR2GRAY);

        detector->detect(obj_gray, obj_keypoints);
        extractor->compute(obj_gray, obj_keypoints, obj_descriptors);

        min_matches = 20; //obj_keypoints.size() / 3;

        cv::drawKeypoints(obj_gray, obj_keypoints, obj_img);
    }

    bool find_object(cv::Mat color) {
        float score = 0;

        cv::Mat img;

        // Perform the Color->Gray conversion
        cv::cvtColor(color, img, CV_BGR2GRAY);

        // Compute keypoints and descriptors
        std::vector<cv::KeyPoint> img_keypoints;
        cv::Mat img_descriptors;

        std::vector<cv::DMatch> matches;

        try {
            detector->detect(img, img_keypoints);
            extractor->compute(img, img_keypoints, img_descriptors);
            // Match keypoints from current frame to calibration image
            matcher->match(obj_descriptors, img_descriptors, matches);
        } catch(...) {

        }
        if(!matches.size()) {
            cv::imshow("OUT", color);
            cv::waitKey(1);
            return 0;
        }

        double max_dist = 0; double min_dist = 100;

        // Calculate min and max distances between keypoints
        //for( int i = 0; i < obj_descriptors.rows; i++ )
        for( int i = 0; i < matches.size(); i++ )
        {
            double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }

        // The determination of what a "good" match is should probably
        // be revised.
        std::vector<cv::DMatch> good_matches;

        for( int i = 0; i < matches.size(); i++ )
        {
            if( matches[i].distance < std::max(3*min_dist, 0.02)) {
                good_matches.push_back( matches[i]);
                score += matches[i].distance;
            }
        }

        score /= good_matches.size();

        std::vector<cv::Point2f> obj;
        std::vector<cv::Point2f> scene;

        for( int i = 0; i < good_matches.size(); i++ )
        {
            // Get the keypoints from the good matches
            obj.push_back( obj_keypoints[ good_matches[i].queryIdx ].pt );
            scene.push_back( img_keypoints[ good_matches[i].trainIdx ].pt );
        }

        if(good_matches.size() >= min_matches) {
            // Find homography from the calibration image to the current frame.
            cv::Mat H = cv::findHomography( obj, scene, CV_RANSAC );
            // Get the corners from the image_1 ( the object to be "detected" )
            std::vector<cv::Point2f> obj_corners(4);
            obj_corners[0] = cvPoint(0,0);
            obj_corners[1] = cvPoint(obj_img.cols, 0 );
            obj_corners[2] = cvPoint(obj_img.cols, obj_img.rows );
            obj_corners[3] = cvPoint(0, obj_img.rows );


            std::vector<cv::Point2f> scene_corners(4);

            // Transform object corners from calibration image to current frame
            perspectiveTransform( obj_corners, scene_corners, H);

            cv::Scalar drawColor(0, 255, 0);
            if(cv::isContourConvex(scene_corners)) // && cv::arcLength(scene_corners, true)
            {
                // Draw lines around object
                cv::line( color, scene_corners[0], scene_corners[1], drawColor, 4 );
                cv::line( color, scene_corners[1], scene_corners[2], drawColor, 4 );
                cv::line( color, scene_corners[2], scene_corners[3], drawColor, 4 );
                cv::line( color, scene_corners[3], scene_corners[0], drawColor, 4 );
            } else { 
                score = 0;
            }
        } else {
            score = 0;
        }

        return score;
    }
};

int main(int argc, char** argv) {
    if(argc < 2) {
        std::cout << "Needs at least 1 argument" << std::endl;
        return -1;
    }

    std::string images_folder(argv[1]);

    cv::FileStorage config("config.yaml", cv::FileStorage::READ);

    cv::FileNode objs = config["objects"];
    if(objs.type() != cv::FileNode::SEQ) {
        std::cout << "'objects' in config file not a sequence" << std::endl;
        return -1;
    }

    std::vector<std::string> object_list;
    std::vector<int> num_calib_list;

    for(cv::FileNodeIterator it = objs.begin(); it != objs.end(); ++it) {
        object_list.push_back((std::string)(*it)["name"]);
        num_calib_list.push_back((int)(*it)["num_calib"]);
    }

    cv::Mat_<double> confusion_matrix = cv::Mat_<double>::zeros(object_list.size(), object_list.size());
    std::vector<double> precision(object_list.size());
    std::vector<double> recall(object_list.size());

    for(int i = 0; i < object_list.size(); ++i) {
        int num_calib = num_calib_list[i];

        // Shortcut to avoid opening files when there are no calibration
        // images to compare to.
        if(num_calib == 0) {
            recall[i] = 0;
            precision[i] = 0;
            for(int j = 0; j < object_list.size(); ++j) {
                printf("%s%.2f%%", j==0?"":",\t", 0.0);
            }
            printf(",\t\t\t%.2f%%,\t%.2f%%,\n", recall[i], precision[i]);
            continue;
        }

        std::vector<ObjDetector> detectors;

        for(int j = 0; j < num_calib; ++j) {
            std::stringstream ss;
            ss << (images_folder+"/"+object_list[i]) << "/calib" << j << ".jpeg";
            try {
                detectors.push_back(ObjDetector(ss.str()));
            } catch(cv::Exception e) {
                std::cout << "Error creating object detector: " << e.what() << std::endl;
                return -1;
            }
        }

        int precision_total_imgs = 0;
        int precision_correct_detects = 0;

        for(int j = 0; j < object_list.size(); ++j) {
            std::string obj_folder = images_folder + "/" + object_list[j];
            int num_detected = 0;
            int total = 0;
            for(int cam = 1; cam <= 5; cam++) {
                for(int angle = 0; angle < 360; angle += ((i==j) ? 15 : 90)) {
                    std::stringstream s;
                    s << "NP" << cam << "_" << angle << ".jpg";
                    //s << "N" << cam << "_" << angle << ".jpg";
                    std::string filename = obj_folder + "/" + s.str();
                    cv::Mat img;
                    try {
                        img = cv::imread(filename);
                    } catch(std::exception e) {
                        std::cout << "Error opening file '" << filename << "': " << e.what() << std::endl;
                        continue;
                    }
                    bool detected = false;
                    for(int k = 0; k < num_calib; k++) {
                        if(detectors[k].find_object(img) > 0) detected = true;
                    }

                    if(detected) ++num_detected;
                    ++total;

                }
            }

            double percent_detected = (double)num_detected/(double)total;

            confusion_matrix(i, j) = percent_detected;
            if(i == j) recall[i] = percent_detected;
            precision_total_imgs += total;
            precision_correct_detects += (i==j) ? num_detected : total - num_detected;
            printf("%s%.2f%%", j==0?"":",\t", percent_detected);
        }

        precision[i] = (double)precision_correct_detects/(double)precision_total_imgs;

        printf(",\t\t\t%.2f%%,\t%.2f%%,\n", recall[i], precision[i]);
    }

    if(argc >= 3) {
        cv::FileStorage output(argv[2], cv::FileStorage::WRITE);
        output << "confusion_matrix" << confusion_matrix;
    }

    return 0;
}
