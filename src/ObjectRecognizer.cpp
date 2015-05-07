#include "ObjectRecognizer.h"
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <stdio.h>
#include <iostream>

ObjectRecognizer::ObjectRecognizer() {
    detector = cv::FeatureDetector::create("ORB");
    extractor = cv::DescriptorExtractor::create("ORB");
    //detector = cv::FeatureDetector::create("SURF");
    //extractor = cv::DescriptorExtractor::create("SURF");

    matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    //matcher = cv::DescriptorMatcher::create("FlannBased");
}

ObjectRecognizer::ObjectRecognizer(std::vector<std::string> calib_image_names) {
    // Setup feature detection/extraction/matching objects

    detector = cv::FeatureDetector::create("ORB");
    extractor = cv::DescriptorExtractor::create("ORB");
    //detector = cv::FeatureDetector::create("SURF");
    //extractor = cv::DescriptorExtractor::create("SURF");

    matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    //matcher = cv::DescriptorMatcher::create("FlannBased");

    for(int i = 0; i < calib_image_names.size(); i++) {
        // Read calibration image
        cv::Mat calib_img = cv::imread(calib_image_names[i], CV_LOAD_IMAGE_COLOR);
        
        cv::Mat_<unsigned char> calib_gray(calib_img.size());
        cv::cvtColor(calib_img, calib_gray, CV_BGR2GRAY);

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptor;

        detector->detect(calib_gray, keypoints);
        extractor->compute(calib_gray, keypoints, descriptor);

        obj_imgs.push_back(calib_img);
        obj_keypoints.push_back(keypoints);
        obj_descriptors.push_back(descriptor);        

        min_matches = 20; //obj_keypoints.size() / 3;
    }
}

float ObjectRecognizer::detect(cv::Mat color, std::vector<cv::Point2f>& obj_corners) {
    float score = 0;
    cv::Mat_<unsigned char> img(color.size());

    // Perform the Color->Gray conversion
    cv::cvtColor(color, img, CV_BGR2GRAY);

    // Compute keypoints and descriptors
    std::vector<cv::KeyPoint> img_keypoints;
    cv::Mat img_descriptors;

    std::vector<cv::DMatch> matches;

    detector->detect(img, img_keypoints);
    extractor->compute(img, img_keypoints, img_descriptors);

    float best_score = 0;

    if(img_descriptors.size() == cv::Size(0, 0)) return 0;
    
    for(int o = 0; o < obj_imgs.size(); o++) {
        // Match keypoints from current frame to calibration image
        matcher->match(obj_descriptors[o], img_descriptors, matches);

        double max_dist = 0; double min_dist = 100;

        // Calculate min and max distances between keypoints
        //for( int i = 0; i < obj_descriptors[o].rows; i++ )
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
            if( matches[i].distance < std::max(4*min_dist, 0.02)) {
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
            obj.push_back( obj_keypoints[o][ good_matches[i].queryIdx ].pt );
            scene.push_back( img_keypoints[ good_matches[i].trainIdx ].pt );
        }

        //std::cout << good_matches.size() << std::endl;

        if(good_matches.size() >= min_matches) {
            // Find homography from the calibration image to the current frame.
            cv::Mat H = cv::findHomography( obj, scene, CV_RANSAC );
            // Get the corners from the image_1 ( the object to be "detected" )
            obj_corners.reserve(4);
            obj_corners[0] = cvPoint(0,0);
            obj_corners[1] = cvPoint(obj_imgs[o].cols, 0 );
            obj_corners[2] = cvPoint(obj_imgs[o].cols, obj_imgs[o].rows );
            obj_corners[3] = cvPoint(0, obj_imgs[o].rows );


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

        best_score = std::max(best_score, score);
    }

    return best_score;
}
