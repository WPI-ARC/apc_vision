#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ObjectIDs.h"
#include "ColorDetector.h"
#include <stdio.h>
#include <stdlib.h>
#include "ObjectIDs.h"

using namespace cv;
using namespace std;


void ColorDetector::Init(std::vector<std::string> objectNames)
{
    for(std::vector<std::string>::iterator objectIterator = objectNames.begin(); objectIterator!=objectNames.end(); objectIterator++)
    {
        Object object;
        object.colorToHSV = HSVGenerator(*objectIterator);
        objectHSVs[*objectIterator] = object;
    }
    total_success = 0;
}

ColorDetector::ColorDetector()
{}

map<string,HSV> ColorDetector::HSVGenerator(std::string objectName)
{
    map<string,HSV> hsvMap;
    switch(string_to_id(objectName))
    {
        case KONG_DUCK_DOG_TOY:   HSV hsv_duck_dog;
                                    hsv_duck_dog.iLowH = 17;
                                    hsv_duck_dog.iHighH = 29;

                                    hsv_duck_dog.iLowS = 38;
                                    hsv_duck_dog.iHighS = 213;

                                    hsv_duck_dog.iLowV = 135;
                                    hsv_duck_dog.iHighV = 230;
                                    hsvMap["yellow"] = hsv_duck_dog;
                                    break;
       case KONG_SITTING_FROG_DOG_TOY:    HSV hsv_frog;
                                            hsv_frog.iLowH = 37;
                                            hsv_frog.iHighH = 75;

                                            hsv_frog.iLowS = 25;
                                            hsv_frog.iHighS = 255;

                                            hsv_frog.iLowV = 0;
                                            hsv_frog.iHighV = 255;
                                            hsvMap["green"] = hsv_frog;
                                            break;
        case KYJEN_SQUEAKIN_EGGS_PLUSH_PUPPIES:    HSV hsv_eggs_blue;
                                                     hsv_eggs_blue.iLowH = 75;
                                                     hsv_eggs_blue.iHighH = 130;

                                                     hsv_eggs_blue.iLowS = 11;
                                                     hsv_eggs_blue.iHighS = 151;

                                                     hsv_eggs_blue.iLowV = 28;
                                                     hsv_eggs_blue.iHighV = 218;
                                                     hsvMap["blue"] = hsv_eggs_blue;

                                                     HSV hsv_eggs_orange;
                                                     hsv_eggs_orange.iLowH = 0;
                                                     hsv_eggs_orange.iHighH = 22;

                                                     hsv_eggs_orange.iLowS = 106;
                                                     hsv_eggs_orange.iHighS = 230;

                                                     hsv_eggs_orange.iLowV = 77;
                                                     hsv_eggs_orange.iHighV = 217;
                                                     hsvMap["orange"] = hsv_eggs_orange;

                                                     HSV hsv_eggs_yellow;
                                                     hsv_eggs_yellow.iLowH = 22;
                                                     hsv_eggs_yellow.iHighH = 40;

                                                     hsv_eggs_yellow.iLowS = 18;
                                                     hsv_eggs_yellow.iHighS = 255;

                                                     hsv_eggs_yellow.iLowV = 114;
                                                     hsv_eggs_yellow.iHighV = 255;
                                                     hsvMap["yellow"] = hsv_eggs_yellow;

                                                     break;


        case MUNCHKIN_WHITE_HOT_DUCK_BATH_TOY:    HSV hsv_duck;
                                                    hsv_duck.iLowH = 22;
                                                    hsv_duck.iHighH = 38;

                                                    hsv_duck.iLowS = 20;
                                                    hsv_duck.iHighS = 234;

                                                    hsv_duck.iLowV = 135;
                                                    hsv_duck.iHighV = 206;
                                                    hsvMap["yellow"] = hsv_duck;
                                                    break;
        case STANLEY_66_052:                      HSV hsv_screwdrivers;
                                                    hsv_screwdrivers.iLowH = 19;
                                                    hsv_screwdrivers.iHighH = 37;

                                                    hsv_screwdrivers.iLowS = 38;
                                                    hsv_screwdrivers.iHighS = 255;

                                                    hsv_screwdrivers.iLowV = 91;
                                                    hsv_screwdrivers.iHighV = 255;
                                                    hsvMap["yellow"] = hsv_screwdrivers;
                                                    break;
        case KONG_AIR_DOG_SQUEAKAIR_TENNIS_BALL:  HSV hsv_tennis_green;
                                                    hsv_tennis_green.iLowH = 28;
                                                    hsv_tennis_green.iHighH = 71;

                                                    hsv_tennis_green.iLowS = 36;
                                                    hsv_tennis_green.iHighS = 206;

                                                    hsv_tennis_green.iLowV = 74;
                                                    hsv_tennis_green.iHighV = 255;
                                                    hsvMap["green"] = hsv_tennis_green;

                                                    HSV hsv_tennis_red;
                                                    hsv_tennis_red.iLowH = 0;
                                                    hsv_tennis_red.iHighH = 10;

                                                    hsv_tennis_red.iLowS = 38;
                                                    hsv_tennis_red.iHighS = 255;

                                                    hsv_tennis_red.iLowV = 15;
                                                    hsv_tennis_red.iHighV = 162;
                                                    hsvMap["red"] = hsv_tennis_red;

                                                    break;
        case FIRST_YEARS_TAKE_AND_TOSS_STRAW_CUP:
                                                    HSV hsv_straw_cups_blue;
                                                    hsv_straw_cups_blue.iLowH = 93;
                                                    hsv_straw_cups_blue.iHighH = 114;

                                                    hsv_straw_cups_blue.iLowS = 31;
                                                    hsv_straw_cups_blue.iHighS = 178;

                                                    hsv_straw_cups_blue.iLowV = 104;
                                                    hsv_straw_cups_blue.iHighV = 255;
                                                    hsvMap["blue"] = hsv_straw_cups_blue;
                                                    break;
        case CRAYOLA_64_CT:                       HSV hsv_crayola_yellow;
                                                    hsv_crayola_yellow.iLowH = 18;
                                                    hsv_crayola_yellow.iHighH = 30;

                                                    hsv_crayola_yellow.iLowS = 134;
                                                    hsv_crayola_yellow.iHighS = 255;

                                                    hsv_crayola_yellow.iLowV = 80;
                                                    hsv_crayola_yellow.iHighV = 184;
                                                    hsvMap["yellow"] = hsv_crayola_yellow;

                                                    HSV hsv_straw_cups_green;
                                                    hsv_straw_cups_green.iLowH = 25;
                                                    hsv_straw_cups_green.iHighH = 81;

                                                    hsv_straw_cups_green.iLowS = 103;
                                                    hsv_straw_cups_green.iHighS = 255;

                                                    hsv_straw_cups_green.iLowV = 35;
                                                    hsv_straw_cups_green.iHighV = 148;
                                                    hsvMap["blue"] = hsv_straw_cups_green;

                                                    break;
        case SHARPIE_ACCENT_TANK_STYLE_HIGHLIGHTERS:  HSV hsv_highlighters_blue;
                                                        hsv_highlighters_blue.iLowH = 82;
                                                        hsv_highlighters_blue.iHighH = 112;

                                                        hsv_highlighters_blue.iLowS = 68;
                                                        hsv_highlighters_blue.iHighS = 244;

                                                        hsv_highlighters_blue.iLowV = 78;
                                                        hsv_highlighters_blue.iHighV = 255;
                                                        hsvMap["blue"] = hsv_highlighters_blue;

                                                        HSV hsv_highlighters_yellow;
                                                        hsv_highlighters_yellow.iLowH = 28;
                                                        hsv_highlighters_yellow.iHighH = 48;

                                                        hsv_highlighters_yellow.iLowS = 56;
                                                        hsv_highlighters_yellow.iHighS = 234;

                                                        hsv_highlighters_yellow.iLowV = 102;
                                                        hsv_highlighters_yellow.iHighV = 222;

                                                        hsvMap["yellow"] = hsv_highlighters_yellow;
                                                        HSV hsv_highlighters_orange;

                                                        hsv_highlighters_orange.iLowH = 0;
                                                        hsv_highlighters_orange.iHighH = 23;

                                                        hsv_highlighters_orange.iLowS = 93;
                                                        hsv_highlighters_orange.iHighS = 255;

                                                        hsv_highlighters_orange.iLowV = 82;
                                                        hsv_highlighters_orange.iHighV = 237;
                                                        hsvMap["orange"] = hsv_highlighters_orange;

                                                        HSV hsv_highlighters_green;
                                                        hsv_highlighters_green.iLowH = 43;
                                                        hsv_highlighters_green.iHighH = 76;

                                                        hsv_highlighters_green.iLowS = 20;
                                                        hsv_highlighters_green.iHighS = 213;

                                                        hsv_highlighters_green.iLowV = 50;
                                                        hsv_highlighters_green.iHighV = 237;
                                                        hsvMap["green"]  = hsv_highlighters_green;

                                                    break;
        default:    HSV hsv;
                    hsv.iLowH = 0;
                    hsv.iHighH = 255;

                    hsv.iLowS = 0;
                    hsv.iHighS = 255;

                    hsv.iLowV = 0;
                    hsv.iHighV = 255;
                    hsvMap[""] = hsv;
                break;
    }
    return hsvMap;
}

//int main( int argc, char** argv )
//{
//    cout <<"Running algorithm..."<<endl;
//    for(int i=1; i<=600; i++)
//    {
//        //cout << "currently at image: " << i << endl;
//        string selectedcolor = "";
//        stringstream ss;
//        ss << i;
//        string imgnum = ss.str();

//        //Select which HSV set to use for thresholding
//        int select = 16;
//        if (select == 1) //plush yellow duck
//        {
//            iLowH = 17;
//            iHighH = 29;

//            iLowS = 38;
//            iHighS = 213;

//            iLowV = 135;
//            iHighV = 230;

//            selectedcolor = "Yellow";
//            image_path = "../apc_images/plush_duck/plush_duck_(" + imgnum + ").jpg";
//        }
//        else if (select == 2) //plush green frog
//        {
//            iLowH = 37;
//            iHighH = 75;

//            iLowS = 25;
//            iHighS = 255;

//            iLowV = 0;
//            iHighV = 255;

//            selectedcolor = "Green";
//            image_path = "../apc_images/plush_frog/plush_frog_(" + imgnum + ").jpg";
//        }
//        else if (select == 3) //color eggs blue
//        {
//            iLowH = 75;
//            iHighH = 130;

//            iLowS = 11;
//            iHighS = 151;

//            iLowV = 28;
//            iHighV = 218;

//            selectedcolor = "blue";
//            image_path = "../apc_images/color_balls/colorballs_(" + imgnum + ").jpg";
//        }
//        else if (select == 4) //color eggs orange
//        {
//            iLowH = 0;
//            iHighH = 22;

//            iLowS = 106;
//            iHighS = 230;

//            iLowV = 77;
//            iHighV = 217;

//            selectedcolor = "orange";
//            image_path = "../apc_images/color_balls/colorballs_(" + imgnum + ").jpg";
//        }
//        else if (select == 5) //color eggs yellow
//        {
//            iLowH = 22;
//            iHighH = 40;

//            iLowS = 18;
//            iHighS = 255;

//            iLowV = 114;
//            iHighV = 255;

//            selectedcolor = "yellow";
//            image_path = "../apc_images/color_balls/colorballs_(" + imgnum + ").jpg";
//        }
//        else if (select == 6) //rubber duck
//        {
//            iLowH = 22;
//            iHighH = 38;

//            iLowS = 20;
//            iHighS = 234;

//            iLowV = 135;
//            iHighV = 206;

//            selectedcolor = "Yellow";
//            image_path = "../apc_images/rubber_duck/rubber_duck_(" + imgnum + ").jpg";
//        }
//        else if (select == 7) //screwdrivers
//        {
//            iLowH = 19;
//            iHighH = 37;

//            iLowS = 38;
//            iHighS = 255;

//            iLowV = 91;
//            iHighV = 255;

//            selectedcolor = "Yellow";
//            image_path = "../apc_images/screwdrivers/screwdrivers_(" + imgnum + ").jpg";
//        }
//        else if (select == 8) //tennis ball
//        {
//            iLowH = 28;
//            iHighH = 71;

//            iLowS = 36;
//            iHighS = 206;

//            iLowV = 74;
//            iHighV = 255;

//            selectedcolor = "green";
//            image_path = "../apc_images/tennis_ball/tennisball_(" + imgnum + ").jpg";
//        }
//        else if (select == 9) //tennis ball
//        {
//            iLowH = 0;
//            iHighH = 10;

//            iLowS = 38;
//            iHighS = 255;

//            iLowV = 15;
//            iHighV = 162;

//            selectedcolor = "red";
//            image_path = "../apc_images/tennis_ball/tennisball_(" + imgnum + ").jpg";
//        }
//        else if (select == 10) //cups
//        {
//            iLowH = 93;
//            iHighH = 114;

//            iLowS = 31;
//            iHighS = 178;

//            iLowV = 104;
//            iHighV = 255;

//            selectedcolor = "Blue";
//            image_path = "../apc_images/cups/cups_(" + imgnum + ").jpg";
//        }
//        else if (select == 11) //crayon yellow
//        {
//            iLowH = 18;
//            iHighH = 30;

//            iLowS = 134;
//            iHighS = 255;

//            iLowV = 80;
//            iHighV = 184;

//            selectedcolor = "yellow";
//            image_path = "../apc_images/crayon/crayon_(" + imgnum + ").jpg";
//        }
//        else if (select == 12) //crayon green
//        {
//            iLowH = 25;
//            iHighH = 81;

//            iLowS = 103;
//            iHighS = 255;

//            iLowV = 35;
//            iHighV = 148;

//            selectedcolor = "Blue";
//            image_path = "../apc_images/crayon/crayon_(" + imgnum + ").jpg";
//        }
//        else if (select == 13) //highlighter yellow
//        {
//            iLowH = 28;
//            iHighH = 48;

//            iLowS = 56;
//            iHighS = 234;

//            iLowV = 102;
//            iHighV = 222;

//            selectedcolor = "yellow";
//            image_path = "../apc_images/highlighters/highlighters_(" + imgnum + ").jpg";
//        }
//        else if (select == 14) //highlighter blue
//        {
//            iLowH = 82;
//            iHighH = 112;

//            iLowS = 68;
//            iHighS = 244;

//            iLowV = 78;
//            iHighV = 255;

//            selectedcolor = "blue";
//            image_path = "../apc_images/highlighters/highlighters_(" + imgnum + ").jpg";
//        }
//        else if (select == 15) //highlighter orange
//        {
//            iLowH = 0;
//            iHighH = 23;

//            iLowS = 93;
//            iHighS = 255;

//            iLowV = 82;
//            iHighV = 237;

//            selectedcolor = "orange";
//            image_path = "../apc_images/highlighters/highlighters_(" + imgnum + ").jpg";
//        }
//        else if (select == 16) //highlighter green
//        {
//            iLowH = 43;
//            iHighH = 76;

//            iLowS = 20;
//            iHighS = 213;

//            iLowV = 50;
//            iHighV = 237;

//            selectedcolor = "green";
//            image_path = "../apc_images/highlighters/highlighters_(" + imgnum + ").jpg";
//        }
//        else
//        {
//            iLowH = 0;
//            iHighH = 255;

//            iLowS = 0;
//            iHighS = 255;

//            iLowV = 0;
//            iHighV = 255;
//            image_path = "";
//        }


bool ColorDetector::detect(std::string objectName, Mat src, Mat contoursrc, string colorName)
{
        if ( !src.data )
        {
            cout << "No image data \n";
            return -1;
        }
        HSV hsv = (objectHSVs[objectName]).colorToHSV[colorName];

        //contour
        vector< vector<Point> > contours;

        Mat imgHSV;

        cvtColor(src, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        Mat imgThresholded;

        inRange(imgHSV, Scalar(hsv.iLowH, hsv.iLowS, hsv.iLowV), Scalar(hsv.iHighH, hsv.iHighS, hsv.iHighV), imgThresholded); //Threshold the image

        //morphological opening (remove small objects from the foreground)
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(6, 6)) ); //change both to size(6,6)
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(6, 6)) ); //get rid of the noise pixels

        //morphological closing (fill small holes in the foreground)
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
/*
        imshow("Thresholded Image", imgThresholded); //show the thresholded image
        imshow("Original", src); //show the original image
*/
        //Find contours Points, (Imput Image, Storage, Mode1, Mode2, Offset??)
        findContours(imgThresholded,
        contours, // a vector of contours
        CV_RETR_EXTERNAL,// retrieve the external contours
        CV_CHAIN_APPROX_NONE,
        Point(0, 0)); // all pixels of each contours

    //new code here------>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        int largest_contour_index=0;
        Rect bounding_rect;
        int largest_area=0;
        vector<Vec4i> hierarchy;


        // iterate through each contour.
        for( int i = 0; i< contours.size(); i++ )
        {
            //  Find the area of contour
            double a=contourArea( contours[i],false);
            if(a>largest_area){
                largest_area=a;
                //cout<<i<<" area  "<<a<<endl;
                // Store the index of largest contour
                largest_contour_index=i;
                // Find the bounding rectangle for biggest contour
                bounding_rect=boundingRect(contours[i]);
            }
        }

        Scalar color(255,255,255);
        //Draw the contour and rectangle
        //drawContours( src, contours,largest_contour_index, color, CV_FILLED,8,hierarchy);
        if (largest_area > 1000)
            {
            rectangle(contoursrc, bounding_rect,  Scalar(0,255,0),2, 8,0);
            drawContours( contoursrc, contours,largest_contour_index, color, CV_FILLED,8,hierarchy);
            rectangle(src, bounding_rect,  Scalar(0,255,0),2, 8,0);
//            ROS_INFO("Color %s successfully detected", colorName);
            //cout<<"Detected color is: "<<selectedcolor<<endl;
            total_success++;
            return true;
            }
        else
        {
//            ROS_INFO("NO DETECTION of Selected Color %s", colorName);
//            cout<<"Didn't detect: "<<selectedcolor<<endl;
//            cout<< image_path<<endl;
            return false;
        }

        namedWindow( "Bounding Box", CV_WINDOW_AUTOSIZE );
        imshow( "Bounding Box", src );
        waitKey(1);
        namedWindow( "Draw contour", CV_WINDOW_AUTOSIZE );
        imshow( "Draw contour", contoursrc );
        waitKey(1);

}


/*
    while(true)
    {
        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }
    }
*/

