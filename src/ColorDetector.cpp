#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ObjectIDs.h"
#include "ColorDetector.h"
#include <stdio.h>
#include <stdlib.h>
#include "ObjectIDs.h"

using namespace std;
using namespace cv;

void ColorDetector::Init(vector<string> objectNames)
{
    for(vector<string>::iterator objectIterator = objectNames.begin(); objectIterator!=objectNames.end(); objectIterator++)
    {
        Object object;
        object.colorToHSV = HSVGenerator(*objectIterator);
        objectHSVs[*objectIterator] = object;
    }
    total_success = 0;
}

ColorDetector::ColorDetector()
{}

map<string, Object> ColorDetector::getObjectHSVs() {
    return objectHSVs;
}

map<string, HSV> ColorDetector::HSVGenerator(string objectName) {
    // Maybe this should go in a config file?
    map<string,HSV> hsvMap;
    switch(string_to_id(objectName))
    {
    case CHEEZIT_BIG_ORIGINAL:

		HSV hsv_cheezit_orange;
		hsv_cheezit_orange.iLowH = 0;
		hsv_cheezit_orange.iHighH = 22;
		hsv_cheezit_orange.iLowS = 106;
		hsv_cheezit_orange.iHighS = 230;
		hsv_cheezit_orange.iLowV = 77;
		hsv_cheezit_orange.iHighV = 217;
		hsvMap["orange"] = hsv_cheezit_orange;

		HSV hsv_cheezit_yellow;
		hsv_cheezit_yellow.iLowH = 22;
		hsv_cheezit_yellow.iHighH = 40;
		hsv_cheezit_yellow.iLowS = 18;
		hsv_cheezit_yellow.iHighS = 255;
		hsv_cheezit_yellow.iLowV = 114;
		hsv_cheezit_yellow.iHighV = 255;
		hsvMap["yellow"] = hsv_cheezit_yellow;

		HSV hsv_cheezit_red;
		hsv_cheezit_red.iLowH = 0;
		hsv_cheezit_red.iHighH = 10;
		hsv_cheezit_red.iLowS = 38;
		hsv_cheezit_red.iHighS = 255;
		hsv_cheezit_red.iLowV = 15;
		hsv_cheezit_red.iHighV = 162;
		hsvMap["red"] = hsv_cheezit_red;

		break;

    case GENUINE_JOE_PLASTIC_STIR_STICKS:
		HSV hsv_stirsticks_red;
		hsv_stirsticks_red.iLowH = 0;
		hsv_stirsticks_red.iHighH = 10;
		hsv_stirsticks_red.iLowS = 38;
		hsv_stirsticks_red.iHighS = 255;
		hsv_stirsticks_red.iLowV = 15;
		hsv_stirsticks_red.iHighV = 162;
		hsvMap["red"] = hsv_stirsticks_red;

		HSV hsv_stirsticks_orange;
		hsv_stirsticks_orange.iLowH = 0;
		hsv_stirsticks_orange.iHighH = 22;
		hsv_stirsticks_orange.iLowS = 106;
		hsv_stirsticks_orange.iHighS = 230;
		hsv_stirsticks_orange.iLowV = 77;
		hsv_stirsticks_orange.iHighV = 217;
		hsvMap["orange"] = hsv_stirsticks_orange;

		break;

    case CRAYOLA_64_CT:
		HSV hsv_crayola_yellow;
		hsv_crayola_yellow.iLowH = 22;
		hsv_crayola_yellow.iHighH = 40;
		hsv_crayola_yellow.iLowS = 18;
		hsv_crayola_yellow.iHighS = 255;
		hsv_crayola_yellow.iLowV = 114;
		hsv_crayola_yellow.iHighV = 255;
		hsvMap["yellow"] = hsv_crayola_yellow;

		HSV hsv_crayola_green;
		hsv_crayola_green.iLowH = 37;
		hsv_crayola_green.iHighH = 75;
		hsv_crayola_green.iLowS = 25;
		hsv_crayola_green.iHighS = 255;
		hsv_crayola_green.iLowV = 0;
		hsv_crayola_green.iHighV = 255;
		hsvMap["green"] = hsv_crayola_green;

		HSV hsv_crayola_blue;
		hsv_crayola_blue.iLowH = 75;
		hsv_crayola_blue.iHighH = 130;
		hsv_crayola_blue.iLowS = 11;
		hsv_crayola_blue.iHighS = 151;
		hsv_crayola_blue.iLowV = 28;
		hsv_crayola_blue.iHighV = 218;
		hsvMap["blue"] = hsv_crayola_blue;

		break;

    case EXPO_DRY_ERASE_BOARD_ERASER:
		HSV hsv_expo_yellow;
		hsv_expo_yellow.iLowH = 22;
		hsv_expo_yellow.iHighH = 40;
		hsv_expo_yellow.iLowS = 18;
		hsv_expo_yellow.iHighS = 255;
		hsv_expo_yellow.iLowV = 114;
		hsv_expo_yellow.iHighV = 255;
		hsvMap["yellow"] = hsv_expo_yellow;

		HSV hsv_expo_green;
		hsv_expo_green.iLowH = 37;
		hsv_expo_green.iHighH = 75;
		hsv_expo_green.iLowS = 25;
		hsv_expo_green.iHighS = 255;
		hsv_expo_green.iLowV = 0;
		hsv_expo_green.iHighV = 255;
		hsvMap["green"] = hsv_expo_green;

		HSV hsv_expo_blue;
		hsv_expo_blue.iLowH = 75;
		hsv_expo_blue.iHighH = 130;
		hsv_expo_blue.iLowS = 11;
		hsv_expo_blue.iHighS = 151;
		hsv_expo_blue.iLowV = 28;
		hsv_expo_blue.iHighV = 218;
		hsvMap["blue"] = hsv_expo_blue;

		break;

    case ELMERS_WASHABLE_NO_RUN_SCHOOL_GLUE:
		HSV hsv_glue_blue;
		hsv_glue_blue.iLowH = 75;
		hsv_glue_blue.iHighH = 130;
		hsv_glue_blue.iLowS = 11;
		hsv_glue_blue.iHighS = 151;
		hsv_glue_blue.iLowV = 28;
		hsv_glue_blue.iHighV = 218;
		hsvMap["blue"] = hsv_glue_blue;

		HSV hsv_glue_orange;
		hsv_glue_orange.iLowH = 0;
		hsv_glue_orange.iHighH = 22;
		hsv_glue_orange.iLowS = 106;
		hsv_glue_orange.iHighS = 230;
		hsv_glue_orange.iLowV = 77;
		hsv_glue_orange.iHighV = 217;
		hsvMap["orange"] = hsv_glue_orange;

		HSV hsv_glue_red;
		hsv_glue_red.iLowH = 0;
		hsv_glue_red.iHighH = 10;
		hsv_glue_red.iLowS = 38;
		hsv_glue_red.iHighS = 255;
		hsv_glue_red.iLowV = 15;
		hsv_glue_red.iHighV = 162;
		hsvMap["red"] = hsv_glue_red;

		break;

    case KYJEN_SQUEAKIN_EGGS_PLUSH_PUPPIES:
		HSV hsv_eggs_blue;
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

    case OREO_MEGA_STUF:
		HSV hsv_oreo_blue;
		hsv_oreo_blue.iLowH = 75;
		hsv_oreo_blue.iHighH = 130;
		hsv_oreo_blue.iLowS = 11;
		hsv_oreo_blue.iHighS = 151;
		hsv_oreo_blue.iLowV = 28;
		hsv_oreo_blue.iHighV = 218;
		hsvMap["blue"] = hsv_oreo_blue;

		break;

    case CHAMPION_COPPER_PLUS_SPARK_PLUG:
		HSV hsv_spark_orange;
		hsv_spark_orange.iLowH = 0;
		hsv_spark_orange.iHighH = 22;
		hsv_spark_orange.iLowS = 106;
		hsv_spark_orange.iHighS = 230;
		hsv_spark_orange.iLowV = 77;
		hsv_spark_orange.iHighV = 217;
		hsvMap["orange"] = hsv_spark_orange;

		HSV hsv_spark_red;
		hsv_spark_red.iLowH = 0;
		hsv_spark_red.iHighH = 10;
		hsv_spark_red.iLowS = 38;
		hsv_spark_red.iHighS = 255;
		hsv_spark_red.iLowV = 15;
		hsv_spark_red.iHighV = 162;
		hsvMap["red"] = hsv_spark_red;

		break;

    case MUNCHKIN_WHITE_HOT_DUCK_BATH_TOY:
		HSV hsv_duck;
		hsv_duck.iLowH = 22;
		hsv_duck.iHighH = 38;
		hsv_duck.iLowS = 20;
		hsv_duck.iHighS = 234;
		hsv_duck.iLowV = 135;
		hsv_duck.iHighV = 206;
		hsvMap["yellow"] = hsv_duck;

		break;

    case SHARPIE_ACCENT_TANK_STYLE_HIGHLIGHTERS:
		HSV hsv_highlighters_blue;
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

    case STANLEY_66_052:
		HSV hsv_screwdrivers_yellow;
		hsv_screwdrivers_yellow.iLowH = 19;
		hsv_screwdrivers_yellow.iHighH = 37;
		hsv_screwdrivers_yellow.iLowS = 38;
		hsv_screwdrivers_yellow.iHighS = 255;
		hsv_screwdrivers_yellow.iLowV = 91;
		hsv_screwdrivers_yellow.iHighV = 255;
		hsvMap["yellow"] = hsv_screwdrivers_yellow;

		HSV hsv_screwdrivers_orange;
		hsv_screwdrivers_orange.iLowH = 0;
		hsv_screwdrivers_orange.iHighH = 22;
		hsv_screwdrivers_orange.iLowS = 106;
		hsv_screwdrivers_orange.iHighS = 230;
		hsv_screwdrivers_orange.iLowV = 77;
		hsv_screwdrivers_orange.iHighV = 217;
		hsvMap["orange"] = hsv_screwdrivers_orange;

		break;

    case SAFETY_WORKS_SAFETY_GLASSES:
		HSV hsv_safety_green;
		hsv_safety_green.iLowH = 43;
		hsv_safety_green.iHighH = 76;
		hsv_safety_green.iLowS = 20;
		hsv_safety_green.iHighS = 213;
		hsv_safety_green.iLowV = 50;
		hsv_safety_green.iHighV = 237;
		hsvMap["green"]  = hsv_safety_green;

		HSV hsv_safety_blue;
		hsv_safety_blue.iLowH = 82;
		hsv_safety_blue.iHighH = 112;
		hsv_safety_blue.iLowS = 68;
		hsv_safety_blue.iHighS = 244;
		hsv_safety_blue.iLowV = 78;
		hsv_safety_blue.iHighV = 255;
		hsvMap["blue"] = hsv_safety_blue;

		HSV hsv_safety_yellow;
		hsv_safety_yellow.iLowH = 19;
		hsv_safety_yellow.iHighH = 37;
		hsv_safety_yellow.iLowS = 38;
		hsv_safety_yellow.iHighS = 255;
		hsv_safety_yellow.iLowV = 91;
		hsv_safety_yellow.iHighV = 255;
		hsvMap["yellow"] = hsv_safety_yellow;

		break;

    case PAPER_MATE_12_COUNT_MIRADO_BLACK_WARRIOR:
		HSV hsv_papermate_blue;
		hsv_papermate_blue.iLowH = 82;
		hsv_papermate_blue.iHighH = 112;
		hsv_papermate_blue.iLowS = 68;
		hsv_papermate_blue.iHighS = 244;
		hsv_papermate_blue.iLowV = 78;
		hsv_papermate_blue.iHighV = 255;
		hsvMap["blue"] = hsv_papermate_blue;

		break;

    case MARK_TWAIN_HUCKLEBERRY_FINN:
		HSV hsv_mark_green;
		hsv_mark_green.iLowH = 43;
		hsv_mark_green.iHighH = 76;
		hsv_mark_green.iLowS = 20;
		hsv_mark_green.iHighS = 213;
		hsv_mark_green.iLowV = 50;
		hsv_mark_green.iHighV = 237;
		hsvMap["green"]  = hsv_mark_green;

		HSV hsv_mark_blue;
		hsv_mark_blue.iLowH = 82;
		hsv_mark_blue.iHighH = 112;
		hsv_mark_blue.iLowS = 68;
		hsv_mark_blue.iHighS = 244;
		hsv_mark_blue.iLowV = 78;
		hsv_mark_blue.iHighV = 255;
		hsvMap["blue"] = hsv_mark_blue;

		HSV hsv_mark_yellow;
		hsv_mark_yellow.iLowH = 19;
		hsv_mark_yellow.iHighH = 37;
		hsv_mark_yellow.iLowS = 38;
		hsv_mark_yellow.iHighS = 255;
		hsv_mark_yellow.iLowV = 91;
		hsv_mark_yellow.iHighV = 255;
		hsvMap["yellow"] = hsv_mark_yellow;

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

    case FELINE_GREENIES_DENTAL_TREATS:
		HSV hsv_feline_green;
		hsv_feline_green.iLowH = 43;
		hsv_feline_green.iHighH = 76;
		hsv_feline_green.iLowS = 20;
		hsv_feline_green.iHighS = 213;
		hsv_feline_green.iLowV = 50;
		hsv_feline_green.iHighV = 237;
		hsvMap["green"]  = hsv_feline_green;

		HSV hsv_feline_yellow;
		hsv_feline_yellow.iLowH = 19;
		hsv_feline_yellow.iHighH = 37;
		hsv_feline_yellow.iLowS = 38;
		hsv_feline_yellow.iHighS = 255;
		hsv_feline_yellow.iLowV = 91;
		hsv_feline_yellow.iHighV = 255;
		hsvMap["yellow"] = hsv_feline_yellow;

		HSV hsv_feline_orange;
		hsv_feline_orange.iLowH = 0;
		hsv_feline_orange.iHighH = 22;
		hsv_feline_orange.iLowS = 106;
		hsv_feline_orange.iHighS = 230;
		hsv_feline_orange.iLowV = 77;
		hsv_feline_orange.iHighV = 217;
		hsvMap["orange"] = hsv_feline_orange;

		break;

    case LAUGH_OUT_LOUD_JOKE_BOOK:
		HSV hsv_laugh_orange;
		hsv_laugh_orange.iLowH = 0;
		hsv_laugh_orange.iHighH = 22;
		hsv_laugh_orange.iLowS = 106;
		hsv_laugh_orange.iHighS = 230;
		hsv_laugh_orange.iLowV = 77;
		hsv_laugh_orange.iHighV = 217;
		hsvMap["orange"] = hsv_laugh_orange;

		HSV hsv_laugh_blue;
		hsv_laugh_blue.iLowH = 93;
		hsv_laugh_blue.iHighH = 114;
		hsv_laugh_blue.iLowS = 31;
		hsv_laugh_blue.iHighS = 178;
		hsv_laugh_blue.iLowV = 104;
		hsv_laugh_blue.iHighV = 255;
		hsvMap["blue"] = hsv_laugh_blue;

		HSV hsv_laugh_red;
		hsv_laugh_red.iLowH = 0;
		hsv_laugh_red.iHighH = 10;
		hsv_laugh_red.iLowS = 38;
		hsv_laugh_red.iHighS = 255;
		hsv_laugh_red.iLowV = 15;
		hsv_laugh_red.iHighV = 162;
		hsvMap["red"] = hsv_laugh_red;

		break;

    case KONG_AIR_DOG_SQUEAKAIR_TENNIS_BALL:
		HSV hsv_tennis_green;
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


        HSV hsv_tennis_orange;
        hsv_tennis_orange.iLowH = 0;
        hsv_tennis_orange.iHighH = 22;
        hsv_tennis_orange.iLowS = 106;
        hsv_tennis_orange.iHighS = 230;
        hsv_tennis_orange.iLowV = 77;
        hsv_tennis_orange.iHighV = 217;
		hsvMap["orange"] = hsv_tennis_orange;

		HSV hsv_tennis_yellow;
		hsv_tennis_yellow.iLowH = 19;
		hsv_tennis_yellow.iHighH = 37;
		hsv_tennis_yellow.iLowS = 38;
		hsv_tennis_yellow.iHighS = 255;
		hsv_tennis_yellow.iLowV = 91;
		hsv_tennis_yellow.iHighV = 255;
		hsvMap["yellow"] = hsv_tennis_yellow;

        break;

    case HIGHLAND_6539_SELF_STICK_NOTES:
		HSV hsv_highland_yellow;
		hsv_highland_yellow.iLowH = 19;
		hsv_highland_yellow.iHighH = 37;
		hsv_highland_yellow.iLowS = 38;
		hsv_highland_yellow.iHighS = 255;
		hsv_highland_yellow.iLowV = 91;
		hsv_highland_yellow.iHighV = 255;
		hsvMap["yellow"] = hsv_highland_yellow;

		break;

    case KONG_SITTING_FROG_DOG_TOY:
		HSV hsv_frog;
		hsv_frog.iLowH = 37;
		hsv_frog.iHighH = 75;
		hsv_frog.iLowS = 25;
		hsv_frog.iHighS = 255;
		hsv_frog.iLowV = 0;
		hsv_frog.iHighV = 255;
		hsvMap["green"] = hsv_frog;

		break;

    case KONG_DUCK_DOG_TOY:
		HSV hsv_duck_dog;
		hsv_duck_dog.iLowH = 17;
		hsv_duck_dog.iHighH = 29;
		hsv_duck_dog.iLowS = 38;
		hsv_duck_dog.iHighS = 213;
		hsv_duck_dog.iLowV = 135;
		hsv_duck_dog.iHighV = 230;
		hsvMap["yellow"] = hsv_duck_dog;

		break;

    case ROLODEX_JUMBO_PENCIL_CUP:
		HSV hsv_rolodex_blue;
		hsv_rolodex_blue.iLowH = 93;
		hsv_rolodex_blue.iHighH = 114;
		hsv_rolodex_blue.iLowS = 31;
		hsv_rolodex_blue.iHighS = 178;
		hsv_rolodex_blue.iLowV = 104;
		hsv_rolodex_blue.iHighV = 255;
		hsvMap["blue"] = hsv_rolodex_blue;

		break;

    case MEAD_INDEX_CARDS:
		HSV hsv_mead_orange;
		hsv_mead_orange.iLowH = 0;
		hsv_mead_orange.iHighH = 22;
		hsv_mead_orange.iLowS = 106;
		hsv_mead_orange.iHighS = 230;
		hsv_mead_orange.iLowV = 77;
		hsv_mead_orange.iHighV = 217;
		hsvMap["orange"] = hsv_mead_orange;

		HSV hsv_mead_red;
		hsv_mead_red.iLowH = 0;
		hsv_mead_red.iHighH = 10;
		hsv_mead_red.iLowS = 38;
		hsv_mead_red.iHighS = 255;
		hsv_mead_red.iLowV = 15;
		hsv_mead_red.iHighV = 162;
		hsvMap["red"] = hsv_mead_red;

		break;

    case DR_BROWNS_BOTTLE_BRUSH:
		HSV hsv_bottle_blue;
		hsv_bottle_blue.iLowH = 93;
		hsv_bottle_blue.iHighH = 114;
		hsv_bottle_blue.iLowS = 31;
		hsv_bottle_blue.iHighS = 178;
		hsv_bottle_blue.iLowV = 104;
		hsv_bottle_blue.iHighV = 255;
		hsvMap["blue"] = hsv_bottle_blue;

		HSV hsv_bottle_green;
		hsv_bottle_green.iLowH = 37;
		hsv_bottle_green.iHighH = 75;
		hsv_bottle_green.iLowS = 25;
		hsv_bottle_green.iHighS = 255;
		hsv_bottle_green.iLowV = 0;
		hsv_bottle_green.iHighV = 255;
		hsvMap["green"] = hsv_bottle_green;
		break;

    default:
        // Why is this here?
        HSV hsv;
        hsv.iLowH = 0;
        hsv.iHighH = 255;
        hsv.iLowS = 0;
        hsv.iHighS = 255;
        hsv.iLowV = 0;
        hsv.iHighV = 255;
        hsvMap[""] = hsv; // Why is this necessary?
        break;
    }
    return hsvMap;
}

cv::Mat ColorDetector::detect(string objectName, Mat src, string colorName) {
    if (!src.data) {
        cout << "No image data \n";
        return Mat_<uint8_t>::zeros(src.size());
    }

    HSV hsv = (objectHSVs[objectName]).colorToHSV[colorName];
    vector<vector<Point> > contours;

    Mat imgHSV;
    // Convert the captured frame from BGR to HSV
    cvtColor(src, imgHSV, COLOR_BGR2HSV);

    Mat imgThresholded;

    // Threshold image
    inRange(imgHSV, Scalar(hsv.iLowH, hsv.iLowS, hsv.iLowV), Scalar(hsv.iHighH, hsv.iHighS, hsv.iHighV), imgThresholded);
    // morphological opening (remove small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(6, 6)) ); //change both to size(6,6)
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(6, 6)) ); //get rid of the noise pixels
    // morphological closing (fill small holes in the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    //Find contours Points, (Imput Image, Storage, Mode1, Mode2, Offset??)
    findContours(imgThresholded,
    contours, // a vector of contours
    CV_RETR_EXTERNAL,// retrieve the external contours
    CV_CHAIN_APPROX_NONE,
    Point(0, 0)); // all pixels of each contours

    int largest_contour_index = 0;
    int largest_area = 0;

    // iterate through each contour.
    for(int i = 0; i < contours.size(); i++) {
        // Find the area of contour
        double a = contourArea(contours[i],false);
        if(a > largest_area) {
            largest_area = a;
            // Store the index of largest contour
            largest_contour_index=i;
        }
    }

    // Binary image representing the detected object
    Mat_<uint8_t> object = imgThresholded;
    //Draw the contour and rectangle
    if (largest_area > 1000) {
        Mat_<uint8_t> mask(object.size());
        drawContours(mask, contours, largest_contour_index, Scalar(255), -1);
        bitwise_and(object, mask, object);
    } else {
        object = cv::Mat_<uint8_t>::zeros(imgHSV.size());
    }

    return object;
}
