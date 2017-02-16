#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <boost/timer/timer.hpp>

#include "include/image.h"
#include "include/lines.h"
#include "include/contours.h"

using namespace cv;
using namespace std;

Mat src, src_gray, templ;
Mat result;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

/// Function header
void thresh_callback(int, void* );

/** @function main */
int main( int argc, char** argv )
{
    // tuple ranges
    // int_tuple tpl_min_hsv_field, tpl_max_hsv_field;
    // tpl_min_hsv_field = boost::make_tuple(70, 101, 0);
    // tpl_max_hsv_field = boost::make_tuple(101, 255, 255);

    cv::VideoCapture cap(argv[1]);
    cv::Mat frame, frame_hsv, frame_hsv_in, frame_hsv_field_in;
    cv::Mat frame_gray, frame_lines, frame_infield, image_roi, frame_canny;
    double computation_time =0.0;
    int frame_number = 0;
    // vector<cv::Vec2f> lines;
    int roi_x, roi_y, roi_width, roi_height;
    // vector<Vec4i> lines;
    /// Create the result matrix
    
    roi_x = 1;
    roi_y = 240;
    roi_width = IMG_WIDTH-1;
    roi_height = 100;

    templ = imread(argv[2], 1);
    int result_cols =  roi_width - templ.cols + 1;
    int result_rows = roi_height - templ.rows + 1;

    result.create( result_rows, result_cols, CV_32FC1 );

    //(235,248,192,43)
    cv::Rect region_of_interest = cv::Rect(roi_x,
                                           roi_y,
                                           roi_width,
                                           roi_height);
    while(cap.read(frame))
    {
        double timer = (double)getTickCount();

        if(frame.size() != Size(640,480))
          cv::resize(frame, frame, cv::Size(640,480));

        image_roi = frame(region_of_interest);

        // image::convert(image_roi, frame_gray, CV_BGR2GRAY);
        //image::convert(image_roi, frame_hsv, CV_BGR2HSV);
        //image::hsv(frame_hsv, frame_hsv_field_in, tpl_min_hsv_field, tpl_max_hsv_field);
       
        // image::threshold(frame_gray, frame_lines, 170, 255, cv::THRESH_TOZERO);

        //frame_hsv_field_in and img_hsv_field_out_
        // image::morphClosing(frame_hsv_field_in, image::ELLIPSE, 15);
        // image::morphOpening(frame_hsv_field_in, image::ELLIPSE, 15);
        // image::negate(this->frame_hsv_field_in, this->img_hsv_field_out_);

        // image::intersection(frame_hsv_field_in, frame_lines, frame_infield);
        // image::canny(frame_infield, frame_canny, 200, 400);
        
        // lines::applyHough(frame_lines, lines, ROI_WIDTH/1.5);

        // imwrite("final.jpg", frame_lines);

        /// Do the Matching and Normalize
        // matchTemplate( frame_lines, templ, result, CV_TM_SQDIFF );
        matchTemplate( image_roi, templ, result, CV_TM_SQDIFF );
        normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

        /// Localizing the best match with minMaxLoc
        double minVal; double maxVal; Point minLoc; Point maxLoc; Point final_rect;
        Point matchLoc;

        minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
        matchLoc = minLoc;

        matchLoc.x += roi_x; matchLoc.y += roi_y;
        final_rect.x = matchLoc.x - 4;
        final_rect.y = matchLoc.y - 50;

        cv::rectangle( frame, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
        cv::rectangle( frame, final_rect, Point(final_rect.x + 33, final_rect.y + 38 ), Scalar(0,255,0));

        // for(size_t i=0; i<lines.size();i++)
          // lines::show(frame_lines, lines[i], Scalar(0), 1);

        // HoughLinesP(frame_infield, lines, 1, CV_PI/180, 50, 30, 10 );

        // for( size_t i = 0; i < lines.size(); i++ )
        // {
          // Vec4i l = lines[i];
          // line( frame_infield, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
        // }

        cv::rectangle(frame,region_of_interest,cv::Scalar(0,0,255));
        imshow("original", frame);
        // imshow("lines", frame_lines);
        // imshow("infield", frame_infield);

        timer = ((double)getTickCount() - timer)/getTickFrequency();
        std::cout << "wallclock_time: " << timer << std::endl;
        computation_time += timer;
        frame_number+=1;

        char key = waitKey(100);
        if(key==27) return 1;
        if(key=='p')
        {
            imwrite("saved.jpg", frame);
            waitKey(0);
        } 
    }
    return 0;
}


      // /// Load source image and convert it to gray
      // src = imread( argv[1], 1 );

      // /// Convert image to gray and blur it
      // cvtColor( src, src_gray, CV_BGR2GRAY );
      // blur( src_gray, src_gray, Size(3,3) );

      // /// Create Window
      // // char* source_window = "Source";
      // // namedWindow( "Source", CV_WINDOW_AUTOSIZE );
      // imshow( "source", src );

      // createTrackbar( " Threshold:", "Source", &thresh, max_thresh, thresh_callback );
      // thresh_callback( 0, 0 );

      // waitKey(0);
      

/** @function thresh_callback */
void thresh_callback(int, void* )
{
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Detect edges using Threshold
    threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );
    /// Find contours
    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );

    for( int i = 0; i < contours.size(); i++ )
       { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
         boundRect[i] = boundingRect( Mat(contours_poly[i]) );
         minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
       }


    /// Draw polygonal contour + bonding rects + circles
    Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
       {
         Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
         drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
         rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
         circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
       }

    /// Show in a window
    // namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    imshow( "source", drawing );
}