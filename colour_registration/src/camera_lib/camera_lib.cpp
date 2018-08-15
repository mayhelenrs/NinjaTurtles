#include <iostream>
#include <cstdlib>

// opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>


cv::Mat drawCrosshairs(cv::Mat input)
{

    cv::Scalar red(0,0,255);
    //crosshair horizontal
    cv::line(input, cv::Point((input.cols/2)-40, input.rows/2), 
    	cv::Point((input.cols/2)+40, input.rows/2), red, 1, cv::LINE_4,0);  

    //crosshair vertical
    cv::line(input, cv::Point(input.cols/2, (input.rows/2)-40), 
    	cv::Point(input.cols/2, (input.rows/2)+40), red, 1, cv::LINE_4,0);  

    return input;
}
