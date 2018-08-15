// peter kydd pkydd@cse.unsw.edu.au
// program to pull video image data from generic USB camera/or a video file

#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <vector>
#include <limits.h>
#include <sstream>

// opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "opencv2/core/version.hpp"


const int8_t NEXT_SAME_LEVEL = 0;
const int8_t PREV_SAME_LEVEL = 1;
const int8_t CHILD = 2;
const int8_t PARENT = 3 ;

const int8_t DRAW_SELECTED_CONTOUR = 0;
const int8_t DRAW_NESTED_CONTOURS = 1;


#define CIRCLE_THICKNESS 1
#define CIRCLE_FILL -1

#define DEFAULT_HOR_RES 640
#define DEFAULT_VER_RES 480

const short ESC_KEY = 27;


struct threshold_limits
{
    cv::Scalar low_thresh;
    cv::Scalar high_thresh;
};


void print_usage(char* p);
void initialise(int argc, char* argv[], cv::VideoCapture& cap, cv::Point& center_point, bool& image_file_flag, std::string& image_file_path, bool& determine_range);

void draw_contours_and_rectangles(cv::Mat& input, 
                                    std::vector<std::vector<cv::Point> >& target_contours,
                                    int largest_cont_index,
                                    int second_largest_cont_index,
                                    int largest_child_index,
                                    cv::Scalar& color_largest, 
                                    std::vector<cv::Vec4i>& contour_hierarchy,
                                    cv::Rect& target_primary, cv::Rect& target_secondary
                                    );

void calculate_and_cache_contour_areas(std::vector<std::vector<cv::Point> >& target_contours,  
                                                  std::vector<std::pair<uint32_t, double>>& contour_areas);

void apply_thresh_and_morphology(cv::Mat& hsv, cv::Mat& target_mask);

void find_largest_child_contour(std::vector<cv::Vec4i>& contour_hierarchy, 
                                    std::vector<std::vector<cv::Point>>& target_contours, 
                                    double& largest_child_area, 
                                    int& largest_child_index ,  
                                    const int largest_contour_index 
                            );

void find_target(cv::VideoCapture& cap,  bool& image_file_flag, const std::string& image_file_path);
void determine_range(cv::VideoCapture& cap, bool& image_file_flag, const std::string& image_file_path);


int main(int argc, char* argv[])
{
    

    cv::VideoCapture cap;
    cv::Point center_point;
    bool image_file_flag = false, range_mode = false;
    std::string image_file_path;
    initialise(argc, argv, cap, center_point, image_file_flag, image_file_path, range_mode);

    // peter: hard code colour limits for now.
    
    // blue limits
    struct threshold_limits threshold_limits_hsv;

    // tuned red(?) limits
    threshold_limits_hsv.low_thresh = cv::Scalar(55,75,75);
    threshold_limits_hsv.high_thresh = cv::Scalar(130, 255, 220);

    if(range_mode){
        determine_range(cap, image_file_flag, image_file_path);
    } else {
        find_target(cap, image_file_flag, image_file_path);
    }
    
    std::cout << "Cleaning up camera.\n"; 
    cap.release();

    cv::destroyAllWindows();

    return 0;   
}

void print_usage(char* p)
{
    printf("usage: %s \n", p);

    printf("\t-v [VIDEO_FILE]\t\tvideo file to use as input.\n\t\t\t");
    printf("defaults to camera index 0.\n");

    printf("\t-i [IMAGE_FILE]\t\timage file to use as input.\n\t\t\t");
    printf("no defaiult value.\n");

    printf("\t-c [CAMERA INDEX]\t\tcamera index to use.\n\t\t\t");
    printf("defaults to camera index 0.\n");
}



void initialise(int argc, char* argv[], cv::VideoCapture& cap, cv::Point& center_point, bool& image_file_flag, std::string& image_file_path, bool& determine_range)
{

    if(CV_MAJOR_VERSION == 2){
        std::cout << "USING OPENCV VER: 2\n";
    } else if(CV_MAJOR_VERSION == 3){
        std::cout << "USING OPENCV VER: 3\n";        
    } else {
        std::cout << "USING OPENCV VER: UNKNOWN\n";
    }

    bool video_file_flag = false;
    
    int c = 0;
    // read in cmd line args
    opterr = 0;
    std::string video_file_path_arg, image_file_path_arg;
    int cam_index = 0;

    while ((c = getopt (argc, argv, "b:v:c:i:r")) != -1){   
        switch (c){
            
            case 'i':
                image_file_flag = true;
                image_file_path_arg = optarg;
                printf("Using image file at: %s.\n", image_file_path_arg.c_str());
                break;

            case 'v':
                video_file_flag = true;
                video_file_path_arg = optarg;
                printf("Using video file at: %s.\n", video_file_path_arg.c_str());
                break;
            
            case 'c':
                cam_index = std::stoi(optarg);
                printf("Setting camera index to: %d\n", cam_index);
                break;
            case 'r':
                determine_range = true;
                printf("Setting up to determine range.\n");
                break;

            case '?':
                print_usage(argv[0]);    
                exit(1);
        
            default:
                print_usage(argv[0]);
                abort ();
        }
    }



    printf("Initialising capture object...\n");
    if(video_file_flag){
        printf("\tusing video file: %s\n", video_file_path_arg.c_str());
        cap = cv::VideoCapture(video_file_path_arg);
        
        if(!cap.isOpened()) {
            printf("ERROR: could not open ");
            exit(1);
        }

    } else if (image_file_flag) {
        printf("\tusing image file: %s\n", image_file_path_arg.c_str());
        image_file_path = std::string(image_file_path_arg);
    } else {
        printf("\tusing input stream, video index %d\n", cam_index);
        cap = cv::VideoCapture(cam_index);
    }

    

    int input_height = DEFAULT_VER_RES, input_width = DEFAULT_HOR_RES;

    // set resolution of camera .
    if(video_file_flag){
        input_height = (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT);
        input_width = (int)cap.get(CV_CAP_PROP_FRAME_WIDTH); 
        printf("\tvideo file dimensions: %d x %d\n",input_width, input_height);  
        printf("\tvideo file framerate: %lf\n", cap.get(CV_CAP_PROP_FPS));
        
    } else {
        cap.set(CV_CAP_PROP_FRAME_WIDTH, input_width);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT,input_height);        
    }
 
    // determine centerpoint of image
    center_point = cv::Point(input_width/2, input_height/2);
}


void draw_contours_and_rectangles(cv::Mat& input, 
                    std::vector<std::vector<cv::Point> >& target_contours,
                    int largest_cont_index,
                    int second_largest_cont_index,
                    int largest_child_index,
                    cv::Scalar& color_largest, 
                    std::vector<cv::Vec4i>& contour_hierarchy,
                    cv::Rect& target_primary, cv::Rect& target_secondary
                    )
{

    cv::Scalar color_child( 255, 100, 100 );
    cv::drawContours( input, target_contours, largest_child_index, 
                color_child, 2, CV_AA, contour_hierarchy, 1 );

    cv::drawContours( input, target_contours, second_largest_cont_index, 
    color_largest, 2, CV_AA, contour_hierarchy, DRAW_SELECTED_CONTOUR );

    // find center of first and second largest contours (to determine vector between them )
    target_primary  = boundingRect(target_contours.at(largest_cont_index)); 
    target_secondary = boundingRect(target_contours.at(second_largest_cont_index));
    


    cv::Scalar color_rect( 0, 220, 0 );
    int thickness = 4;
    int shift = 0;

    // draw the above:
    rectangle(input, target_primary, color_rect , thickness, CV_AA, shift);
    rectangle(input, target_secondary, color_rect, thickness, CV_AA, shift);
}


void calculate_and_cache_contour_areas(std::vector<std::vector<cv::Point> >& target_contours,  
                                                  std::vector<std::pair<uint32_t, double>>& contour_areas)
{
    for (uint32_t i = 0; i < target_contours.size(); ++i){
        contour_areas.push_back(std::make_pair(i, cv::contourArea(target_contours.at(i))));
    }

    // sort the set of contours by area.
    std::sort(contour_areas.begin(), contour_areas.end(), 
        [](std::pair<uint32_t, double> a, std::pair<uint32_t, double> b)
        {
            return a.second > b.second;
        }
    );
}


void apply_thresh_and_morphology(cv::Mat& hsv, cv::Mat& target_mask)
{
    struct threshold_limits threshold_limits_hsv;
    
    // SIMULATOR values for RED TARGET
    // threshold_limits_hsv.low_thresh = cv::Scalar(0,200,200);
    // threshold_limits_hsv.high_thresh = cv::Scalar(80, 255, 255);

    // LIVE values for RED TARGET
    threshold_limits_hsv.low_thresh  = cv::Scalar(140,  55,  55);
    threshold_limits_hsv.high_thresh = cv::Scalar(179, 255, 255);

    cv::Mat first_target_mask;
    cv::inRange(hsv, threshold_limits_hsv.low_thresh, threshold_limits_hsv.high_thresh, first_target_mask);
    imshow("primary_thresholded", first_target_mask);

    struct threshold_limits second_threshold;
    second_threshold.low_thresh     = cv::Scalar(0,   55,  55);
    second_threshold.high_thresh    = cv::Scalar(5, 255, 255);
    cv::Mat second_target_mask;

    cv::inRange(hsv, second_threshold.low_thresh, second_threshold.high_thresh, second_target_mask);
    imshow("secondary_thresholded", second_target_mask);

    target_mask = first_target_mask + second_target_mask;

    // erode and dilate our masked images to remove (as much as possible) noise 
    // peter: try a more agressive morphology here to remove the light noise? 
    cv::erode(target_mask, target_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)) );
    cv::dilate(target_mask, target_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)) ); 

    imshow("post_erode_dilate", target_mask);

}


void find_largest_child_contour(std::vector<cv::Vec4i>& contour_hierarchy, 
                                std::vector<std::vector<cv::Point>>& target_contours, 
                                double& largest_child_area, 
                                int& largest_child_index ,  
                                const int largest_contour_index 
                            )
{
    // find largest sub-contour  
    int child_level = contour_hierarchy[largest_contour_index][CHILD];
    int curr_child = child_level;


    // reset search at beginning of contours
    // peter: this doesnt actually solve that problem at all. 
    // remove and solve correctly.
    int tmp_prev = -1;
    while(curr_child != -1){
        tmp_prev = curr_child;
        curr_child = contour_hierarchy[curr_child][PREV_SAME_LEVEL];
    }
    curr_child = tmp_prev;

    // search for largest sub-contour 
    while( curr_child != -1 ){ 
        double current_child_area = cv::contourArea(target_contours.at(curr_child));
        if( largest_child_area < current_child_area ){
            largest_child_area = current_child_area;
            largest_child_index = curr_child;
        }
        curr_child = contour_hierarchy[curr_child][NEXT_SAME_LEVEL];
    
    }
}


void find_target(cv::VideoCapture& cap, bool& image_file_flag, const std::string& image_file_path)
{
    // produce named windows
    cv::namedWindow("input",CV_WINDOW_NORMAL);
    cv::namedWindow("hsv",CV_WINDOW_NORMAL);
    
    while(true){

        cv::Mat input;
        bool read_success = false;
        if( image_file_flag ){
            input = cv::imread(image_file_path);
            
            if(input.empty()){
                std::cout << "Cannot read a frame from video stream\n";
                exit(1);
            }
            
            read_success = true;
        
        } else {
            read_success = cap.read(input); // read a new frame from video
        }

        // make sure we use a different (deep clone) matrix 
        cv::Mat frame(input);

        // convert to HSV 
        cv::Mat hsv, target_mask;
        cv::cvtColor(frame, hsv, CV_BGR2HSV);
        medianBlur(hsv, hsv, 9);
        cv::imshow("hsv", hsv);

        apply_thresh_and_morphology(hsv, target_mask);



        // find contours in the image
        std::vector<std::vector<cv::Point> > target_contours;
        std::vector<cv::Vec4i> contour_hierarchy;
        findContours( target_mask.clone(), target_contours, contour_hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));


        if( target_contours.size() == 0 ){
            std::cout << "\t"<< __func__ << ": no target contours in target frame.\n";    
        } else {
            
            // check to see if we found any contours - if so, mark them (temporary) with a circle/dot
            // find the largest contour
            // target square
            double largest_cont_area = INT_MIN;
            int largest_cont_index = -1;
            int second_largest_cont_index = -1;

            // to improve sort performance, we cache the areas of each contour
            // to improve sort performance, we cache the areas of each contour
            std::vector<std::pair<uint32_t, double>> contour_areas;
            calculate_and_cache_contour_areas( target_contours, contour_areas);


            largest_cont_index = contour_areas.at(0).first;
            if(contour_areas.size() > 1){
                second_largest_cont_index = contour_areas.at(1).first; 
            }


            if (largest_cont_index == -1){
                std::cout << "ERROR: largest_cont_index was -1\n";  
                exit(1);
            } else if (largest_cont_index >= static_cast<int>(contour_hierarchy.size()) ){
                std::cout << "ERROR: largest_cont_index was \n";  
                exit(1);
            }


            // draw the largest area contour
            cv::Scalar color_largest( 0, 0, 255 );
            cv::drawContours( input, target_contours, largest_cont_index, 
                    color_largest, 2, CV_AA, contour_hierarchy, DRAW_SELECTED_CONTOUR );


            // find largest sub-contour  
            int largest_child_index = -1;
            double largest_child_area = INT_MIN;
            
            find_largest_child_contour( contour_hierarchy, target_contours, largest_child_area, 
                            largest_child_index, largest_cont_index);




            if( largest_child_index == -1 ){
                //std::cout << "No child of current contour exists...\n"; 
            } else if( largest_child_area < 0.05*largest_cont_area ){
                //std::cout << "Child contour not large enough to be of interest.\n";

                // send instructions to move towards the contour (improve our position towards goal) 

            } else {

                cv::Scalar color_child( 125, 125, 255 );
                cv::drawContours( input, target_contours, largest_child_index, 
                        color_child, 2, CV_AA, contour_hierarchy, 1 );


                if ( second_largest_cont_index == -1 ){
                    //std::cout << "could not find second largest contour!\n";
                    
                } else {


                    cv::Rect target_primary, target_secondary;
                    draw_contours_and_rectangles( input, target_contours, largest_cont_index, 
                                        second_largest_cont_index, largest_child_index, color_largest, 
                                        contour_hierarchy, target_primary,
                                        target_secondary );

                    cv::Point circle_target_center( (target_primary.x+(target_primary.width/2)), 
                            (target_primary.y+(target_primary.height/2)) ); 

                    cv::Point rectangle_target_center( (target_secondary.x+(target_secondary.width/2)), 
                            (target_secondary.y+(target_secondary.height/2)) ); 

                    arrowedLine(input, circle_target_center, rectangle_target_center, cv::Scalar(0,0,255), 3, CV_AA, 0, 0.3);

                }


            }

        }


        imshow("input", input);

        if (cv::waitKey(30) == 27)  {
            std::cout << "esc key is pressed by user\n";
            break; 
        }
        
    }
}



void determine_range(cv::VideoCapture& cap, bool& image_file_flag, const std::string& image_file_path)
{

    cv::namedWindow("Control",CV_WINDOW_NORMAL); 

    int iLowH = 0;
    int iHighH = 179;

    int iLowS = 0; 
    int iHighS = 255;

    int iLowV = 0;
    int iHighV = 255;

    //Create trackbars in "Control" window
    cv::createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cv::createTrackbar("HighH", "Control", &iHighH, 179);

    cv::createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cv::createTrackbar("HighS", "Control", &iHighS, 255);

    cv::createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cv::createTrackbar("HighV", "Control", &iHighV, 255);

    while (true)
    {
        cv::Mat imgOriginal;

        bool read_success = false;
        if( image_file_flag ){
            imgOriginal = cv::imread(image_file_path);
            
            if(imgOriginal.empty()){
                std::cout << "Cannot read a frame from video stream\n";
                exit(1);
            }
            read_success = true;

        } else {
            read_success = cap.read(imgOriginal); // read a new frame from video
            
        }

        if (!read_success) //if not success, break loop
        {
            std::cout << "Cannot read a frame from video stream\n";
            break;
        }

        cv::Mat imgHSV;

        medianBlur(imgOriginal, imgOriginal, 9);
        cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        imshow("hsv", imgHSV);

        cv::Mat imgThresholded;

        inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

        //morphological opening (remove small objects from the foreground)
        erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 

        //morphological closing (fill small holes in the foreground)
        dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> contour_hierarchy;
        findContours( imgThresholded.clone(), contours, contour_hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

        cv::Mat contour_out(imgOriginal);

        for( unsigned int i = 0; i< contours.size(); i++ )
        {
            cv::Scalar color = cv::Scalar( 0,255,0 );
            drawContours( contour_out, contours, i, color, 2, 8, contour_hierarchy, 0, cv::Point() );
        }





        imshow("Control", imgThresholded); //show the thresholded image
        imshow("Original", imgOriginal); //show the original image

        if (cv::waitKey(30) == 27)  {
            std::cout << "esc key is pressed by user\n";
            break; 
        }
    }
}