#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <math.h>
#include <string.h>


const char* wndname = "Cross and Circle Detection Demo";
int thresh = 50, N = 11;


static void help();
static void findSquares( const cv::Mat& image, std::vector<std::vector<cv::Point> >& squares );
static void drawSquares( cv::Mat& image, const std::vector<std::vector<cv::Point> >& squares );

int main(int argc, char** argv)
{
    help();
    cv::namedWindow( wndname, 1 );
    std::vector<std::vector<cv::Point> > squares;

        cv::Mat image = cv::imread(argv[1], 1);
        if( image.empty() ) {
            std::cout << "Couldn't load " << argv[1] << "\n";
            return -1;
        }

        findSquares(image, squares);
        drawSquares(image, squares);

        cv::waitKey();

    return 0;
}



static void help()
{
    std::cout <<
    "Call:\n"
    "./squares <filename>\n"
    "Using OpenCV version %s\n" << CV_VERSION << "\n";
}



// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
static double angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}



// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
static void findSquares( const cv::Mat& image, std::vector<std::vector<cv::Point> >& squares )
{
    squares.clear();

    cv::Mat pyr, timg, gray0(image.size(), CV_8U), gray;

    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, cv::Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());
    std::vector<std::vector<cv::Point> > contours;

    // find squares in every color plane of the image
    for ( int c = 0; c < 3; c++ ) {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        for( int l = 0; l < N; l++ ) {

            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            
            if ( l == 0 ) {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 0, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, cv::Mat(), cv::Point(-1,-1));
            
            } else {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }

            // find contours and store them all as a list
            findContours(gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

            std::vector<cv::Point> approx;

            // test each contour
            for ( size_t i = 0; i < contours.size(); i++ )  {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(cv::Mat(contours[i]), approx, 5, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() > 7 && isContourConvex(cv::Mat(approx)) ) {
                    double maxCosine = 0;

                    for( int j = 2; j < 5; j++ ) {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                  //  if( maxCosine < 0.3 )
                        squares.push_back(approx);
                }
            }
        }
    }
}


// the function draws all the squares in the image
static void drawSquares( cv::Mat& image, const std::vector<std::vector<cv::Point> >& squares )
{
    for( size_t i = 0; i < squares.size(); i++ ){
        const cv::Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(image, &p, &n, 1, true, cv::Scalar( (rand()&255), (rand()&255), (rand()&255) ), 1, cv::LINE_AA);
    }

    imshow(wndname, image);
}


