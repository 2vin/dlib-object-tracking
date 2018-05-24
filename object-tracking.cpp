#include <dlib/opencv.h>
#include <opencv2/opencv.hpp>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>

#include <dlib/image_io.h>
#include <dlib/dir_nav.h>
#include <vector>
#include <iostream>
#include <string.h>

using namespace dlib;
using namespace std;


std::vector<correlation_tracker> tracker;
image_window win;

cv::CascadeClassifier cascade;
cv::Mat bin;
cv::Mat temp;
float scale = 0.5;

float checkOverlap(cv::Rect& rec1, cv::Rect& rec2)
{
	cv::Rect rectX = rec1 & rec2;
	float overlap = min(std::abs((rec1.x+rec1.width/2)-(rec2.x+rec2.width/2)),std::abs((rec1.x+rec1.width/2)-(rec2.x+rec2.width/2)));
	return overlap;	
}

void initTrackers(array2d<bgr_pixel>& dlibImage, std::vector<cv::Rect>& detects, std::vector<cv::Rect>& tracks)
{
	for(int i = 0; i< detects.size(); i++)
	{
		float overlap = 10000;
		int simIndex = -1;
		for(int j = 0; j< tracks.size(); j++)
		{
			float tempoverlap = checkOverlap(detects[i],tracks[j]);
			if(overlap>tempoverlap)
			{
				overlap = tempoverlap;
				simIndex = j;
			}

		}
		if(overlap>0.5*detects[i].width)
		{
			// Add tracker
			tracker.resize(tracker.size()+1);
			tracker[tracker.size()-1].start_track(dlibImage, centered_rect(point(detects[i].x+detects[i].width/2,detects[i].y+detects[i].height/2), detects[i].width, detects[i].height));
		}
		else
		{
			// Re-initialize tracker
			tracker[simIndex].start_track(dlibImage, centered_rect(point(detects[i].x+detects[i].width/2,detects[i].y+detects[i].height/2), detects[i].width, detects[i].height));
		}
	}
}

void updateTrackers(array2d<bgr_pixel>& dlibImage)
{
	for(int i = 0; i<tracker.size(); i++)
    	tracker[i].update(dlibImage);
}

void drawTrackers()
{
	for(int i = 0; i<tracker.size(); i++)
    	win.add_overlay(tracker[i].get_position());
}

void removeTracker(int cols, int rows, cv::Mat& temp, float& scale)
{
	int loopCount = tracker.size();
	for(int i = 0; i<loopCount; i++)
	{

		if(tracker.size()>0)
		{
			if(int(tracker[i].get_position().left()) <= cols/20 )
			{
				tracker.erase(tracker.begin()+i);
				i--;
			}
			else if(int(tracker[i].get_position().top()) <= rows/20 )
			{
				tracker.erase(tracker.begin()+i);
				i--;
			}
			else if(tracker[i].get_position().right() >= (cols-cols/20) )
			{
				tracker.erase(tracker.begin()+i);
				i--;
			}
			else if(tracker[i].get_position().bottom() >= (rows-rows/20) )
			{
				tracker.erase(tracker.begin()+i);
				i--;
			}
			else if(tracker[i].get_position().right() - tracker[i].get_position().left() < rows/15 )
			{
				tracker.erase(tracker.begin()+i);
				i--;
			}
			else
			{
	            cv::rectangle(temp, cv::Rect(tracker[i].get_position().left()*1.0/scale,tracker[i].get_position().top()*1.0/scale,(tracker[i].get_position().right()-tracker[i].get_position().left())*1.2/scale,(tracker[i].get_position().bottom()-tracker[i].get_position().top())*1.2/scale), cv::Scalar(0,0,255), 1);
			}

			loopCount = tracker.size();
		}

	}

}

void deleteTracker(int index)
{
	tracker.erase(tracker.begin()+index);
}

void detectCascade(cv::Mat& frame, std::vector<cv::Rect>& objects )
{
	cv::Mat frame_gray;
    cv::cvtColor( frame, frame_gray, cv::COLOR_BGR2GRAY );
    cv::equalizeHist( frame_gray, frame_gray );

    cascade.detectMultiScale( frame_gray, objects, 1.1, 2, 0, cv::Size(24, 48) );

}

void reshapeDetect(std::vector<cv::Rect>& detects, float scale )
{
	for(int i = 0; i< detects.size(); i++)
	{
		detects[i].x *= scale;
		detects[i].y *= scale;
		detects[i].width *= scale;
		detects[i].height *= scale;
	}
}

void dlib2cvRect(std::vector<cv::Rect>& tracks)
{
	tracks.resize(tracker.size());
	for(int i = 0; i<tracker.size(); i++)
	{
		tracks[i].x = tracker[i].get_position().left();
		tracks[i].y = tracker[i].get_position().top();

		tracks[i].width = tracker[i].get_position().right() - tracker[i].get_position().left();
		tracks[i].height = tracker[i].get_position().bottom() - tracker[i].get_position().top();
		
	}

}


std::vector<cv::Rect> suppressRect(cv::Mat& frame, std::vector<cv::Rect> objects)
{
    if(bin.empty())
    {
        bin = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC1);
    }
    bin/=1.2;
    for( size_t i = 0; i < objects.size(); i++ )
    {
        bin(cv::Rect(objects[i])) +=100;
        double coeff = cv::countNonZero(bin(cv::Rect(objects[i]))>221)*1.0/(objects[i].width*objects[i].height);
        if(coeff<0.9)
        {
			cv::rectangle(temp, objects[i], cv::Scalar(255,0,0), 1);

            objects.erase(objects.begin()+i);
            i--;
        }
    }
    return objects;
}


int main(int argc, char** argv)
{
    try
    {
        cv::VideoCapture cap;

        if(!strcmp(argv[2],"0"))
        {
        	cap.open(CV_CAP_ANY);
        }
        else
        {
        	cap.open(argv[2]);
        }



        if (!cap.isOpened())
        {
            cerr << "Unable to connect to camera" << endl;
            return 1;
        }

        if( !cascade.load( argv[1] ) )
        { printf("--(!)Error loading the cascade\n"); return -1; };


        cv::Mat tempSmall;
        cv::namedWindow("result", CV_WINDOW_NORMAL );
        cv::setWindowProperty("result", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
        double t = 0;
        dlib::array2d<bgr_pixel> dlibImage;
		
        while(!win.is_closed())
        {
            
            cap >> temp;
            for(int i = 0; i< tracker.size()*1; i++)
    			cap>>temp;

            // Turn OpenCV's Mat into something dlib can deal with.  Note that this just
            // wraps the Mat object, it doesn't copy anything.  So cimg is only valid as
            // long as temp is valid.  Also don't do anything to temp that would cause it
            // to reallocate the memory which stores the image as that will make cimg
            // contain dangling pointers.  This basically means you shouldn't modify temp
            // while using cimg.
            cv::resize(temp, temp, cv::Size(320, 240));
            std::vector<cv::Rect> detects, tracks;
			detectCascade(temp, detects);
			
			/* loop over all the detected objects */ 
		    // for( size_t i = 0; i < detects.size(); i++ )
		    // {
	     	//     cv::rectangle(temp, detects[i], cv::Scalar(255,0,0), 1);
		    // }

		    detects = suppressRect(temp, detects);
		   
		    reshapeDetect(detects, scale);
		    cv::resize(temp, tempSmall, cv::Size(), scale, scale);

		    dlib2cvRect(tracks);

			assign_image(dlibImage, cv_image<bgr_pixel>(tempSmall));

            initTrackers(dlibImage, detects, tracks);
            updateTrackers(dlibImage);
            removeTracker(tempSmall.cols, tempSmall.rows, temp, scale);

            cv::imshow("result", temp);
            if(cv::waitKey(1)>0)
            	break;

			t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();  
			cout<<" Fps achieved : "<<1.0/t<<endl;   
			t = (double)cv::getTickCount();  
        }
    }
    catch(serialization_error& e)
    {
        
    }
    catch(exception& e)
    {
        cout << e.what() << endl;
    }
}
