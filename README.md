# dlib-object-tracking
Accurate and optimized object tracker using DLib and OpenCV    

This program can be used to serve the purpose of a high-quality object tracker. It can be combined easily with an object detector to continously track and locate the object in subsequent frames.    
It uses OpenCV's cascade detection technique to detect the object. Then, the detected rectangle is sent to Dlib for further tracking.
 
# Compile     
g++ -std=c++11 -O3 -I.. $DLIB_ROOT/dlib/all/source.cpp -lpthread -lX11 -ljpeg -DDLIB_JPEG_SUPPORT object-tracking.cpp -o object-tracker `pkg-config --cflags --libs opencv`

# Usage     
./object-track CASCADE_PATH VID_PATH/CAM_INDEX 

# Example Usage    
For Videos:    
./object-track /usr/share/opencv/haarcascades/haarcascade_frontalface_alt2.xml ~/Videos/video.mp4     

For Live Camera:    
./object-track /usr/share/opencv/haarcascades/haarcascade_frontalface_alt2.xml 0

