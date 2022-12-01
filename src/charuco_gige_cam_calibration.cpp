#include "CameraApi.h"
#include <iomanip>
#include <stdio.h>
#include <iostream>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/core.hpp"
#include "opencv2/videoio/legacy/constants_c.h"
#include "opencv2/highgui/highgui_c.h"


using namespace std;
using namespace cv;
unsigned char           * g_pRgbBuffer;     //processed data cache
int markNum(1);

// Mat markerGenerator(int markNum){
//     Mat markerImage;
//     Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
//     aruco::drawMarker(dictionary, markNum, 200, markerImage, 1);
//     imwrite("marker0.png", markerImage);
//     return markerImage;
// }
int gige_cam()
{

    int                     iCameraCounts = 1;
    int                     iStatus=-1;
    tSdkCameraDevInfo       tCameraEnumList;
    int                     hCamera;
    tSdkCameraCapbility     tCapability;      //Device description information
    tSdkFrameHead           sFrameInfo;
    BYTE*			        pbyBuffer;
    int                     iDisplayFrames = 10000;
    IplImage *iplImage = NULL;
    int                     channel=3;

    CameraSdkInit(1);

    
    //Enumerate devices and create a list of devices
    iStatus = CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
	printf("state = %d\n", iStatus);

	printf("count = %d\n", iCameraCounts);
    
    // no device connected
    if(iCameraCounts==0){
        return -1;
    }

    //Camera initialization. After the initialization is successful, 
    // any other camera-related operation interface can be called
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);

    //initialization failed
	printf("state = %d\n", iStatus);
    if(iStatus!=CAMERA_STATUS_SUCCESS){
        return -1;
    }


    //Get the camera's characteristic description structure. 
    // This structure contains the range information of various parameters that can be set
    //  by the camera. Determines the parameters of the relevant function
    
    CameraGetCapability(hCamera,&tCapability);

    //
    g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
       
     /* Let the SDK enter the working mode and start receiving images sent from the camera
     data. If the current camera is in trigger mode, it needs to receive
     The image is not updated until the frame is triggered. */
    CameraPlay(hCamera);

    /*Other camera parameter settings
     For example CameraSetExposureTime CameraGetExposureTime set/read exposure time
          CameraSetImageResolution CameraGetImageResolution Set/read resolution
          CameraSetGamma, CameraSetConrast, CameraSetGain, etc. set image gamma, contrast, RGB digital gain, etc.
          This routine is just to demonstrate how to convert the image obtained in the SDK into the OpenCV image format, so as to call the OpenCV image processing function for subsequent development
     */

    if(tCapability.sIspCapacity.bMonoSensor){
        channel=1;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        channel=3;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
    }


// Loop to display 1000 frames of images
    while(true)
    {

        if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
		{
		    CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);
		    
		    cv::Mat frame(
					cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight), 
					sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
					g_pRgbBuffer
					);

            cv::namedWindow("Opencv Demo");
			imshow("Opencv Demo", frame);

            int key = cv::waitKey(1);
            if (key == 27) // ESP stop
                break;

            // After successfully calling CameraGetImageBuffer, 
            // you must call CameraReleaseImageBuffer to release the obtained buffer.
            //Otherwise, when calling CameraGetImageBuffer again, the program will 
            // be suspended and blocked until other threads call CameraReleaseImageBuffer to release the buffer
            
			CameraReleaseImageBuffer(hCamera,pbyBuffer);

		}
    }

    CameraUnInit(hCamera);
    //Note, free after deinitialization
    
    free(g_pRgbBuffer);


    return 0;
}

int gige_aruco_detector()
{

    int                     iCameraCounts = 1;
    int                     iStatus=-1;
    tSdkCameraDevInfo       tCameraEnumList;
    int                     hCamera;
    tSdkCameraCapbility     tCapability;      //Device description information
    tSdkFrameHead           sFrameInfo;
    BYTE*			        pbyBuffer;
    int                     iDisplayFrames = 10000;
    IplImage *iplImage = NULL;
    int                     channel=3;

    CameraSdkInit(1);

    
    //Enumerate devices and create a list of devices
    iStatus = CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
	printf("state = %d\n", iStatus);

	printf("count = %d\n", iCameraCounts);
    
    // no device connected
    if(iCameraCounts==0){
        return -1;
    }

    //Camera initialization. After the initialization is successful, 
    // any other camera-related operation interface can be called
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);

    //initialization failed
	printf("state = %d\n", iStatus);
    if(iStatus!=CAMERA_STATUS_SUCCESS){
        return -1;
    }


    //Get the camera's characteristic description structure. 
    // This structure contains the range information of various parameters that can be set
    //  by the camera. Determines the parameters of the relevant function
    
    CameraGetCapability(hCamera,&tCapability);

    //
    g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
       
     /* Let the SDK enter the working mode and start receiving images sent from the camera
     data. If the current camera is in trigger mode, it needs to receive
     The image is not updated until the frame is triggered. */
    CameraPlay(hCamera);

    /*Other camera parameter settings
     For example CameraSetExposureTime CameraGetExposureTime set/read exposure time
          CameraSetImageResolution CameraGetImageResolution Set/read resolution
          CameraSetGamma, CameraSetConrast, CameraSetGain, etc. set image gamma, contrast, RGB digital gain, etc.
          This routine is just to demonstrate how to convert the image obtained in the SDK into the OpenCV image format, so as to call the OpenCV image processing function for subsequent development
     */

    if(tCapability.sIspCapacity.bMonoSensor){
        channel=1;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        channel=3;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
    }


// Loop to display 1000 frames of images
    while(true)
    {

        if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
		{
		    CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);
		    
		    cv::Mat frame(
					cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight), 
					sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
					g_pRgbBuffer
					);

            cv::namedWindow("Opencv Demo");
			imshow("Opencv Demo", frame);

            int key = cv::waitKey(1);
            if (key == 27) // ESP stop
                break;

            // After successfully calling CameraGetImageBuffer, 
            // you must call CameraReleaseImageBuffer to release the obtained buffer.
            //Otherwise, when calling CameraGetImageBuffer again, the program will 
            // be suspended and blocked until other threads call CameraReleaseImageBuffer to release the buffer
            
			CameraReleaseImageBuffer(hCamera,pbyBuffer);

		}
    }

    CameraUnInit(hCamera);
    //Note, free after deinitialization
    
    free(g_pRgbBuffer);


    return 0;
}



void markderDetector(){
    VideoCapture inputVideo;
    inputVideo.open(2);
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
    while (inputVideo.grab()) {
    Mat image, imageCopy;
    inputVideo.retrieve(image);
    image.copyTo(imageCopy);
    vector<int> ids;
    vector<vector<Point2f> > corners;
    aruco::detectMarkers(image, dictionary, corners, ids);

    // if at least one marker detected
    if (ids.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, corners, ids);
    imshow("out", imageCopy);
    char key = (char) cv::waitKey(50);
    if (key == 'q')
        break;
    }
}



int main(int, char**)
{
    gige_aruco_detector();

    // markderDetector();

    return 0;
}
