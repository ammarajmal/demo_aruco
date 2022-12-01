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
const float calibrationSquareDimension = 0.0245f; // meters
const float arucoSquareDimension = 0.1016f; // meters
const Size chessboardDimensions = Size(6,9);
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
    iStatus = CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
	printf("state = %d\n", iStatus);
	printf("count = %d\n", iCameraCounts);
    if(iCameraCounts==0){       // no device connected
        return -1;
    }
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);
	printf("state = %d\n", iStatus);
    if(iStatus!=CAMERA_STATUS_SUCCESS){
        return -1;
    }
    CameraGetCapability(hCamera,&tCapability);
    g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
    CameraPlay(hCamera);
    if(tCapability.sIspCapacity.bMonoSensor){
        channel=1;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    } else {
        channel=3;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
    }
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
    while(true) {
        auto total_start = chrono::steady_clock::now();
        if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS) {
		    CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);
		    cv::Mat frame(
					cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight), 
					sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
					g_pRgbBuffer
					);
            cv::Mat frameCopy;
            frame.copyTo(frameCopy);
            vector<int> ids;
            vector<vector<Point2f> > corners;
            aruco::detectMarkers(frame, dictionary, corners, ids);

            if (ids.size() > 0) {
                aruco::drawDetectedMarkers(frameCopy, corners, ids);
            }
            auto total_end_gige = chrono::steady_clock::now();
            float total_fps_gige = 1000.0 / chrono::duration_cast<chrono::milliseconds>(total_end_gige - total_start).count();
            std::ostringstream stats_ss;
            stats_ss << fixed << setprecision(2);
            stats_ss << "Total FPS: " << total_fps_gige;
            auto stats = stats_ss.str();
                

                int baseline;
                auto stats_bg_sz = getTextSize(stats.c_str(), FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
                rectangle(frameCopy, Point(0, 0), Point(stats_bg_sz.width, stats_bg_sz.height + 10), Scalar(0, 0, 0), FILLED);
                putText(frameCopy, stats.c_str(), Point(0, stats_bg_sz.height + 5), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255, 255, 255));

                cout<< stats.c_str()<<endl;
                

            cv::namedWindow("ARUCO Demo");
			imshow("ARUCO Demo", frameCopy);
            int key = cv::waitKey(1);
            if (key == 27) // ESP stop
                break;
			CameraReleaseImageBuffer(hCamera,pbyBuffer);
		}
    }
    CameraUnInit(hCamera);
    free(g_pRgbBuffer);
    return 0;
}
void webcam_aruco_detector(){
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
bool loadCameraCalibration(string name, Mat& cameraMatrix, Mat& distanceCoefficients){
    ifstream instream(name);
    if (instream){
        uint16_t rows;
        uint16_t columns;
        instream >> rows;
        instream >> columns;
        cameraMatrix = Mat(Size(columns, rows), CV_64F);
        for (int r = 0 ; r< rows; r++){
            for (int c =0; c < columns; c++){
                double read = 0.0f;
                instream >> read;
                cameraMatrix.at<double>(r,c) = read;
                cout<< cameraMatrix.at<double>(r, c) << endl;
            }
        }
        // Distance Coefficients
        instream >> rows;
        instream >> columns;
        distanceCoefficients = Mat::zeros(rows, columns, CV_64F);
        for (int r = 0; r < rows; r ++ ){
            for (int c = 0; c < columns ; c++){
                double read = 0.0f;
                instream >> read;
                distanceCoefficients.at<double>(r,c) = read;
                cout<< distanceCoefficients.at<double>(r,c) << endl;
            }
        }
        instream.close();
        return true;
    }
    return false;
}
int gige_ARUCO_opencv(const Mat& cameramatrix, const Mat& distcoeffs, float arucoSuqareDimension) {
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
    //g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

     /* Let the SDK enter the working mode and start receiving images sent from the camera
     data. If the current camera is in trigger mode, it needs to receive
     The image is not updated until the frame is triggered. */
    CameraPlay(hCamera);

        /*Other camera parameter settings
        For example CameraSetExposureTime CameraGetExposureTime set/read exposure 
        time CameraSetImageResolution CameraGetImageResolution Set/read resolution
        CameraSetGamma, CameraSetConrast, CameraSetGain, etc. set image gamma, 
        contrast, RGB digital gain, etc.
        
        This routine is just to demonstrate how to convert the image obtained in the 
        SDK into the OpenCV image format, so as to call the OpenCV image processing 
        function for subsequent development
     */

    if(tCapability.sIspCapacity.bMonoSensor){
        channel=1;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        channel=3;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
    }

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    while(1)
    {

        auto total_start = chrono::steady_clock::now();

// *********************************************************
    if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
            {
                CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);
                cv::Mat frame(
                        cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight), 
                        sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                        g_pRgbBuffer
                        );
                Mat frameCopy;
                frame.copyTo(frameCopy);
                vector<int> ids;
                vector<vector<Point2f> > corners, rejected;
                aruco::detectMarkers(frameCopy, dictionary, corners, ids, aruco::DetectorParameters::create(), rejected);
                
                // if at least one marker detected
                if (ids.size() > 0) {
                    // aruco::drawDetectedMarkers(frameCopy, corners, ids);
                    vector<Vec3d> rvecs, tvecs;
                    aruco::estimatePoseSingleMarkers(corners, arucoSuqareDimension, cameramatrix, distcoeffs, rvecs, tvecs);

                    // // draw axis for each marker
                    // for(int i=0; i<ids.size(); i++)
                    //     aruco::drawAxis(frameCopy, cameramatrix, distcoeffs, rvecs[i], tvecs[i], 0.1);
                    aruco::drawAxis(frameCopy, cameramatrix, distcoeffs, rvecs[0], tvecs[0], 0.1);
                    auto total_end = chrono::steady_clock::now();
                float total_fps_gige = 1000.0 / chrono::duration_cast<chrono::milliseconds>(total_end - total_start).count();


                std::ostringstream stats_ss;
                stats_ss << fixed << setprecision(2);
                stats_ss << "Total FPS: " << total_fps_gige;
                auto stats = stats_ss.str();
                                int baseline;
                auto stats_bg_sz = getTextSize(stats.c_str(), FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
                rectangle(frame, Point(0, 0), Point(stats_bg_sz.width, stats_bg_sz.height + 10), Scalar(0, 0, 0), FILLED);
                putText(frame, stats.c_str(), Point(0, stats_bg_sz.height + 5), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255, 255, 255));

                cout<< stats.c_str()<<endl;

                }
                cv::namedWindow("ARUCO");
                imshow("ARUCO", frameCopy);
                // auto total_end = chrono::steady_clock::now();
                // float total_fps_gige = 1000.0 / chrono::duration_cast<chrono::milliseconds>(total_end - total_start).count();


                // std::ostringstream stats_ss;
                // stats_ss << fixed << setprecision(2);
                // stats_ss << "Total FPS: " << total_fps_gige;
                // auto stats = stats_ss.str();
                

                // int baseline;
                // auto stats_bg_sz = getTextSize(stats.c_str(), FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
                // rectangle(frame, Point(0, 0), Point(stats_bg_sz.width, stats_bg_sz.height + 10), Scalar(0, 0, 0), FILLED);
                // putText(frame, stats.c_str(), Point(0, stats_bg_sz.height + 5), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255, 255, 255));

                // cout<< stats.c_str()<<endl;
                // cv::namedWindow("Opencv Demo");
                // imshow("Opencv Demo", frame);

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




int main(int, char**)
{
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    Mat distCoefficients;
    loadCameraCalibration("CameraCalibration", cameraMatrix, distCoefficients);
    gige_ARUCO_opencv(cameraMatrix, distCoefficients, arucoSquareDimension);


    // gige_aruco_detector();
    // webcam_aruco_detector();

    return 0;
}
