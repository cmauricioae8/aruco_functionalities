/**
 * @description: Camera calibration program.
 * 
 * @author C. Mauricio Arteaga-Escamilla
 * @note This program obtains the intrinsic parameters and distortion coefficients of the camera
 * from multiple photos (from 30 to 50 for a good calibration) using a chessboard.
 * The calibration parameters are saved in a file without extension in 'build' folder, next to the executable.
 * 
 * @note To run the program:
 * $ ./camera_calibration -s=0.03 -w=8 -ny=5 -ci=0
 * 
 */

#include <sstream>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui/highgui_c.h> //To solve criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER ... problem

using namespace std;
using namespace cv;

namespace {
const char* about = "Camera calibration program.";
const char* keys  =
        "{h help   |false | Print help }"
        "{s square |0.015 | Calibration square dimension (meters) }"
        "{w width  |9     | Number of internal corners horizontally }"
        "{ny height|6     | Number of internal corners vertically }"
        "{ci camera|0     | Camera ID }"
        ;
}


/**
 * @brief Create the known board position
 * 
 * @param boardSize 
 * @param squareEdgeLength 
 * @param corners 
 */
void createKnownBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f>& corners)
{
	for (int i=0; i < boardSize.height; i++)
	{
		for (int j = 0; j < boardSize.width; j++)
		{
			corners.push_back(Point3f(j*squareEdgeLength, i*squareEdgeLength, 0.0f));
		}
  }
}

void getChessboardCorners(vector<Mat> images, vector<vector<Point2f> >& allFoundCorners, Size chessboardDimensions, bool showResults = false )
{
  for(vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
	{
		vector<Point2f> pointBuf;
    bool found = findChessboardCorners(*iter, chessboardDimensions, pointBuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
    if(found)
		{
			allFoundCorners.push_back(pointBuf);
		}
    if(showResults)
		{
      drawChessboardCorners(*iter, chessboardDimensions, pointBuf, found);
			imshow("Looking for Corners", *iter); 
			waitKey(0);
		}
	}
}

void cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distanceCoefficients)
{
  vector<vector<Point2f> > checkerboardImageSpacePoints;
	getChessboardCorners(calibrationImages, checkerboardImageSpacePoints, boardSize, false);

  vector<vector<Point3f> > worldSpaceCornerPoints(1);
	createKnownBoardPosition(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);
	worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);
	vector <Mat> rVectors, tVectors;
	distanceCoefficients = Mat::zeros(8, 1, CV_64F);

	calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints, boardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors); 
}

bool saveCameraCalibration(const char* name, Mat cameraMatrix, Mat distanceCoefficients)
{
	ofstream outStream(name); 
	if (outStream)
	{
		uint16_t rows = cameraMatrix.rows;
		uint16_t columns = cameraMatrix.cols;

		outStream << rows << endl;
    	outStream << columns << endl;

		for (int r = 0; r<rows; r++)
		{
			for (int c = 0; c<columns; c++)
			{
				double value = cameraMatrix.at<double>(r, c); 
				outStream << value << endl; 
			}
		}

    // Intrinsic parameters (in pixels)
    cout << "\nIntrinsic parameters (in pixels)\n";
    cout<<"fx: "<<cameraMatrix.at<double>(0, 0)<<"\tfy: "<<cameraMatrix.at<double>(1,1)<<"\n";
    cout<<"gamma: "<<cameraMatrix.at<double>(0, 1)<<"\n";
    cout<<"Cx: "<<cameraMatrix.at<double>(0,2)<<"\tCy: "<<cameraMatrix.at<double>(1,2);


    rows = distanceCoefficients.rows;
    columns = distanceCoefficients.cols;
    
    outStream << rows << endl;
    outStream << columns << endl;

    for (int r = 0; r<rows; r++)
    {
      for (int c = 0; c <columns; c++)
      {
        double value = distanceCoefficients.at<double>(r, c);
        outStream << value << endl;
      }
    }

    // Distortion coefficients (k1, k2, p1, p2, k3)
    cout << "\nDistortion coefficients (k1, k2, p1, p2, k3) are the last ";
    cout << rows << " values in the file\n";

	  outStream.close();
	  return true; 
	}
	return false; 
}


int main(int argc, char** argv)
{
  cv::CommandLineParser parser(argc, argv, keys);
  parser.about(about);

  if (parser.get<bool>("h"))
  {
    parser.printMessage();
    return 0;
  }

  float calibrationSquareDimension = parser.get<float>("s");
  int width = parser.get<int>("w");
  int height = parser.get<int>("ny");
  int camera_id = parser.get<int>("ci");
  Size chessboardDimensions(width, height);

  if (!parser.check())
  {
    parser.printErrors();
    return 1;
  }

  Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
  Mat distanceCoefficients;
  bool finish = false;

  cout << "The program ends with 'Esc' if the web cam window is active\n";
  cout << "calibrationSquareDimension: " << calibrationSquareDimension << endl;
  cout << "chessboardDimensions: " << chessboardDimensions.width << "x" << chessboardDimensions.height << endl;
  cout << "camera_id: " << camera_id << endl << endl;
  

  Mat frame, gray, drawToFrame;
  int img_counter = 0;
	
  vector<Mat> savedImages;
  //vector<vector<Point2f> > markerCorners, rejectedCandidates;

  VideoCapture vid(camera_id);
  if (!vid.isOpened())
  {
    return -1;
  }

  // int framesPerSecond = 20;
  string camera_win = "Camera Calibration";
  namedWindow(camera_win, WINDOW_AUTOSIZE);

  cout << "To calibrate the camera, once the corners of the chessboard are drawn, with "
    "the 'Space' key the image is saved and after 15 captures (preferably 50) with "
    "the 'Enter' key the camera parameters are obtained and written in a "
    "file in the 'build' folder.\n\n";

  while (!finish)
  {
    if (!vid.read(frame))
      break;
    
    vector<Vec2f> foundPoints;
    bool found = false;
    found = findChessboardCorners(frame, chessboardDimensions, foundPoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
    frame.copyTo(drawToFrame);
    drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);

    if (found) {
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
      cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
      
      // refining pixel coordinates for given 2d points.
      cv::cornerSubPix(gray, foundPoints, cv::Size(11, 11), cv::Size(-1, -1), criteria); //Function to get better result

      imshow(camera_win, drawToFrame);
    }
    else
      imshow(camera_win, frame);
    
    char character = waitKey(50);

    switch (character)
    {
      case ' '://Espace
        //saving image
        if (found)
        {
          Mat temp;
          frame.copyTo(temp);
          savedImages.push_back(temp);
          cout << frame.size() << endl;
          img_counter++; 
          cout << "counter: " << img_counter << endl;
        }
        break;

      case 13://Enter
        //start calibration
        if (savedImages.size() > 15)
        {
          destroyWindow(camera_win);
          cout << "\nGetting the camera parameters..." << endl;
          cameraCalibration(savedImages, chessboardDimensions, calibrationSquareDimension, cameraMatrix, distanceCoefficients);
          saveCameraCalibration("calibration_values.txt", cameraMatrix, distanceCoefficients);
          finish = true;
        }
        break;

      case 27://Esc
        finish = true;
        break;
    }
  }

  return 0;
}
