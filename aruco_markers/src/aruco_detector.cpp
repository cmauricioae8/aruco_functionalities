/**
 * @author C. Mauricio Arteaga-Escamilla
 * @note This program detects aruco markers from the 7x7_1000 dictionary using either the 'build/calibration_values.txt'
 * or the default values.
 * If one or more markers are found, their ID is written and their coordinate frame is drawn.
 * @source: https://www.youtube.com/watch?v=R3RRKDcW2RU&list=PLAp0ZhYvW6XbEveYeefGSuLhaPlFML9gP&index=19
 *
 * @note To run the program:
 * $ ./aruco_detector -s=0.084 -ci=0
 */

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/calib3d.hpp"

#include <sstream>
#include <iostream>
#include <fstream>
#include <stdio.h>

using namespace std;
using namespace cv;


namespace {
const char* about = "ArUco marker detection program.";
const char* keys  =
        "{h help   |false | Print help }"
        "{s square |0.084 | ArUco square dimension (meters) }"
        "{ci camera|0     | Video capture ID }"
        ;
}


void loadCameraCalibration(const char* name, Mat& cameraMatrix, Mat& distanceCoefficients)
{
	ifstream inStream(name);
	if (inStream)
	{
		uint16_t rows;
		uint16_t columns;

		inStream >> rows;
		inStream >> columns;

		cameraMatrix = Mat(Size(columns, rows), CV_64F);

    	cout << "Intrinsic parameters (in pixels)";
		  
		for (int r = 0; r<rows; r++)
		{
			cout << "\n";
			for (int c = 0; c <columns; c++)
			{
				double read = 0.0f;
				inStream >> read;
				cameraMatrix.at<double>(r, c) = read; 
				cout << cameraMatrix.at<double>(r, c) << "\t";
			}
		}
		// Distortion coefficients
		inStream >> rows;
		inStream >> columns;

		distanceCoefficients = Mat::zeros(rows, columns, CV_64F);

		cout << "\nThe distortion coefficients are:\n";
		for (int r = 0; r<rows; r++)
		{
			for (int c = 0; c <columns; c++)
			{
				double read = 0.0f;
				inStream >> read;
				distanceCoefficients.at<double>(r, c) = read;
				cout << distanceCoefficients.at<double>(r, c) << "\t";
			}
		}
		cout << "\n\n";
		inStream.close();
		return;
	}
	cout << "\x1B[1;33m '" << name << "' NOT found in the same executable directory (build folder). "
		<< "\x1B[1;32mUsing default values.\x1B[0m\n\n";

	cameraMatrix = (cv::Mat_<double>(3,3) <<  1.2195112968898779e+003, 0., 3.6448211117862780e+002, 0.,
                                        1.2414409169216196e+003, 2.4321803868732076e+002, 0., 0., 1.);
	distanceCoefficients = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion
}

int startWebcamMonitoring(const Mat& cameraMatrix, const Mat& distanceCoefficients, float arucoSquareDimension, int video_capture_id)
{
	Mat frame;
	vector<int> markerIds;
	vector<vector<Point2f> > markerCorners, rejectedCandidates;

	//Initialize the detector parameters using default values
	Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();

	Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_7X7_1000);

	VideoCapture vid(video_capture_id);

	if (!vid.isOpened())
	{
		return -1;
	}

	namedWindow("Webcam", WINDOW_AUTOSIZE);

	vector<Vec3d> rotationVectors, translationVectors;

	while (true)
	{
		if (!vid.read(frame))
			break;
		
		// Detect the markers in the image
		aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds, parameters, rejectedCandidates);
		//rejectedCandidates is a returned list of marker candidates. This parameter can be omitted
		//aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);

		aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors);
			
		for (int i = 0; i < markerIds.size(); i++)
		{
			//Axis-color correspondences are X: red, Y: green, Z: blue
			aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], 0.1f);

			//The small red square indicates the marker’s top left corner
			aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
		}

		imshow("Webcam", frame);
		if (waitKey(30) >= 0) break;
	}
	return 1;
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

  float arucoSquareDimension = parser.get<float>("s");
  int video_capture_id = parser.get<int>("ci");

  if (!parser.check())
  {
    parser.printErrors();
    return 1;
  }

  Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
  Mat distanceCoefficients;

  cout << "The program is terminated with 'Esc' if the webcam window is active\n";
  cout << "arucoSquareDimension: " << arucoSquareDimension << endl;
  cout << "video_capture_id: " << video_capture_id << endl << endl;

  loadCameraCalibration("calibration_values.txt", cameraMatrix, distanceCoefficients);
  startWebcamMonitoring(cameraMatrix, distanceCoefficients, arucoSquareDimension, video_capture_id);

  return 0;
}
	
