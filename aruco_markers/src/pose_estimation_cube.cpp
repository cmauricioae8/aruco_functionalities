/**
 * @author C. Mauricio Arteaga-Escamilla
 *
 * @note This program detects the ArUco markers of the 7x7_1000 dictionary showing their ID, drawing
 * their coordinate axis and a cube on these.
 * Additionally, the 3D spatial position and orientation using Euler angles are displayed,
 * for the largest detected marker.
 *
 * @note To run the program, the length of the marker in meters must be added as a parameter,
 * assuming that all have the same length, this is:
 *
 * $ ... pose_estimation_cube -l=0.084 -ci=1
 * */
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>

using namespace std;
using namespace cv;


namespace {
const char* about = "Pose estimation and cube of ArUco marker images";
const char* keys  =
        "{h help   |false | Print help }"
        "{l        |      | Actual marker length in meter }"
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

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());

    return  norm(I, shouldBeIdentity) < 1e-6;
}

// Calculates rotation matrix to euler angles. The result is the same as MATLAB except the
// order of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R)
{
    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return Vec3f(x, y, z);
}

void drawCubeWireframe(InputOutputArray image, InputArray cameraMatrix,
    InputArray distCoeffs, InputArray rvec, InputArray tvec, float l);


int main(int argc, char **argv)
{
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if (argc < 2) {
        parser.printMessage();
        return 1;
    }

    if (parser.get<bool>("h")) {
        parser.printMessage();
        return 0;
    }

    float marker_length_m = parser.get<float>("l");
    cout << "\nmarker length: " << marker_length_m << endl;

    if (marker_length_m <= 0) {
        std::cerr << "marker length must be a positive value in meter" << std::endl;
        // return 1;
        marker_length_m = 0.084f;
    }

    int video_capture_id = parser.get<int>("ci");

    cout << "\nvideo capture id: " << video_capture_id << endl;


    cv::VideoCapture in_video;

    in_video.open(video_capture_id);

    if (!parser.check()) {
        parser.printErrors();
        return 1;
    }

    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    Mat distanceCoefficients;

    loadCameraCalibration("calibration_values.txt", cameraMatrix, distanceCoefficients);

    if (!in_video.isOpened()) {
        std::cerr << "failed to open video input: " << video_capture_id << std::endl;
        return 1;
    }

    cv::Mat image, image_copy;
    std::ostringstream vector_to_marker;

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_7X7_1000);


    while (in_video.grab())
    {
        in_video.retrieve(image);
        image.copyTo(image_copy);

        Mat Rotationmatrix;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);

        // if at least one marker detected
        if (ids.size() > 0)
        {
          //The small red square indicates the marker’s top left corner
          cv::aruco::drawDetectedMarkers(image_copy, corners, ids);
          std::vector<cv::Vec3d> rvecs, tvecs;
          //aruco::estimatePoseSingleMarkers returns rotation vector in Rodrigues format
          cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m,
                    cameraMatrix, distanceCoefficients, rvecs, tvecs);

          // Draw axis for all markers
          for(int i=0; i < ids.size(); i++)
          {
            //Axis-color correspondences are X: red, Y: green, Z: blue
            aruco::drawAxis(image_copy, cameraMatrix, distanceCoefficients, rvecs[i], tvecs[i], 0.1);
            drawCubeWireframe(image_copy,cameraMatrix,distanceCoefficients,rvecs[i],tvecs[i],marker_length_m);
          }

          // Find the index of the largest marker
          int index = 0;
          double maxArea = 0;
          for (int i = 0; i < corners.size(); i++) {
            double area = cv::contourArea(corners[i]);
            if (area > maxArea) {
              maxArea = area;
              index = i;
            }
          }

          if (ids.size() > 0)
          {
            std::stringstream ss;
            ss << "ID: " << ids[index];
            putText(image_copy, ss.str(), cv::Point(10,15),FONT_HERSHEY_SIMPLEX,0.6,
                      cv::Scalar(0,252,124),1,0,false);

            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(4) << "x: " << std::setw(8) << tvecs[index](0);
            putText(image_copy, vector_to_marker.str(),cv::Point(10, 30),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 252, 124), 1, 0,false);

            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(4) << "y: " << std::setw(8) << tvecs[index](1);
            putText(image_copy, vector_to_marker.str(),cv::Point(10, 50),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 252, 124), 1, 0,false);

            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(4) << "z: " << std::setw(8) << tvecs[index](2);
            putText(image_copy, vector_to_marker.str(), cv::Point(10, 70),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 252, 124), 1, 0,false);

            //getting the euler angles
            cv::Rodrigues(rvecs[index], Rotationmatrix);//Rodrigues converts rvec into the rotation matrix R (and vice versa)

            Vec3f attitude;
            attitude = rotationMatrixToEulerAngles(Rotationmatrix);

            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(2) << "roll: " << std::setw(8) << attitude[0];
            putText(image_copy, vector_to_marker.str(), cv::Point(10, 90),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 252, 124), 1, 0,false);

            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(2) << "pitch: " << std::setw(8) << attitude[1];
            putText(image_copy, vector_to_marker.str(), cv::Point(10, 110),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 252, 124), 1, 0,false);

            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(2) << "yaw: " << std::setw(8) << attitude[2];
            putText(image_copy, vector_to_marker.str(), cv::Point(10, 130),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 252, 124), 1, 0,false);
          }
        }

        imshow("Pose estimation", image_copy);
        char key = (char)cv::waitKey(30);
        if (key == 27)
            break;
    }

    in_video.release();

    return 0;
}

//Funcion para dibujar un cubo en cada marcador
void drawCubeWireframe(InputOutputArray image, InputArray cameraMatrix,
    InputArray distCoeffs, InputArray rvec, InputArray tvec, float l)
{
    CV_Assert(
        image.getMat().total() != 0 &&
        (image.getMat().channels() == 1 || image.getMat().channels() == 3)
    );
    CV_Assert(l > 0);
    float half_l = l / 2.0;

    // project cube points.
    //Se definen los 8 vertices en el espacio 3D respecto de los ejes de cada marcador
    std::vector<cv::Point3f> axisPoints;
    axisPoints.push_back(cv::Point3f(half_l, half_l, l));
    axisPoints.push_back(cv::Point3f(half_l, -half_l, l));
    axisPoints.push_back(cv::Point3f(-half_l, -half_l, l));
    axisPoints.push_back(cv::Point3f(-half_l, half_l, l));
    axisPoints.push_back(cv::Point3f(half_l, half_l, 0));
    axisPoints.push_back(cv::Point3f(half_l, -half_l, 0));
    axisPoints.push_back(cv::Point3f(-half_l, -half_l, 0));
    axisPoints.push_back(cv::Point3f(-half_l, half_l, 0));

    //Se obtienen las proyecciones de los vertices en el plano de la imagen
    std::vector<cv::Point2f> imagePoints;
    projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

    // draw cube edges lines
    cv::line(image, imagePoints[0], imagePoints[1], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[0], imagePoints[4], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[1], imagePoints[2], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[1], imagePoints[5], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[2], imagePoints[3], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[2], imagePoints[6], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[3], imagePoints[7], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[4], imagePoints[5], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[4], imagePoints[7], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[5], imagePoints[6], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[6], imagePoints[7], cv::Scalar(255, 0, 0), 3);
}
