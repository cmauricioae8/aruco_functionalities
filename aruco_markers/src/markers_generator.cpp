/**
 * @description: Aruco markers generator (png images).
 * 
 * \author C. Mauricio Arteaga-Escamilla
 * \note Markers images are saved in the same directory as the executable (build)
 */

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"

#include <sstream>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>

int last_marker_id = 5; // markers will be generating starting from 0 to last_marker_id

using namespace std;
using namespace cv;


void createArucoMarkers()
{
  Mat outputMarker;

  Ptr<aruco::Dictionary>markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_7X7_1000);

  for(int i = 0; i <= last_marker_id; i++){
    aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
    ostringstream convert;

    string imageName = "7x7Marker_";
    
    convert << imageName << i << ".png";
    
    imwrite(convert.str(), outputMarker);
  }
}


int main(int argc, char **argv)
{
  createArucoMarkers();
  return 0;
}
