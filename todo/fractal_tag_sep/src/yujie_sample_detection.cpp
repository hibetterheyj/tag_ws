#include <aruco.h>
#include <iostream>
#include <opencv2/highgui.hpp>

using namespace std;
int main(int argc, char **argv) {
  cv::Mat im = cv::imread(argv[1]);
  aruco::FractalDetector FDetector;
  FDetector.setConfiguration("FRACTAL_3L_6");
  if (FDetector.detect(im)) {
    std::cout << "Founded!" << std::endl;
    FDetector.drawMarkers(im);
  } else {
    std::cout << "Not founded!" << std::endl;
  }
  // Show me the inner corners !
      FDetector.draw2d(im);
      // FDetector.draw3d(im, true, true);
      // autoresize
      cv::imshow("image", im);
  cv::waitKey(0);
  return 0;
}
