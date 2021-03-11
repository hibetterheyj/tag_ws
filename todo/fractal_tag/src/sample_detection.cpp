#include <aruco.h>
#include <opencv2/highgui.hpp>

using namespace std;
int main(int argc, char **argv) {
  cv::Mat im = cv::imread(argv[1]);
  aruco::FractalDetector FDetector;
  FDetector.setConfiguration("FRACTAL_3L_6");
  if (FDetector.detect(im)) {
    FDetector.drawMarkers(im);
  }
  FDetector.draw2d(im); // Show me the inner corners!

  cv::imshow("image", im);
  cv::waitKey(0);
}
