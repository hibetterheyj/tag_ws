
#include "cvdrawingutils.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <time.h>

#include <aruco.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;
using namespace aruco;

cv::Mat __resize(const cv::Mat &in, int width) {
  if (in.size().width <= width)
    return in;
  float yf = float(width) / float(in.size().width);
  cv::Mat im2;
  cv::resize(in, im2, cv::Size(width, static_cast<int>(in.size().height * yf)));
  return im2;
}

// class for parsing command line
class CmdLineParser {
  int argc;
  char **argv;

public:
  CmdLineParser(int _argc, char **_argv) : argc(_argc), argv(_argv) {}
  bool operator[](string param) {
    int idx = -1;
    for (int i = 0; i < argc && idx == -1; i++)
      if (string(argv[i]) == param)
        idx = i;
    return (idx != -1);
  }
  string operator()(string param, string defvalue = "-1") {
    int idx = -1;
    for (int i = 0; i < argc && idx == -1; i++)
      if (string(argv[i]) == param)
        idx = i;
    if (idx == -1)
      return defvalue;
    else
      return (argv[idx + 1]);
  }
};

FractalDetector FDetector;

void drawImage(cv::Mat &img, cv::Mat &landingPad) {
  std::vector<Marker> Markers;
  Markers = FDetector.getMarkers();

  if (Markers.size() > 0) {
    std::map<int, FractalMarker> id_fmarker =
        FDetector.getConfiguration().fractalMarkerCollection;

    std::vector<cv::Point2f> srcPnts;
    std::vector<cv::Point2f> point2d;
    for (auto m : Markers) {
      for (auto p : id_fmarker[m.id].points) {
        srcPnts.push_back(cv::Point2f(p.x, p.y));
      }
      for (auto p : m)
        point2d.push_back(p);
    }

    cv::Mat H;
    H = cv::findHomography(srcPnts, point2d);
    std::vector<cv::Point2f> fractal{cv::Point2f(-1, 1), cv::Point2f(1, 1),
                                     cv::Point2f(1, -1), cv::Point2f(-1, -1)};

    std::vector<cv::Point2f> dstPnt;
    cv::perspectiveTransform(fractal, dstPnt, H);

    std::vector<cv::Point2f> pts_src;
    pts_src.push_back(cv::Point2f(0, 0));
    pts_src.push_back(cv::Point2f(landingPad.cols, 0));
    pts_src.push_back(cv::Point2f(landingPad.cols, landingPad.cols));
    pts_src.push_back(cv::Point2f(0, landingPad.cols));

    cv::Mat h = findHomography(pts_src, dstPnt);

    // Output image
    cv::Mat im_out;
    // Warp source image to destination based on homography
    cv::warpPerspective(landingPad, im_out, h, img.size());

    cv::Point pts_dst[4];
    for (int i = 0; i < 4; i++)
      pts_dst[i] = dstPnt[i];

    cv::fillConvexPoly(img, pts_dst, 4, cv::Scalar(0), CV_AA);
    img = im_out + img;
  }
}

int main(int argc, char **argv) {
  try {
    CmdLineParser cml(argc, argv);
    if (argc < 2 || cml["-h"]) {
      cerr << "Usage: video.avi landPad.png" << endl;
      return 0;
    }

    cv::Mat InImage;

    VideoCapture vreader;
    vreader.open(argv[1]);
    if (vreader.isOpened())
      vreader >> InImage;
    else {
      cerr << "Could not open input" << endl;
      return -1;
    }

    FDetector.setConfiguration("FRACTAL_5L_6");

    char key = 0;
    int waitTime = 10;

    cv::Mat landingPad = imread(argv[2]);

    // Yujie
    clock_t start, end;
    int frame_num = 0;
    int detected_num = 0;
    double overall_time = 0.0;
    bool cube(false);
    bool axis = true;

    do {
      vreader.retrieve(InImage);
      __resize(InImage, 1000).copyTo(InImage);

      cv::Mat InImageCopy;
      InImage.copyTo(InImageCopy);

      start = clock();
      bool fractal_detected = FDetector.detect(InImage);
      end = clock();
      float lapsed_time = (float)(end - start) * 1000 / CLOCKS_PER_SEC;
      printf("lapsed time= %f ms\n", lapsed_time);
      frame_num++;
      overall_time += lapsed_time;

      // Ok, let's detect
      if (fractal_detected) {
        cout << "Detected fractal markers" << endl;
        FDetector.drawMarkers(InImage);
        detected_num++;
      }

      // Draw inners corners and show fractal marker
      FDetector.draw2d(InImage);
      // FDetector.draw3d(InImage, cube, axis);
      cv::imshow("Fractal", InImage);

      // Draw landingPad (at least one marker is detected)
      drawImage(InImage, landingPad);
      cv::imshow("LandigPad", InImage);

      key = cv::waitKey(waitTime); // wait for key to be pressed

      if (key == 's')
        waitTime = waitTime == 0 ? 10 : 0;

      if (vreader.grab() == false)
        key = 27;

    } while (key != 27);

    cout << "Average runtime : " << overall_time / frame_num
         << " ms on FRACTAL_5L_6" << endl;
    cout << detected_num << "/" << frame_num << " frames detected" << endl;

  } catch (std::exception &ex) {
    cout << "Exception :" << ex.what() << endl;
  }
}
