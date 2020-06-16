#ifndef SHAPESCANNER_SCANNER_H
#define SHAPESCANNER_SCANNER_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <iostream>
#include <cmath>
#include "utilis.h"

using namespace cv;
using namespace std;

struct Circle{
  Point2f center;
  float radius;
};

class ShapeScanner {
private:
  string _imgPath = "";
  int _lowThreshold;

  Mat _srcImg;
  Mat _dstImg;
  Mat _edImg;

  vector<RotatedRect> _rectangles;
  vector<vector<Point>> _triangles;
  vector<Circle> _circles;

public:
  ShapeScanner(const string imgPath, const int lowThreshold) : _imgPath(imgPath), _lowThreshold(lowThreshold) {}

  void scanShapes() {
    vector<vector<Point>> contours;
    vector<Point> vertices;

    int ratio = 3;
    int kernelSize = 3;

    int rec = 0;
    int squ = 0;

    _srcImg = imread(_imgPath);
    Mat grayImg;
    cvtColor(_srcImg, grayImg, CV_BGR2GRAY);
    Mat cannyImg;
    Canny(grayImg, cannyImg, _lowThreshold, ratio * _lowThreshold, kernelSize);

    findContours(cannyImg.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    _dstImg = _srcImg.clone();

    auto setLabel= [&](const string& label, vector<Point>& contour) {
        int fontFace = FONT_HERSHEY_SIMPLEX;
        double scale = 1;
        int thickness = 1;
        int baseline = 0;

        Size text = getTextSize(label, fontFace, scale, thickness, &baseline);
        Rect r = boundingRect(contour);
        Point barycentricPt(r.x + (r.width / 2), r.y + (r.height) /2);
        Point labelPt(r.x , r.y - 8);
        putText(_dstImg, label, labelPt, fontFace, scale, Scalar(0, 0, 255), 1);
        if (label != "TRI")
          circle(_dstImg, barycentricPt, 2, Scalar(0, 255, 0), 2);
    };

    auto angleCos = [&](Point pt1, Point pt2, Point pt3) -> double {
      double dx1 = pt1.x - pt3.x;
      double dy1 = pt1.y - pt3.y;
      double dx2 = pt2.x - pt3.x;
      double dy2 = pt2.y - pt3.y;

      return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2));
    };

    for (auto& contour : contours) {
      approxPolyDP(Mat(contour), vertices, arcLength(Mat(contour), true) * 0.02, true);

      if (fabs(contourArea(contour)) < 100 || !isContourConvex(vertices))
        continue;

      if (vertices.size() == 3) {
        _triangles.emplace_back(vertices);
        Point pt((vertices[1].x + vertices[0].x + vertices[2].x) / 3,
                 (vertices[1].y + vertices[0].y + vertices[2].y) / 3);
        circle(_dstImg, pt, 2, Scalar(0, 255, 255), 2);

        for (size_t i = 0; i < 3; ++i)
          line(_dstImg, vertices[i], vertices[(i + 1) % 3], Scalar(255, 0, 0), 3);

        setLabel("TRI", contour);
      }

      else if (vertices.size() == 4) {
       vector<double> cos;
       for (int j = 2; j < 4; ++j)
         cos.emplace_back(angleCos(vertices[j % 4], vertices[j - 2], vertices[j - 1]));
       sort(cos.begin(), cos.end());
       double minCos = cos.front();
       double maxCos = cos.back();

       if (minCos >= -0.3 && maxCos <= 0.3) {
         RotatedRect rotRect = minAreaRect(contour);
         _rectangles.emplace_back(rotRect);

         Point2f rects[4];
         rotRect.points(rects);

         auto recLine = [&](Scalar& scalar) {
           for (int i = 0; i < 4; ++i)
             line(_dstImg, rects[i], rects[(i + 1) % 4], scalar, 3);
         };

         string rotLabel = "";

         float mirr = abs(abs(rects[3].x - rects[1].x) - abs(rects[2].y - rects[0].y));

//          LyxUtilis::log(mirr);
         if (mirr < 4) {
           rotLabel = "squ";
           squ++;
           Scalar scalar(0, 255, 0);
           recLine(scalar);
         } else {
           rotLabel == "rec";
           rec++;
           Scalar scalar(0, 0, 255);
           recLine(scalar);
         }

         setLabel(rotLabel, contour);
       }
      }
      else {
        double area = contourArea(contour);
        Rect r = boundingRect(contour);
        int radius = r.width / 2;
        if (abs(1 - ((double)r.width / r.height)) <= 0.2 && abs(1 - (area / (CV_PI * pow(radius, 2)))) <= 0.2) {
          Circle circle;
          minEnclosingCircle(contour, circle.center, circle.radius);
          _circles.emplace_back(circle);
          cv::circle(_dstImg, circle.center, circle.radius, Scalar(0, 255, 255) , 3);
          setLabel("CIR", contour);
        }
      }
    }

    _edImg = cannyImg;
    LyxUtilis::log("triangles: ", _triangles.size(), "rec: ", rec, "squ: ", squ, "circles: ", _circles.size());
  }

  Mat& return_dst() {
    return _dstImg;
  }
  Mat& return_ed() {
    return _edImg;
  }
};
#endif // SHAPESCANNER_SCANNER_H



