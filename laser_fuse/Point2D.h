/*********************************************************************************
 * Copyright (C) 2018, EAIBOT team
 * @date   Created       : 2018-04-18 11:59:12
 *         Last Modified :
 * @author Tony <chushuifurong618@eaibot.com>
*********************************************************************************/

/**
 * @file   Point2D.h
 * @brief  This is a class to store 2D point
 *
 *
 */

#ifndef BASE_POSE_POINT2D_H_
#define BASE_POSE_POINT2D_H_

#include <math.h>

class Point2D {
 public:
  inline Point2D() : x_(0.0), y_(0.0) {}
  inline Point2D(double x, double y) : x_(x), y_(y) {}
  Point2D(const Point2D &p);
  ~Point2D() {}

  inline double x() const {
    return x_;
  }
  inline double y() const {
    return y_;
  }
  inline void setX(double x) {
    x_ = x;
  }
  inline void setY(double y) {
    y_ = y;
  }

  Point2D operator+(const Point2D &p);
  Point2D operator-(const Point2D &p);
  void operator=(const Point2D &p);
  Point2D operator*(const double &v);
  Point2D operator*(const int &v);

  double euclidDistance(const Point2D &p);
  bool isEqual(const Point2D &p);
  bool isNear(const Point2D &p, const double theshold);

 private:
  double x_;
  double y_;
};

#endif //BASE_POSE_POINT2D_H_
