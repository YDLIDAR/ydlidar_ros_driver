/*********************************************************************************
 * Copyright (C) 2018, EAIBOT team
 * @date   Created       : 2018-05-15 10:12:34
 *         Last Modified :
 * @author Tony <chushuifurong618@eaibot.com>
*********************************************************************************/

/**
 * @file   line.h
 * @brief  This class describes geometric line.
 * cartesian coordinate line : a*x + b*y + c = 0
 * polar coordinate line: x*cos(theta) + y*sin(theta) = rho
 */

#ifndef LASER_FUSE_LINE_H_
#define LASER_FUSE_LINE_H_

#include "Point2D.h"

class Line {
 public:
  Line();
  explicit Line(Point2D start_point, Point2D end_point);
  explicit Line(double theta, double rho);
  ~Line();

  double getDistanceFromPoint(Point2D point) const;
  double getLength() const;
  Point2D getPoint(double angle) const;
  double getX(double y);
  double getY(double x);
  double getRho() const {
    return rho_;
  }
  double getTheta() const {
    return theta_;
  }

  void normalizeAngle(double &angle) const;
  void operator = (const Line &l);

 private:
  double theta_;
  double cos_theta_;
  double sin_theta_;
  double rho_;
};

#endif //LASER_FUSE_LINE_H_
