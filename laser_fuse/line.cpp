/*********************************************************************************
 * Copyright (C) 2018, EAIBOT team
 * @date   Created       : 2018-05-15 10:12:34
 *         Last Modified :
 * @author Tony <chushuifurong618@eaibot.com>
*********************************************************************************/

/**
 * @file   line.cpp
 * @brief  This class describes geometric line.
 *
 *
 */

#include "line.h"

Line::Line() {

}

Line::Line(Point2D start_point, Point2D end_point) {
  // Anlge of end_point is bigger than angle of start_point
  theta_ = atan2(end_point.x() - start_point.x(),
                 start_point.y() - end_point.y());
  cos_theta_ = cos(theta_);
  sin_theta_ = sin(theta_);
  rho_ = end_point.x() * cos_theta_ + end_point.y() * sin_theta_;
}

Line::Line(double theta, double rho) {
  normalizeAngle(theta);
  rho_ = rho;
  theta_ = theta;
  cos_theta_ = cos(theta_);
  sin_theta_ = sin(theta_);
}

Line::~Line() {

};

void Line::operator = (const Line &l) {
  rho_ = l.getRho();
  theta_ = l.getTheta();
  cos_theta_ = cos(theta_);
  sin_theta_ = sin(theta_);
}

double Line::getDistanceFromPoint(Point2D point) const {
  return fabs(point.x() * cos_theta_ + point.y() * sin_theta_ - rho_);
}

Point2D Line::getPoint(double angle) const {
  normalizeAngle(angle);
  double x, y;

  if (fabs(fabs(angle) - M_PI / 2) < 0.01 || fabs(fabs(angle) - M_PI) < 0.01) {
    x = 0.0;
    y = (fabs(sin_theta_) > 1e-2) ? rho_ / sin_theta_ : rho_;
  } else {
    x = (cos(angle) * rho_) / cos(angle - theta_);
    y = sin(angle) * x / cos(angle);
  }

  return Point2D(x, y);
}

void Line::normalizeAngle(double &angle) const {
  if (fabs(angle) > 2 * M_PI) {
    int ratio = (int)angle / M_PI / 2;
    angle -= double(ratio) * 2 * M_PI;
  }

  if (angle > M_PI) {
    angle = M_PI - angle;
  } else if (angle < -M_PI) {
    angle += 2 * M_PI;
  }
}
// double Line::getX(double y) {
//   return
// }
