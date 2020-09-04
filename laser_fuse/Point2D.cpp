/*********************************************************************************
 * Copyright (C) 2018, EAIBOT team
 * @date   Created       : 2018-04-18 11:59:12
 *         Last Modified :
 * @author Tony <chushuifurong618@eaibot.com>
*********************************************************************************/

/**
 * @file   Point2D.cpp
 * @brief  This is a class to store 2D point
 *
 *
 */
#include "Point2D.h"

Point2D::Point2D(const Point2D &p) {
  x_ = p.x();
  y_ = p.y();
}

Point2D Point2D::operator + (const Point2D &p) {
  return Point2D(p.x() + x_, p.y() + y_);
}

Point2D Point2D::operator - (const Point2D &p) {
  return Point2D(x_ - p.x(), y_ - p.y());
}

Point2D Point2D::operator * (const double &v) {
  return Point2D(x_ * v, y_ * v);
}

Point2D Point2D::operator * (const int &v) {
  return Point2D(x_ * v, y_ * v);
}

void Point2D::operator = (const Point2D &p) {
  x_ = p.x();
  y_ = p.y();
}

bool Point2D::isEqual(const Point2D &p) {
  if (fabs(p.x() - x_) < 1e-3 && fabs(p.y() - y_) < 1e-3) {
    return true;
  } else {
    return false;
  }
}

bool Point2D::isNear(const Point2D &p, const double threshold) {
  if (this->euclidDistance(p) < threshold) {
    return true;
  } else {
    return false;
  }
}

double Point2D::euclidDistance(const Point2D &p) {
  return sqrt((p.x() - x_) * (p.x() - x_) + (p.y() - y_) * (p.y() - y_));
}
