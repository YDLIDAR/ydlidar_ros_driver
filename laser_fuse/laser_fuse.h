/*********************************************************************************
 * Copyright (C) 2018, EAIBOT team
 * @date   Created       : 2018-05-08 15:09:38
 *         Last Modified :
 * @author Tony <chushuifurong618@eaibot.com>
*********************************************************************************/

/**
 * @file   laser_fuse.h
 * @brief  This class fuse two laser scan to one.
 *
 *
 */

#ifndef LASER_FUSE_H_
#define LASER_FUSE_H_

#include "line.h"
#include <vector>
#include <map>
#include <set>
#include <limits>// std::numeric_limits
#include <sensor_msgs/LaserScan.h>
#include <core/common/ydlidar_datatype.h>

#define LASER_NUMBER 1
#define RAD2DEG(x) (x)*180/M_PI
#define DEG2RAD(x) (x)*M_PI/180




class LaserFuse {
 public:

  struct Laser {
   public:
    Laser(double d, Point2D p)
      : angle(0),
        distance(d),
        point(p) { }
    Laser() {}
    ~Laser() {}

    double angle;
    double distance;
    Point2D point;
  };

  struct LaserLine {
    std::set<double> angles;
    Line   line;
  };


  LaserFuse();
  ~LaserFuse();


  void processLaser(const LaserScan &scan,
                    sensor_msgs::LaserScan &msg);

 private:
  void polar2Cartesian(const LaserScan &scan,
                       std::vector<Point2D> &points);
  void cartesian2Polar(const std::vector<Point2D> &points,
                       std::map<double, Laser> &laser_data);
  void segmentPDB(const std::map<double, Laser> &laser_data,
                  std::set<double> &break_angles);
  Line fitLineLRM(const std::vector<Point2D> &points);
  void fitLines(const std::map<double, Laser> &laser_data,
                const std::set<double>        &line_angles,
                std::vector<LaserLine>        &laser_lines);
  void findLines(const std::map<double, Laser> &laser_data,
                 std::set<double>              &break_angles,
                 std::set<double>              &line_angles);
  void excludeData(const std::set<double>      &line_angles,
                   std::map<double, Laser>     &laser_data);
  void buildLaserMessage(const std::map<double, Laser> &laser_data,
                         const std::vector<LaserLine>  &laser_lines,
                         sensor_msgs::LaserScan &msg);
  void filterLaser(std::map<double, Laser> &laser_data);

};

#endif //LASER_FUSE_H_
