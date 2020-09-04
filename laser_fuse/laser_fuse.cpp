/*********************************************************************************
 * Copyright (C) 2018, EAIBOT team
 * @date   Created       : 2018-05-08 15:09:38
 *         Last Modified :
 * @author Tony <chushuifurong618@eaibot.com>
*********************************************************************************/

/**
 * @file   laser_fuse.cpp
 * @brief  This class fuse two laser scan to one.
 *
 *
 */

#include "laser_fuse.h"

#define DISTANCE_THRESHOLD 0.05
#define MIN_POINT_IN_LINE 20

static bool isEqual(const Point2D &p, const Point2D &p1) {
  if (fabs(p.x() - p1.x()) < 1e-3 && fabs(p.y() - p1.y()) < 1e-3) {
    return true;
  } else {
    return false;
  }
}

static double euclidDistance(const Point2D &p,
                             const Point2D &p1) {
  return sqrt((p.x() -  p1.x()) * (p.x() -  p1.x()) + (p.y() - p1.y()) *
              (p.y() - p1.y()));
}

static bool isNear(const Point2D &p, const Point2D &p1,
                   const double threshold) {
  if (euclidDistance(p, p1) < threshold) {
    return true;
  } else {
    return false;
  }
}



LaserFuse::LaserFuse() {

}

LaserFuse::~LaserFuse() {
}


void LaserFuse::buildLaserMessage(const std::map<double, Laser> &laser_data,
                                  const std::vector<LaserLine>  &laser_lines,
                                  sensor_msgs::LaserScan &msg) {

  for (auto it = laser_data.begin(); it != laser_data.end(); ++it) {
    int index = int(((it->first) - msg.angle_min) / msg.angle_increment);

    double residual = it->first - (index * msg.angle_increment - msg.angle_min);

    if (residual > msg.angle_increment * 0.5) {
      ++index;
    }

    if (index < msg.ranges.size()) {
      msg.ranges[index] = float(it->second.distance);
      if(msg.ranges[index] < msg.range_min) {
        msg.ranges[index] = 0;//std::numeric_limits<float>::infinity();
      }
    }

  }

  for (size_t i = 0; i < laser_lines.size(); ++i) {
    int all_size = laser_lines[i].angles.size();
    int angles_index = 0;
    for (auto it = laser_lines[i].angles.begin(); it != laser_lines[i].angles.end();
         ++it) {
      angles_index++;
      if (angles_index <  4 || angles_index > all_size - 4) {
        continue;
      }
      int index = int((*it - msg.angle_min) / msg.angle_increment);

      if ((*it - (index * msg.angle_increment - msg.angle_min)) > msg.angle_increment *
          0.5) {
        ++index;
      }

      Point2D p = laser_lines[i].line.getPoint((double)index *
                  msg.angle_increment +
                  msg.angle_min);

      if (index < msg.ranges.size()) {
        float d = sqrt(p.x() * p.x() + p.y() * p.y());
        msg.ranges[index] =  d;
        if(msg.ranges[index] < msg.range_min) {
          msg.ranges[index] = 0;//std::numeric_limits<float>::infinity();
        }
      }

    }
  }
}

void LaserFuse::polar2Cartesian(const LaserScan &scan,
                                std::vector<Point2D> &points) {
  points.clear();

  for (int i = 0; i < scan.points.size(); i++) {
    LaserPoint point = scan.points[i];
    if (point.range < scan.config.min_range) {
      continue;
    }

    double theta = point.angle;
    //polar coordinate to cartesian coordinate in laser coordinate
    double lx = point.range * cos(theta);
    double ly = point.range * sin(theta);
    points.push_back(Point2D(lx, ly));
  }
}


void LaserFuse::cartesian2Polar(const std::vector<Point2D> &points,
                                std::map<double, Laser> &laser_data) {
  for (size_t i = 0; i < points.size(); i++) {
    double angle = atan2(points[i].y(), points[i].x());
    double d = sqrt(points[i].x() * points[i].x() + points[i].y() * points[i].y());
    Laser laser(d, points[i]);
    laser_data.insert(std::pair<double, Laser>(angle, laser));
  }
}

void LaserFuse::excludeData(const std::set<double>      &line_angles,
                            std::map<double, Laser>     &laser_data) {
  auto it = line_angles.begin();

  while (it != line_angles.end()) {
    double start_angle = *it;
    ++it;
    double end_angle = *it;
    laser_data.erase(laser_data.lower_bound(start_angle),
                     laser_data.upper_bound(end_angle));
    ++it;
  }
}

Line LaserFuse::fitLineLRM(const std::vector<Point2D> &points) {
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_x_x = 0.0;
  double sum_y_y = 0.0;
  double sum_x_y = 0.0;

  for (size_t i = 0; i < points.size(); i++) {
    sum_x += points[i].x();
    sum_y += points[i].y();
    sum_x_x += points[i].x() * points[i].x();
    sum_y_y += points[i].y() * points[i].y();
    sum_x_y += points[i].x() * points[i].y();
  }

  double a = sum_x * sum_y_y - sum_y * sum_x_y;
  double b = sum_y * sum_x_x - sum_x * sum_x_y;
  double c = sum_x_y * sum_x_y - sum_x_x * sum_y_y;
  double theta = atan2(b, a);
  double rho = (fabs(a) < 1e-3) ? (-sin(theta) * c / b) : (-c * cos(theta) / a);

  return Line(theta, rho);
}

void LaserFuse::fitLines(const std::map<double, Laser> &laser_data,
                         const std::set<double>        &line_angles,
                         std::vector<LaserLine>        &laser_lines) {
  auto angles_it = line_angles.begin();

  while (angles_it != line_angles.end()) {
    std::vector<Point2D> points;
    LaserLine laser_line;
    double start_angle = *angles_it;
    ++angles_it;
    double end_angle = *angles_it;

    for (auto data_it = laser_data.lower_bound(start_angle);
         data_it != laser_data.upper_bound(end_angle); ++data_it) {
      points.push_back(data_it->second.point);
      laser_line.angles.insert(data_it->first);
    }

    laser_line.line = fitLineLRM(points);

    if (fabs(laser_line.line.getRho()) > 0.6) {
      laser_lines.push_back(laser_line);
    }

    ++angles_it;
  }
}

void LaserFuse::findLines(const std::map<double, Laser> &laser_data,
                          std::set<double>              &break_angles,
                          std::set<double>              &line_angles) {
  auto break_it = break_angles.begin();
  double distance_threshold;

  while (break_it != break_angles.end()) {
    double first_d = laser_data.find(*break_it)->second.distance;

    if (first_d < 1.0) {
      distance_threshold = 0.2 * DISTANCE_THRESHOLD;
    } else if (first_d < 2.0) {
      distance_threshold = 0.3 * DISTANCE_THRESHOLD;
    } else if (first_d < 3.0) {
      distance_threshold = 0.4 * DISTANCE_THRESHOLD;
    } else if (first_d < 4.0) {
      distance_threshold = 0.5 * DISTANCE_THRESHOLD;
    } else if (first_d < 6.0) {
      distance_threshold = 1.2 * DISTANCE_THRESHOLD;
    } else if (first_d < 9.0) {
      distance_threshold = 2 * DISTANCE_THRESHOLD;
    } else if (first_d < 13.0) {
      distance_threshold = 3 * DISTANCE_THRESHOLD;
    } else {
      distance_threshold = 4 * DISTANCE_THRESHOLD;
    }

    auto next_break = break_it;

    if (++next_break == break_angles.end()) {
      break;
    }

    Point2D start_point = laser_data.find(*break_it)->second.point;
    Point2D end_point = laser_data.find(*next_break)->second.point;

    if (isNear(start_point, end_point, 0.1)) {
      ++break_it;
      continue;
    }

    Line raw_line(start_point, end_point);

    double max_d = 1e-10;
    double sum_d = 0.0;
    double max_angle;
    int cluster_point_number = 0;

    for (auto laser_it = laser_data.lower_bound(*break_it);
         laser_it != laser_data.upper_bound(*next_break); ++laser_it) {
      double d = raw_line.getDistanceFromPoint(laser_it->second.point);
      sum_d += d;

      if (d > max_d) {
        max_d = d;
        max_angle = laser_it->first;
      }

      ++cluster_point_number;
    }

    if (isNear(start_point, end_point, 2.5 * sum_d / cluster_point_number)) {
      //May be a circle
      ++break_it;
    } else if ((max_d > distance_threshold) &&
               cluster_point_number > 2 * MIN_POINT_IN_LINE) {
      break_angles.insert(max_angle);
      break_angles.insert(laser_data.upper_bound(max_angle)->first);
    } else if ((max_d < distance_threshold) &&
               cluster_point_number > MIN_POINT_IN_LINE) {
      line_angles.insert(*break_it);
      line_angles.insert(*next_break);
      ++break_it;
      ++break_it;
    } else {
      ++break_it;
    }
  }
}

void LaserFuse::filterLaser(std::map<double, Laser> &laser_data) {
  auto it = laser_data.begin();
  auto before_it = it;
  ++it;
  auto next_it = it;
  ++next_it;

  while (next_it != laser_data.end()) {
    if (it->first - before_it->first < 0.0044 &&
        next_it->first - it->first < 0.0044 &&
        fabs(it->second.distance - before_it->second.distance) < 0.3 &&
        fabs(next_it->second.distance - it->second.distance) < 0.3) {
      it->second.distance = (next_it->second.distance + it->second.distance +
                             before_it->second.distance) / 3;
      it->second.point = Point2D(it->second.distance * cos(it->first),
                                 it->second.distance * sin(it->first));
    }

    ++before_it;
    ++it;
    ++next_it;
  }
}

/* D(r_i, r_i+1) = sqrt(r_i * r_i + r_i+1 * r_i+1 - 2r_i * r_i+1 * cos(angle_increment));
   D_thd = C_0 + C_1*min{r_i, r_i+1}
   D(r_i, r_i+1) < D_thd : not split
*/
void LaserFuse::segmentPDB(const std::map<double, Laser> &laser_data,
                           std::set<double> &break_angles) {
  break_angles.insert(laser_data.begin()->first);

  for (auto it = laser_data.begin(); it != laser_data.end(); ++it) {
    auto next_it = it;
    ++next_it;

    if (next_it == laser_data.end()) {
      break;
    }

    double r = it->second.distance;
    double r_next = next_it->second.distance;
    double d = sqrt(r * r + r_next * r_next - 2 * r * r_next * cos(
                      next_it->first - it->first));
    double threshold;

    if (next_it->first - it->first < DEG2RAD(1)) {
      threshold = 0.2 + 0.5 * cos(next_it->first - it->first);
    } else {
      if (r > r_next) {
        threshold = 0.05 + sqrt(0.5 * (1.0005 - cos(next_it->first - it->first))) *
                    r_next;
      } else {
        threshold = 0.05 + sqrt(0.5 * (1.0005 - cos(next_it->first - it->first))) * r;
      }
    }

    if (d > threshold) {
      break_angles.insert(it->first);
      break_angles.insert(next_it->first);
    }

  }

  break_angles.insert(laser_data.rbegin()->first);
}



void LaserFuse::processLaser(const LaserScan &scan,
                             sensor_msgs::LaserScan &msg) {
  std::vector<Point2D> laser_points;
  polar2Cartesian(scan, laser_points);

  if (laser_points.size()) {
    std::map<double, Laser> laser_data;
    cartesian2Polar(laser_points, laser_data);
    //segmentation
    std::set<double> break_angles;
    segmentPDB(laser_data, break_angles);
    //Find line among all the segmentations
    std::set<double> line_angles;
    findLines(laser_data, break_angles, line_angles);

    //Fitting
    std::vector<LaserLine> laser_lines;
    fitLines(laser_data, line_angles, laser_lines);

    //Excule laser in line from laser_data
    excludeData(line_angles, laser_data);

    //Build laser data
    buildLaserMessage(laser_data, laser_lines, msg);
    laser_data.clear();
    break_angles.clear();
    line_angles.clear();
    laser_lines.clear();
    laser_points.clear();
  }
}
