#include "NoiseFilter.h"
#include <math.h>
#include <stdio.h>

NoiseFilter::NoiseFilter(): minIncline(0.11), maxIncline(3.0),
  nonMaskedNeighbours(4),
  maskedNeighbours(1), m_Monotonous(false), maskedFilter(true) {
}

NoiseFilter::~NoiseFilter() {

}

double NoiseFilter::calculateInclineAngle(double reading1,
    double reading2,  double angleBetweenReadings) const {
  return atan2(sin(angleBetweenReadings) * reading2,
               reading1 - (cos(angleBetweenReadings) * reading2));
}

double NoiseFilter::getTargtAngle(double reading1, double angle1,
                                  double reading2, double angle2) {
  double reading1_x = reading1 * cos(angle1);
  double reading1_y = reading1 * sin(angle1);
  double reading2_x = reading2 * cos(angle2);
  double reading2_y = reading2 * sin(angle2);
  double dx = reading2_x - reading1_x;
  double dy = reading2_y - reading1_y;
  return atan2(dy, dx);
}
double NoiseFilter::calculateTargetOffset(double reading1,
    double angle1, double reading2, double angle2) {
  double target_angle = getTargtAngle(reading1, angle1, reading2, angle2);
  double cos_inv_angle = cos(-target_angle);
  double sin_inv_angle = sin(-target_angle);
  double reading2_x = reading2 * cos(angle2);
  double reading2_y = reading2 * sin(angle2);
  double offset = reading2_x * sin_inv_angle + reading2_y * cos_inv_angle;
  return offset;
}

bool NoiseFilter::isRangeValid(const LaserConfig &config,
                               double reading) const {
  if (reading >= config.min_range && reading <= config.max_range) {
    return true;
  }

  return false;
}

bool NoiseFilter::isIncreasing(double value) const {
  if (value > 0) {
    return true;
  }

  return false;
}

void NoiseFilter::filter_low(const LaserScan &in, LaserScan &out) {
  //range is empty
  if (in.points.empty()) {
    out = in;
    return;
  }

  std::vector<bool> maskedPoints;
  double lastRange = in.points[0].range;
  double lastAngle = in.points[0].angle;
  const int nrPoints = in.points.size();
  maskedPoints.resize(nrPoints, false);

  //copy attributes to filtered scan
  out = in;
  int pointCount  = 0;
  double lastOffset = 0;
  double lastDistance = lastRange;
  double preAngle = lastAngle;
  double lastDiff = 0;
  int minIndex = 0;
  double minIndexDistance = 0;
  double maxDistance = 0;
  bool isNoise = false;
  float filter_offset = 0.05;

  for (int i = 0; i < nrPoints; i++) {
    double current_range = in.points[i].range;//current lidar distance
    double current_angle = in.points[i].angle;//current lidar angle

    if (isRangeValid(in.config, current_range)) {

      double offset;

      if (isRangeValid(in.config, lastRange)) {
        offset = calculateTargetOffset(lastRange, lastAngle, current_range,
                                       current_angle);//calculate offset distance
      } else {
        offset = calculateTargetOffset(lastDistance, preAngle, current_range,
                                       current_angle);//calculate offset distance
      }

      double Diff = current_range - lastDistance;//distance difference

      if (fabs(Diff) > lastDistance * 0.2 && fabs(offset) < 0.2 &&
          isRangeValid(in.config, lastDistance)) {
        isNoise = true;
        filter_offset = fabs(offset) + 0.05;
        maxDistance = current_range;
        maskedPoints[i] = true;
      }

      if (isNoise && fabs(offset) > filter_offset) {
        isNoise = false;
      }

      if (isNoise) {
        if (pointCount == 0) {
          minIndex = i;
          minIndexDistance = current_range;
        }

        if (current_range > maxDistance) {
          maxDistance = current_range;
        }

        pointCount++;

      } else {
        if (pointCount >= 2) {
          for (int j = minIndex - nonMaskedNeighbours; j <= i + nonMaskedNeighbours;
               j++) {
            if (j >= 0 && j < nrPoints) {
              double offset = calculateTargetOffset(in.points[j].range, in.points[j].angle,
                                                    lastDistance,
                                                    preAngle);//calculate offset distance

              if (in.points[j].range > maxDistance) {
                maxDistance = in.points[j].range;
              }

              if (offset < filter_offset && maxDistance > filter_offset &&
                  maxDistance > 0.2) {
                maskedPoints[j] = true;
              }
            }
          }
        }

        pointCount = 0;
        minIndex = 0;
        maxDistance = 0.0;
      }

      lastOffset = offset;//last offset
      lastDiff = Diff;//last distance difference
      lastDistance = current_range;//last distance
      preAngle = current_angle;
    }

    lastAngle = current_angle;//last angle
    lastRange = current_range;//last range
  }

  //mark all masked points as invalid in scan
  for (unsigned int i = 0; i < in.points.size(); i++) {
    if (maskedPoints[i]) {
      //as we don't have a better error this is an other range error for now
      out.points[i].range = 0.0;
    }
  }
}


void NoiseFilter::filter_strong(const LaserScan &in, LaserScan &out, bool inverted) {
  //range is empty
  if (in.points.empty()) {
    out = in;
    return;
  }

  std::vector<bool> maskedPoints;
  double lastRange = in.points[0].range;
  double lastAngle = in.points[0].angle;
  if(inverted) {
      lastAngle = 2 * M_PI - lastAngle;
  }
  double lastInclineRange = lastRange;
  const int nrPoints = in.points.size();
  maskedPoints.resize(nrPoints, false);
  double lastIncline = 0;
  bool hasFirst = false;

  //copy attributes to filtered scan
  out = in;
  double lastDistance = 0;

  for (int i = 0; i < nrPoints; i++) {
    double current_range = in.points[i].range;//current lidar distance
    double current_angle = in.points[i].angle;//current lidar angle
    if(inverted) {
      current_angle = 2 * M_PI - current_angle;
    }

    if (isRangeValid(in.config,
                     current_range) /*&& isRangeValid(in.config, lastDistance)*/) {

      if (maskedFilter && isRangeValid(in.config, lastDistance)) {
        const double incline = calculateInclineAngle(in.points[i].range, lastDistance,
                               current_angle - lastAngle);

        if (!hasFirst) {
          hasFirst = true;
          lastIncline = incline;
        }

        //this is a filter for false readings that do occur if one scannes over edgeds of objects
        if (incline < minIncline || incline > maxIncline) {
          //mask neighbour points
          for (int j = -maskedNeighbours; j < maskedNeighbours; j++) {
            if ((int(i) + j < 0)
                || ((int(i) + j) > nrPoints)) {
              continue;
            }

            if (i + j - 1 < 0) {
              continue;
            }
            double offset = 0.0;
            if(inverted) {
               offset = calculateTargetOffset(in.points[i + j - 1].range,
                                                  2 * M_PI - in.points[i + j - 1].angle,
                                                  in.points[i + j].range,
                                                  2 * M_PI - in.points[i + j].angle); //calculate offset distance

            } else {
               offset = calculateTargetOffset(in.points[i + j - 1].range,
                                                  in.points[i + j - 1].angle,
                                                  in.points[i + j].range,
                                                  in.points[i + j].angle); //calculate offset distance
            }

            if (offset < 0.2) {
              maskedPoints[i + j] = true;
            }
          }

        }

        if (fabs(lastIncline - incline) > maxIncline - minIncline) {
          //maskedPoints[i] = true;
        }

        lastInclineRange = current_range;
        lastIncline = incline;
      }

      lastDistance = current_range;//last distance
    }

    lastAngle = current_angle;//last angle
    lastRange = current_range;//last range
  }

  //mark all masked points as invalid in scan
  for (unsigned int i = 0; i < in.points.size(); i++) {
    if (maskedPoints[i]) {
      //as we don't have a better error this is an other range error for now
      out.points[i].range = 0.0;
    }
  }
}
