#include "NoiseFilter.h"
#include <math.h>
#include <stdio.h>

#define MAX_OFFSET_DISTANCE 0.2
#define JITTER_OFFSET_DISTANCE 0.05

NoiseFilter::NoiseFilter(): maskedNeighbours(4) {

}

NoiseFilter::~NoiseFilter() {

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

bool NoiseFilter::isRangeValid(double reading) const {
  if (reading >= 0.05 && reading <= 32.0) {
    return true;
  }

  return false;
}


void NoiseFilter::filters(const LaserScan &in, LaserScan &out) {
  //range is empty
  if (in.points.empty()) {
    out = in;
    return;
  }

  std::vector<bool> maskedPoints;///< 滤波标记容器，
  double lastRange = in.points[0].range;///< 第一个雷达点距离
  double lastAngle = in.points[0].angle;///< 第一个雷达点角度
  const int nrPoints = in.points.size();///< 一圈雷达点数
  maskedPoints.resize(nrPoints,
                      false);///< 初始标记所有点　不过滤，如果当前标志位true, 则需滤波，　设置距离到零

  //copy attributes to filtered scan
  out = in;///< 初始化输出数据，
  int pointCount  = 0;///< 统计点数
  double lastValidRange = lastRange;
  double lastValidAngle = lastAngle;
  int minIndex = 0;///< 第一个满足条件索引值
  double maxDistance = 0;///< 满足条件区域中，　雷达最大距离值
  bool isNoise = false;///< 是否满足滤波条件
  float filter_offset = 0.05;///< 满足滤波条件的偏移值

  //遍历一圈雷达数据
  for (int i = 0; i < nrPoints; i++) {
    double current_range = in.points[i].range;///< 当前雷达点距离
    double current_angle = in.points[i].angle;///< 当前雷达点角度

    if(!isRangeValid(lastValidRange)) {
      lastValidRange = current_range;//last distance
      lastValidAngle = current_angle;
      continue;
    }


    /// 判断当前雷达点是否有效
    if (isRangeValid(current_range)) {
      double offset;

      /// 判断上一个点雷达是否有效
      if (isRangeValid(lastRange)) {
        offset = calculateTargetOffset(lastRange, lastAngle, current_range,
                                       current_angle);//calculate offset distance
      } else {/// 上一次有效雷达点
        offset = calculateTargetOffset(lastValidRange, lastValidAngle, current_range,
                                       current_angle);//calculate offset distance
      }

      /// 当前点和最新有效点之间的差值
      double Diff = current_range - lastValidRange;//distance difference

      /// fabs(Diff) > lastValidRange * MAX_OFFSET_DISTANCE
      /// 两个有效点之间的距离差大于20% 是噪点条件之一
      /// fabs(offset) < MAX_OFFSET_DISTANCE
      /// 原点到相邻两个线段之间的距离小于MAX_OFFSET_DISTANCE值是　条件二
      ///
      if (fabs(Diff) > lastValidRange * MAX_OFFSET_DISTANCE &&
          fabs(offset) < MAX_OFFSET_DISTANCE &&
          isRangeValid(lastValidRange)) {/// 满足滤波条件
        isNoise = true;
        filter_offset = fabs(offset) + JITTER_OFFSET_DISTANCE;//原点到线段距离加入一个最大抖动阈值
        maxDistance = current_range;
        maskedPoints[i] = true;
      }

      /// 如果　点到线段的距离值大于　阈值区域值，　则滤波结束．
      if (isNoise && fabs(offset) > filter_offset) {/// 滤波条件失效
        isNoise = false;
      }

      if (isNoise) {/// 统计滤波数据
        if (pointCount == 0) {
          minIndex = i;
        }

        if (current_range > maxDistance) {
          maxDistance = current_range;
        }

        pointCount++;
      } else {/// 滤波失效
        if (pointCount >= 2) {/// 滤波区域点大于２　，　开始滤波
          for (int j = minIndex - maskedNeighbours; j <= i + maskedNeighbours; j++) {
            if (j >= 0 && j < nrPoints) {
              double offset = calculateTargetOffset(in.points[j].range, in.points[j].angle,
                                                    lastValidRange,
                                                    lastValidAngle);//calculate offset distance

              if (in.points[j].range > maxDistance) {
                maxDistance = in.points[j].range;
              }

              if (offset < filter_offset && maxDistance > filter_offset && maxDistance > MAX_OFFSET_DISTANCE) {
                maskedPoints[j] = true;
              }
            }
          }
        }

        /// 滤波失效，　重置变量
        pointCount = 0;
        minIndex = 0;
        maxDistance = 0.0;
      }

      /// 记录最新有效距离和角度
      lastValidRange = current_range;//last distance
      lastValidAngle = current_angle;
    }

    /// 记录上一次　雷达点距离和角度
    lastAngle = current_angle;//last angle
    lastRange = current_range;//last range
  }

  /// 通过滤波标志，　需要滤波的点，　距离重置到零
  //mark all masked points as invalid in scan
  for (unsigned int i = 0; i < in.points.size(); i++) {
    if (maskedPoints[i]) {
      //as we don't have a better error this is an other range error for now
      ///printf("filter point[%f, %f]\n", out.points[i].angle, out.points[i].range);
      out.points[i].range = 0.0;
    }
  }
}
