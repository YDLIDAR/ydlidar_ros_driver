#ifndef NOISEFILTER_H
#define NOISEFILTER_H

#include <stdint.h>
#include <vector>
#include "src/CYdLidar.h"


class NoiseFilter {
 public:
  NoiseFilter();
  ~NoiseFilter();
  /**
   * @brief 雷达拖尾过滤函数
   * @details 通过输入一圈原始雷达数据，经过滤波处理后，输出已经滤波后的一圈雷达数据
   * @param[in] in  雷达原数据
   * @param[out] out 滤波后数据
   */
  void filters(const LaserScan &in, LaserScan &out);

 private:
  /**
   * @brief 计算两个极坐标点之间的直线倾斜角度
   * @param reading1  雷达极点一距离
   * @param angle1    雷达极点一角度
   * @param reading2  雷达极点二距离
   * @param angle2    雷达极点二角度
   * @return　两点倾斜角度
   */
  double getTargtAngle(double reading1, double angle1,
                       double reading2, double angle2);

  /**
   * @brief 计算坐标原点到几个极点线段的距离
   * @param reading1    雷达极点一距离
   * @param angle1      雷达极点一角度
   * @param reading2    雷达极点一距离
   * @param angle2      雷达极点一角度
   * @return 雷达坐标原点到两个极点线段之间的距离
   */
  double calculateTargetOffset(double reading1, double angle1,
                               double reading2, double angle2);

  /**
   * @brief 是否当前距离是有效距离
   * @param reading 雷达距离
   * @return 是否当前雷达距离是有效距离
   * @retval true 当前距离是有效雷达距离
   * @retval false 当前雷达距离无效
   */
  bool isRangeValid(double reading) const;

  int maskedNeighbours;///< 雷达多少个临近点考虑滤波策略

};
#endif
