#ifndef NOISEFILTER_H
#define NOISEFILTER_H
#include <core/common/ydlidar_datatype.h>
#include <vector>

class NoiseFilter {

 public:
  NoiseFilter();
  ~NoiseFilter();
  void filter_low(const LaserScan &in, LaserScan &out);
  void filter_strong(const LaserScan &in, LaserScan &out, bool inverted = false);

 private:

  double calculateInclineAngle(double reading1, double reading2,
                               double angleBetweenReadings) const;
  /**
   * @brief getTargtAngle
   * @param reading1
   * @param angle1
   * @param reading2
   * @param angle2
   * @return
   */
  double getTargtAngle(double reading1, double angle1,
                       double reading2, double angle2);

  /**
   * @brief calculateTargetOffset
   * @param reading1
   * @param angle1
   * @param reading2
   * @param angle2
   * @return
   */
  double calculateTargetOffset(double reading1, double angle1,
                               double reading2, double angle2);

  /**
   * @brief isRangeValid
   * @param reading
   * @return
   */
  bool isRangeValid(const LaserConfig &config, double reading) const;

  /**
   * @brief isIncreasing
   * @param value
   * @return
   */
  bool isIncreasing(double value) const;

  /**
   * Defines how many readings next to an invalid reading get marked as invalid
   * */
  double minIncline, maxIncline;
  int nonMaskedNeighbours;
  int maskedNeighbours;
  bool m_Monotonous;
  bool maskedFilter;

};
#endif
