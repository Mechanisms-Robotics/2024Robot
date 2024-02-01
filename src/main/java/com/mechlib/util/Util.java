package com.mechlib.util;

/**
 * MechLib Util class
 *
 * Contains utility methods
 */
public class Util {
  /**
   * Returns a clamped value between min and max
   *
   * @param value Value to clamp
   * @param min Min value
   * @param max Max value
   *
   * @return Clamped value
   */
  public static double clamp(double value, double min, double max) {
    double ret = Math.max(value, min);
    ret = Math.min(ret, max);

    return ret;
  }

  /**
   * Linear interpolation method
   *
   * @param x1 Min x
   * @param y1 Min y
   * @param x2 Max x
   * @param y2 Max y
   * @param x3 Cur x
   *
   * @return Cur y
   */
  public static double lerp(double x1, double y1, double x2, double y2, double x3) {
    return y1 + (x3 - x1) * ((y2 - y1) / (x2 - x1));
  }
}
