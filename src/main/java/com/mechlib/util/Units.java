package com.mechlib.util;

/**
 * MechLib Units class
 *
 * Contains unit conversion methods
 */
public class Units {
  private static final double kFalconEPR = 2048.0; // Falcon ticks/rotation

  /**
   * Converts native falcon units (ticks) to meters
   *
   * @param ticks Ticks (native falcon units)
   * @param gearRatio Gear ratio (x:1)
   * @param wheelDiameter Diameter of wheel (meters)
   *
   * @return Meters
   */
  public static double falconTicksToMeters(double ticks, double gearRatio, double wheelDiameter) {
    // ticks / (epr * gearRatio) = rotations
    // rotations * PI * wheelDiameter = meters
    return (ticks / (kFalconEPR * gearRatio)) * Math.PI * wheelDiameter;
  }

  /**
   * Converts meters to native falcon units (ticks)
   *
   * @param meters Value in meters
   * @param gearRatio Gear ratio (x:1)
   * @param wheelDiameter Wheel diameter (meters)
   *
   * @return Value in native falcon units (ticks)
   */
  public static double metersToFalconTicks(double meters, double gearRatio, double wheelDiameter) {
    // meters = (ticks / (epr * gearRatio)) * PI * wheelDiameter
    // meters / (PI * wheelDiameter) = ticks / (epr * gearRatio)
    // ticks = (meters / (PI * wheelDiameter)) * epr * gearRatio
    return (meters / (Math.PI * wheelDiameter)) * kFalconEPR * gearRatio;
  }

  /**
   * Converts native falcon velocity units (ticks/100ms) to meters/second
   *
   * @param ticksPer100ms Ticks/100ms (native falcon velocity units)
   * @param gearRatio Gear ratio (x:1)
   * @param wheelDiameter Diameter of wheel (meters)
   *
   * @return Meters/second
   */
  public static double falconTicksPer100msToMetersPerSecond(
    double ticksPer100ms,
    double gearRatio,
    double wheelDiameter
  ) {
    // ticks/100ms * 10 = ticks/second
    // meters/second = falconTicksToMeters(ticks/second)
    return falconTicksToMeters(
      ticksPer100ms * 10,
      gearRatio,
      wheelDiameter
    );
  }

  /**
   * Converts meters/second to native falcon velocity units (ticks/100ms)
   *
   * @param metersPerSecond Value in meters/second
   * @param gearRatio Gear ratio (x:1)
   * @param wheelDiameter Wheel diameter (meters)
   *
   * @return Value in native falcon velocity units (ticks/100ms)
   */
  public static double metersPerSecondToFalconTicksPer100ms(
    double metersPerSecond,
    double gearRatio,
    double wheelDiameter
  ) {
    // ticks/second = metersToFalconTicks(meters/second)
    // ticks/100ms = ticks/second / 10
    return metersToFalconTicks(
      metersPerSecond,
      gearRatio,
      wheelDiameter
    ) / 10;
  }

  /**
   * Converts native falcon units (ticks) to radians
   *
   * @param ticks Ticks (native falcon units)
   * @param gearRatio Gear ratio (x:1)
   *
   * @return Radians
   */
  public static double falconTicksToRadians(double ticks, double gearRatio) {
    // ticks / (epr * gearRatio) = rotations
    // rotations * (2 * PI) = radians
    return (ticks / (kFalconEPR * gearRatio)) * (2 * Math.PI);
  }

  /**
   * Converts inches to meters
   *
   * @param inches Value in inches
   *
   * @return Value in meters
   */
  public static double inchesToMeters(double inches) {
    // meters = inches * 0.0254
    return inches * 0.0254;
  }
}
