package com.mechlib.util;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * MechLib MechUnits class
 *
 * Contains unit conversion methods used throughout MechLib
 */
public class MechUnits {
  /**
   * Converts rotations to radians
   *
   * @param rotations Rotations
   *
   * @return Radians
   */
  public static double rotationsToRadians(double rotations) {
    // radians = rotations * 2pi
    return Rotation2d.fromRotations(rotations).getRadians();
  }

  /**
   * Converts rotations to radians
   *
   * @param rotations Rotations
   * @param gearRatio Gear ratio (x:1)
   *
   * @return Radians
   */
  public static double rotationsToRadians(double rotations, double gearRatio) {
    // radians = (rotations / gearRatio) * 2pi
    return rotationsToRadians(rotations / gearRatio);
  }

  /**
   * Converts radians to rotations
   *
   * @param radians Radians
   *
   * @return Rotations
   */
  public static double radiansToRotations(double radians) {
    // rotations = radians / (2pi)
    return new Rotation2d(radians).getRotations();
  }

  /**
   * Converts radians to rotations
   *
   * @param radians Radians
   * @param gearRatio Gear ratio (x:1)
   *
   * @return Rotations
   */
  public static double radiansToRotations(double radians, double gearRatio) {
    // rotations = (radians * gearRatio) / (2pi)
    return radiansToRotations(radians * gearRatio);
  }

  /**
   * Converts rotations to meters
   *
   * @param rotations Rotations
   * @param gearRatio Gear ratio (x:1)
   * @param wheelDiameter Wheel diameter
   *
   * @return Meters
   */
  public static double rotationsToMeters(double rotations, double gearRatio, double wheelDiameter) {
    // meters = (rotations / gearRatio) * (pi * wheelDiameter)
    return (rotations / gearRatio) * (Math.PI * wheelDiameter);
  }

  /**
   * Converts rpm to m/s
   *
   * @param rpm Rotations per minute
   * @param gearRatio Gear ratio (x:1)
   * @param wheelDiameter Wheel diameter
   *
   * @return Meters per second
   */
  public static double rpmToMPS(double rpm, double gearRatio, double wheelDiameter) {
    // mps = ((rpm / 60) / gearRatio) * (pi * wheelDiameter)
    return rotationsToMeters(rpm / 60, gearRatio, wheelDiameter);
  }
}
