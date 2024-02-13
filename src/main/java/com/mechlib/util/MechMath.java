package com.mechlib.util;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * MechLib MechMath class
 *
 * Contains some math utility methods used throughout MechLib
 */
public class MechMath {
  /**
   * Calculates the optimal rotation to minimize travel distance
   *
   * @param curRotation Current rotation [-pi, pi]
   * @param desiredRotation Desired rotation [-pi, pi]
   *
   * @return Optimal rotation (-2pi, 2pi)
   */
  public static Rotation2d optimizeRotation(Rotation2d curRotation, Rotation2d desiredRotation) {
    // Initialize optimal rotation to desired rotation
    Rotation2d optimalRotation = desiredRotation;

    // Calculate distance to desired rotation
    double distance = Math.abs(desiredRotation.getRadians() - curRotation.getRadians());

    // Check if the distance is more than pi radians away
    if (distance > Math.PI) {
      // If so wrap optimal rotation around
      optimalRotation = new Rotation2d(
        2 * Math.PI * Math.signum(curRotation.getRadians()) + desiredRotation.getRadians()
      );
    }

    // Return the optimal rotation
    return optimalRotation;
  }
}
