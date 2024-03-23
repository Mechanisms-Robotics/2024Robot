package com.mechlib.swerve;

import frc.robot.Robot;

/**
 * MechLib HeadingControllerConfiguration
 *
 * Contains all the configuration parameters for a SwerveDrive heading controller
 */
// TODO: tune the heading controller configuration so that it does not over stablize when the robot turns fast
public class HeadingControllerConfiguration {
  // Default HeadingControllerConfiguration
  public static final HeadingControllerConfiguration DEFAULT = new HeadingControllerConfiguration(
    6, 0, 0.01,
    Math.toRadians(1.0),
    Robot.isReal() ? 0 : 5.0, 0, 0,
    Math.toRadians(1.0),
    Math.PI,
    2 * Math.PI
  );

  public final double stabilizeKP; // Stabilization P gain
  public final double stabilizeKI; // Stabilization I gain
  public final double stabilizeKD; // Stabilization D gain

  public final double stabilizeTolerance; // Stabilization PID tolerance (rads)

  public final double lockKP; // Lock P gain
  public final double lockKI; // Lock I gain
  public final double lockKD; // Lock D gain

  public final double lockTolerance; // Lock PID tolerance (rads)

  public final double maxAngularVelocity; // (rads/s)
  public final double maxAngularAcceleration; // (rads/s^2)

  /**
   * HeadingControllerConfiguration constructor
   *
   * @param stabilizeKP Stabilization P gain
   * @param stabilizeKI Stabilization I gain
   * @param stabilizeKD Stabilization D gain
   *
   * @param stabilizeTolerance Stabilization PID tolerance (rads)
   *
   * @param lockKP Lock P gain
   * @param lockKI Lock I gain
   * @param lockKD Lock D gain
   *
   * @param lockTolerance Lock PID tolerance (rads)
   *
   * @param maxAngularVelocity Max angular velocity (rads/s)
   * @param maxAngularAcceleration Max angular acceleration (rads/s^2)
   */
  public HeadingControllerConfiguration(
    double stabilizeKP,
    double stabilizeKI,
    double stabilizeKD,

    double stabilizeTolerance,

    double lockKP,
    double lockKI,
    double lockKD,

    double lockTolerance,

    double maxAngularVelocity,
    double maxAngularAcceleration
  ) {
    // Set stabilization PID gains
    this.stabilizeKP = stabilizeKP;
    this.stabilizeKI = stabilizeKI;
    this.stabilizeKD = stabilizeKD;

    // Set stabilization tolerance
    this.stabilizeTolerance = stabilizeTolerance;

    // Set lock PID gains
    this.lockKP = lockKP;
    this.lockKI = lockKI;
    this.lockKD = lockKD;

    // Set lock tolerance
    this.lockTolerance = lockTolerance;

    // Set max angular velocity and acceleration
    this.maxAngularVelocity = maxAngularVelocity;
    this.maxAngularAcceleration = maxAngularAcceleration;
  }
}
