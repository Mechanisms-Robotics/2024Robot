package com.mechlib.swerve;

import frc.robot.Robot;

/**
 * MechLib HeadingControllerConfiguration
 *
 * Contains all the configuration parameters for a SwerveDrive heading controller
 */
public class HeadingControllerConfiguration {
  // Default HeadingControllerConfiguration
  public static final HeadingControllerConfiguration DEFAULT = new HeadingControllerConfiguration(
    0.25, 0, 0,
    Robot.isReal() ? 0.25 : 5.0, 0, 0
  );

  public final double stabilizeKP; // Stabilization P gain
  public final double stabilizeKI; // Stabilization I gain
  public final double stabilizeKD; // Stabilization D gain

  public final double lockKP; // Lock P gain
  public final double lockKI; // Lock I gain
  public final double lockKD; // Lock D gain

  /**
   * HeadingControllerConfiguration constructor
   *
   * @param stabilizeKP Stabilization P gain
   * @param stabilizeKI Stabilization I gain
   * @param stabilizeKD Stabilization D gain
   *
   * @param lockKP Lock P gain
   * @param lockKI Lock I gain
   * @param lockKD Lock D gain
   */
  public HeadingControllerConfiguration(
    double stabilizeKP,
    double stabilizeKI,
    double stabilizeKD,

    double lockKP,
    double lockKI,
    double lockKD
  ) {
    // Set stabilization PID gains
    this.stabilizeKP = stabilizeKP;
    this.stabilizeKI = stabilizeKI;
    this.stabilizeKD = stabilizeKD;

    // Set lock PID gains
    this.lockKP = lockKP;
    this.lockKI = lockKI;
    this.lockKD = lockKD;
  }
}
