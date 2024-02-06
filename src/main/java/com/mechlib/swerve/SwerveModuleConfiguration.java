package com.mechlib.swerve;

/**
 * MechLib SwerveModuleConfiguration class
 *
 * Contains all the configuration parameters for a swerve module
 */
public class SwerveModuleConfiguration {
  public final double steerKP; // Steer p gain
  public final double steerKI; // Steer i gain
  public final double steerKD; // Steer d gain
  public final double steerKF; // Steer feedforward (percent)

  public final double steerTolerance; // Steer tolerance (rads)

  public final boolean steerInverted; // Steer motor inverted

  public final double driveKP; // Drive p gain
  public final double driveKI; // Drive i gain
  public final double driveKD; // Drive d gain
  public final double driveKF; // Drive feedforward (percent)

  public final double driveTolerance; // Drive tolerance (m/s)

  public final double maxVelocity; // Max velocity (m/s)
  public final double maxAccel; // Max acceleration (m/s^2)

  public final double driveGearRatio; // Drive gear ratio (6.12:1)
  public final double wheelDiameter; // Wheel diameter (meters)

  // Default SwerveModuleConfiguration (SDS Mk4i L3 w/ Falcons)
  public static SwerveModuleConfiguration DEFAULT = new SwerveModuleConfiguration(
          0.25, // 0.5 worked
    0.0,
    0.0,
    0.01,

    Math.toRadians(1.0),

    true,

    0.1,
    0.0,
    0.0,
    0.01,

    0.5,

    3,
    3,

    6.12,
    0.1016
  );

  /**
   * SwerveModuleConfiguration constructor
   *
   * @param steerKP Steer p gain
   * @param steerKI Steer i gain
   * @param steerKD Steer d gain
   * @param steerKF Steer feedforward (percent)
   * @param steerTolerance Steer tolerance (rads)
   *
   * @param steerInverted Steer motor inverted
   *
   * @param driveKP Drive p gain
   * @param driveKI Drive i gain
   * @param driveKD Drive d gain
   * @param driveKF Drive feedforward (percent)
   * @param driveTolerance Drive tolerance (m/s)
   *
   * @param maxVelocity Max velocity (m/s)
   * @param maxAccel Max acceleration (m/s^2)
   *
   * @param driveGearRatio Gear ratio of drive motor (x:1)
   * @param wheelDiameter Diameter of wheel (meters)
   */
  public SwerveModuleConfiguration(
    double steerKP,
    double steerKI,
    double steerKD,
    double steerKF,

    double steerTolerance,

    boolean steerInverted,

    double driveKP,
    double driveKI,
    double driveKD,
    double driveKF,

    double driveTolerance,

    double maxVelocity,
    double maxAccel,

    double driveGearRatio,
    double wheelDiameter
    ) {
    // Set config parameters
    this.steerKP = steerKP;
    this.steerKI = steerKI;
    this.steerKD = steerKD;
    this.steerKF = steerKF;

    this.steerTolerance = steerTolerance;

    this.steerInverted = steerInverted;

    this.driveKP = driveKP;
    this.driveKI = driveKI;
    this.driveKD = driveKD;
    this.driveKF = driveKF;

    this.driveTolerance = driveTolerance;

    this.maxVelocity = maxVelocity;
    this.maxAccel = maxAccel;

    this.driveGearRatio = driveGearRatio;
    this.wheelDiameter = wheelDiameter;
  }
}
