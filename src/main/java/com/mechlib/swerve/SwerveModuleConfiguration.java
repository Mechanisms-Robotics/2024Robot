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

  public final double driveKP; // Drive p gain
  public final double driveKI; // Drive i gain
  public final double driveKD; // Drive d gain
  public final double driveKF; // Drive feedforward (percent)

  public final double driveTolerance; // Drive tolerance (m/s)

  public final double driveGearRatio; // Drive gear ratio (6.12:1)
  public final double wheelDiameter; // Wheel diameter (meters)

  public final double steerCurrentLimit; // Steer motor current limit (amps)
  public final double steerVoltageComp; // Steer motor voltage compensation (volts)

  public final double driveCurrentLimit; // Drive motor current limit (amps)
  public final double driveVoltageComp; // Drive motor voltage compensation (volts)

  // Default SwerveModuleConfiguration (SDS Mk4i L3 w/ Falcons)
  public static SwerveModuleConfiguration DEFAULT = new SwerveModuleConfiguration(
    0.25,
    0.0,
    0.0,
    0.01,

    Math.toRadians(1.0),

    0.0007298,
    0.0,
    0.0,
    0.0,

    0.1,

    6.75,
    0.1016,

    Double.NaN,
    Double.NaN,

    30.0,
    Double.NaN
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
   * @param driveKP Drive p gain
   * @param driveKI Drive i gain
   * @param driveKD Drive d gain
   * @param driveKF Drive feedforward (percent)
   * @param driveTolerance Drive tolerance (m/s)
   *
   * @param driveGearRatio Gear ratio of drive motor (x:1)
   * @param wheelDiameter Diameter of wheel (meters)
   *
   * @param steerCurrentLimit Steer motor current limit (amps)
   * @param steerVoltageComp Steer motor voltage compensation (volts)

   * @param driveCurrentLimit Drive motor current limit (amps)
   * @param driveVoltageComp Drive motor voltage compensation (volts)
   */
  public SwerveModuleConfiguration(
    double steerKP,
    double steerKI,
    double steerKD,
    double steerKF,

    double steerTolerance,

    double driveKP,
    double driveKI,
    double driveKD,
    double driveKF,

    double driveTolerance,

    double driveGearRatio,
    double wheelDiameter,

    double steerCurrentLimit,
    double steerVoltageComp,

    double driveCurrentLimit,
    double driveVoltageComp
    ) {
    // Set config parameters
    this.steerKP = steerKP;
    this.steerKI = steerKI;
    this.steerKD = steerKD;
    this.steerKF = steerKF;

    this.steerTolerance = steerTolerance;

    this.driveKP = driveKP;
    this.driveKI = driveKI;
    this.driveKD = driveKD;
    this.driveKF = driveKF;

    this.driveTolerance = driveTolerance;

    this.driveGearRatio = driveGearRatio;
    this.wheelDiameter = wheelDiameter;

    this.steerCurrentLimit = steerCurrentLimit;
    this.steerVoltageComp = steerVoltageComp;

    this.driveCurrentLimit = driveCurrentLimit;
    this.driveVoltageComp = driveVoltageComp;
  }
}
