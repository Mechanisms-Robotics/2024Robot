package com.mechlib.swerve;

import com.mechlib.hardware.BrushlessMotorController;
import com.mechlib.hardware.BrushlessMotorControllerType;
import com.mechlib.hardware.SparkMax;
import com.mechlib.hardware.TalonFX;
import com.mechlib.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * MechLib SwerveModule class
 *
 * Contains all the code required for a generic swerve module
 */
public class SwerveModule extends SubsystemBase {
  // Constants
  private static final double STEER_KP = 0.0025; // P gain
  private static final double STEER_KF = 0.05; // Feedforward (percent)
  private static final double STEER_TOLERANCE = Math.toRadians(1.0); // (rads)

  private static final double DRIVE_KP = 0.0001;
  private static final double DRIVE_KF = 0.05;
  private static final double DRIVE_TOLERANCE = 0.5; // (m/s)
  private static final double DRIVE_MAX_VELOCITY = 4.5; // (m/s)
  private static final double DRIVE_MAX_ACCEL = 2.5; // (m/s^2)

  // Module name
  private final String moduleName;

  // Steer motor instance
  private final BrushlessMotorController steerMotor;
  // Drive motor instance
  private final BrushlessMotorController driveMotor;

  // Drive motor gear ratio
  private final double driveGearRatio;
  // Wheel diameter
  private final double wheelDiameter; // (meters)

  private Rotation2d curAngle = new Rotation2d(); // Current angle
  private Rotation2d desiredAngle = new Rotation2d(); // Desired angle

  private boolean driveInverted = false; // Drive inverted

  /**
   * SwerveModule constructor
   *
   * @param moduleName Module name
   * @param steerMotorID CAN ID of steer motor
   * @param steerEncoderID CAN ID of steer CANCoder
   * @param steerOffset Absolute offset of steer CANCoder
   * @param steerInverted Steer motor inverted
   * @param driveMotorID CAN ID of drive motor
   * @param driveBrushlessMotorControllerType Drive brushless motor controller type
   * @param steerBrushlessMotorControllerType Steer brushless motor controller type
   * @param driveGearRatio Gear ratio of drive motor
   * @param wheelDiameter Diameter of wheel (meters)
   */
  public SwerveModule(
    String moduleName,
    int steerMotorID,
    int steerEncoderID,
    double steerOffset,
    boolean steerInverted,
    int driveMotorID,
    BrushlessMotorControllerType driveBrushlessMotorControllerType,
    BrushlessMotorControllerType steerBrushlessMotorControllerType,
    double driveGearRatio,
    double wheelDiameter
  ) {
    // Set module name
    this.moduleName = moduleName;

    // Check which brushless motor controller type to use for drive
    switch (driveBrushlessMotorControllerType) {
      // Default
      default -> {
        // Instantiate TalonFX
        driveMotor = new TalonFX(driveMotorID);
      }

      // SparkMax
      case SparkMax -> {
        // Instantiate SparkMax
        driveMotor = new SparkMax(driveMotorID);
      }
    }

    // Check which brushless motor controller type to use for drive
    switch (steerBrushlessMotorControllerType) {
      // Default
      default -> {
        // Instantiate TalonFX
        steerMotor = new TalonFX(steerMotorID, steerEncoderID, steerOffset);
      }

      // SparkMax
      case SparkMax -> {
        // Instantiate SparkMax
        steerMotor = new SparkMax(steerMotorID, steerEncoderID, steerOffset);
      }
    }


    // Set the steer motor inversion and switch it to brake mode
    steerMotor.setInverted(steerInverted);
    steerMotor.brakeMode();

    // Configure the steer motor PPIDF controller
    steerMotor.setKP(STEER_KP);
    steerMotor.setKF(STEER_KF);
    steerMotor.setDirectionalFeedforward(true);

    // Set the steer motor PIDF tolerance
    steerMotor.setTolerance(STEER_TOLERANCE);

    // Set the drive motor inversion and switch it to brake mode
    driveMotor.setInverted(driveInverted);
    driveMotor.brakeMode();

    // Configure the drive motor PIDF controller
    driveMotor.setKP(DRIVE_KP);
    driveMotor.setKF(DRIVE_KF);
    driveMotor.setDirectionalFeedforward(true);

    // Set the max velocity and max acceleration for the drive motor PPIDF controller
    driveMotor.setMaxVelocity(
      Units.metersPerSecondToFalconTicksPer100ms(DRIVE_MAX_VELOCITY, driveGearRatio, wheelDiameter)
    );
    driveMotor.setMaxAcceleration(
      Units.metersPerSecondToFalconTicksPer100ms(DRIVE_MAX_ACCEL, driveGearRatio, wheelDiameter)
    );

    // Set the drive motor PIDF tolerance
    driveMotor.setTolerance(DRIVE_TOLERANCE);

    // Set drive gear ratio
    this.driveGearRatio = driveGearRatio;

    // Set wheel diameter
    this.wheelDiameter = wheelDiameter;
  }

  /**
   * Returns current module position as a SwerveModulePosition
   *
   * @return Current module position
   */
  public SwerveModulePosition getModulePosition() {
    // Create a new SwerveModulePosition and return it
    return new SwerveModulePosition(
      Units.falconTicksToMeters(driveMotor.getDistance(), driveGearRatio, wheelDiameter),
      curAngle
    );
  }

  /**
   * Steers swerve module to a desired angle
   *
   * @param desiredAngle Desired angle (Rotation2d)
   */
  public void steerTo(Rotation2d desiredAngle) {
    // Set desired angle
    this.desiredAngle = desiredAngle;
  }

  /**
   * Drives wheel at specified speed
   *
   * @param speed Speed (m/s)
   */
  public void drive(double speed) {
    // Set the drive motor PPIDF setpoint
    driveMotor.setSetpoint(Units.metersPerSecondToFalconTicksPer100ms(
      speed * (driveInverted ? -1.0 : 1.0),
      driveGearRatio,
      wheelDiameter
    ));
  }

  /**
   * Sets the module state
   *
   * @param state SwerveModuleState
   */
  public void setState(SwerveModuleState state) {
    // Steer to state angle
    steerTo(state.angle);

    // Drive at state speed
    drive(state.speedMetersPerSecond);
  }

  /**
   * Optimizes an angle to minimize the steer time
   *
   * @param desiredAngle Angle to Optimize
   *
   * @return Optimized angle
   */
  private Rotation2d optimizeAngle(Rotation2d desiredAngle) {
    // Calculate distance to desired angle
    double desiredDist = Math.abs(desiredAngle.minus(curAngle).getRadians());

    // Get the opposite angle (rotated by 180 degrees)
    Rotation2d oppAngle = desiredAngle.rotateBy(new Rotation2d(Math.PI));
    // Calculate distance to opposite angle
    double oppDist = Math.abs(oppAngle.minus(curAngle).getRadians());

    // Check which angle is closest
    Rotation2d closestAngle;
    if (desiredDist <= oppDist) {
      // Set closestAngle to desiredAngle
      closestAngle = desiredAngle;
    } else {
      // Set closestAngle to oppAngle
      closestAngle = oppAngle;
    }

    // Check if we are in the wrap around zone
    if (curAngle.getRadians() > (Math.PI * 3/4) && desiredAngle.getRadians() < (-Math.PI * 3/4)) {
      // If curAngle is positive wrap around positive
      closestAngle = new Rotation2d(Math.PI + (Math.PI - Math.abs(desiredAngle.getRadians())));
    } else if (curAngle.getRadians() < (-Math.PI * 3/4) && desiredAngle.getRadians() > (Math.PI * 3/4)) {
      // If curAngle is negative wrap around negative
      closestAngle = new Rotation2d(-Math.PI - (Math.PI - Math.abs(desiredAngle.getRadians())));
    }

    // Check if the closest angle was the opposite angle
    if (closestAngle.getRadians() == oppAngle.getRadians()) {
      // If it was invert the drive motor
      driveInverted = true;
    } else {
      driveInverted = false;
    }

    // Return the closest angle
    return closestAngle;
  }

  @Override
  public void periodic() {
    // Update curAngle
    curAngle = Rotation2d.fromDegrees(steerMotor.getPosition());

    // Output curAngle to SmartDashboard
    SmartDashboard.putNumber("[" + moduleName + "] Steer Angle", curAngle.getDegrees());

    // Optimize desired angle
    desiredAngle = optimizeAngle(desiredAngle);

    // Output desiredAngle to SmartDashboard
    SmartDashboard.putNumber("[" + moduleName + "] Desired Angle", desiredAngle.getDegrees());

    // Set the steer motor PIDF setpoint
    steerMotor.setSetpoint(desiredAngle.getDegrees());

    // Run the steer motor PIDF controller
    steerMotor.periodicPIDF(curAngle.getDegrees());

    // Output module speed to SmartDashboard
    SmartDashboard.putNumber(
      "[" + moduleName + "] Speed",
      Units.falconTicksPer100msToMetersPerSecond(
        driveMotor.getVelocity(),
        driveGearRatio,
        wheelDiameter
      )
    );

    // Run the drive motor PPIDF controller
    driveMotor.periodicPPIDF(Units.falconTicksPer100msToMetersPerSecond(
      driveMotor.getVelocity(),
      driveGearRatio,
      wheelDiameter
    ));
  }
}
