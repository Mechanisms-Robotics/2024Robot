package com.mechlib.swerve;

import com.mechlib.hardware.BrushlessMotorController;
import com.mechlib.hardware.BrushlessMotorControllerType;
import com.mechlib.hardware.CANCoder;
import com.mechlib.hardware.SparkMax;
import com.mechlib.hardware.TalonFX;
import com.mechlib.util.MechMath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
  // Module name
  private final String moduleName;

  // Steer motor instance
  private final BrushlessMotorController steerMotor;
  // Drive motor instance
  private final BrushlessMotorController driveMotor;

  // Drive feedforward
  private final SimpleMotorFeedforward driveFeedforward =
    new SimpleMotorFeedforward(0.3, 1.0, 0.05); // 2.2544, 0.063528

  // Steer motor inversion
  private final boolean steerInverted;

  // Module configuration
  private final SwerveModuleConfiguration moduleConfiguration;

  private Rotation2d curAngle = new Rotation2d(); // Current angle
  private Rotation2d desiredAngle = new Rotation2d(); // Desired angle

  private boolean driveInverted = false; // Drive inverted

  /**
   * SwerveModule constructor
   *
   * @param moduleName Module name
   *
   * @param steerMotorID CAN ID of steer motor
   * @param steerEncoderID CAN ID of steer CANCoder
   * @param driveMotorID CAN ID of drive motor
   *
   * @param magnetOffset Absolute position magnet offset
   * @param steerInverted Inversion of steer motor
   *
   * @param driveBrushlessMotorControllerType Drive brushless motor controller type
   * @param steerBrushlessMotorControllerType Steer brushless motor controller type
   *
   * @param moduleConfiguration SwerveModuleConfiguration instance
   */
  public SwerveModule(
    String moduleName,

    int steerMotorID,
    int steerEncoderID,
    int driveMotorID,

    double magnetOffset,
    boolean steerInverted,

    BrushlessMotorControllerType driveBrushlessMotorControllerType,
    BrushlessMotorControllerType steerBrushlessMotorControllerType,

    SwerveModuleConfiguration moduleConfiguration
  ) {
    // Set module name
    this.moduleName = moduleName;

    // Check which brushless motor controller type to use for steer
    switch (steerBrushlessMotorControllerType) {
      // Default
      default -> {
        // Instantiate TalonFX
        steerMotor = new TalonFX(steerMotorID, new CANCoder(steerEncoderID, magnetOffset));
      }

      // SparkMax
      case SparkMax -> {
        // Instantiate SparkMax
        steerMotor = new SparkMax(steerMotorID, new CANCoder(steerEncoderID, magnetOffset));
      }
    }

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

    // Set steer inversion
    this.steerInverted = steerInverted;

    // Set module configuration
    this.moduleConfiguration = moduleConfiguration;

    // Configure motors
    configureMotors();
  }

  /**
   * Converts CANCoder units to radians
   *
   * @param canCoderUnits CANCoder units
   *
   * @return Radians
   */
  private double steerCANCoderToRadians(double canCoderUnits) {
    // radians = rotations * pi * 2
    return canCoderUnits * Math.PI * 2;
  }

  /**
   * Converts Falcon units to meters
   *
   * @param falconUnits Falcon units
   *
   * @return Meters
   */
  private double driveFalconToMeters(double falconUnits) {
    // meters = (rotations / gearRatio) * (pi * wheelDiameter)
    return (falconUnits / moduleConfiguration.driveGearRatio) *
      (Math.PI * moduleConfiguration.wheelDiameter);
  }

  /**
   * Converts NEO units to meters (only use for position not velocity)
   *
   * @param neoUnits NEO units
   *
   * @return Position (meters)
   */
  private double driveNEOToMeters(double neoUnits) {
    // meters = (rotations / gearRatio) * (pi * wheelDiameter)
    return (neoUnits / moduleConfiguration.driveGearRatio) *
      (Math.PI * moduleConfiguration.wheelDiameter);
  }

  /**
   * Converts NEO units to m/s
   *
   * @param neoUnits NEO units
   * @return Velocity (m/s)
   */
  private double driveNEOToMPS(double neoUnits) {
    // mps = ((rpm / 60) / gearRatio) * (pi * wheelDiameter)
    return ((neoUnits / 60) / moduleConfiguration.driveGearRatio) *
      (Math.PI * moduleConfiguration.wheelDiameter);
  }

  /**
   * Configures motors
   */
  private void configureMotors() {
    // Set the steer motor inversion and switch it to brake mode
    steerMotor.setInverted(steerInverted);
    steerMotor.brakeMode();

    // Configure the steer motor PPIDF controller
    steerMotor.setKP(moduleConfiguration.steerKP);
    steerMotor.setKI(moduleConfiguration.steerKI);
    steerMotor.setKD(moduleConfiguration.steerKD);
    steerMotor.setKF(moduleConfiguration.steerKF);
    steerMotor.setDirectionalFeedforward(true);

    // Set the steer motor PIDF tolerance
    steerMotor.setTolerance(moduleConfiguration.steerTolerance);

    // Set the steer motor position units function
    steerMotor.setPositionUnitsFunction(this::steerCANCoderToRadians);

    // Set the drive motor inversion and switch it to brake mode
    driveMotor.setInverted(driveInverted);
    driveMotor.brakeMode();

    // Configure the drive motor PIDF controller
    driveMotor.setKP(moduleConfiguration.driveKP);
    driveMotor.setKI(moduleConfiguration.driveKI);
    driveMotor.setKD(moduleConfiguration.driveKD);
    driveMotor.setKF(moduleConfiguration.driveKF);
    driveMotor.setDirectionalFeedforward(false);

    // Set the drive motor PIDF tolerance
    driveMotor.setTolerance(moduleConfiguration.driveTolerance);

    // Set the drive motor units functions
    if (driveMotor instanceof TalonFX) {
      driveMotor.setPositionUnitsFunction(this::driveFalconToMeters);
      driveMotor.setVelocityUnitsFunction(this::driveFalconToMeters);
    } else if (driveMotor instanceof SparkMax) {
      driveMotor.setPositionUnitsFunction(this::driveNEOToMeters);
      driveMotor.setVelocityUnitsFunction(this::driveNEOToMPS);
    }

    // Set steer motor current limit if provided
    if (!Double.isNaN(moduleConfiguration.steerCurrentLimit))
      steerMotor.setCurrentLimit(moduleConfiguration.steerCurrentLimit);

    // Set steer motor voltage compensation if provided
    if (!Double.isNaN(moduleConfiguration.steerVoltageComp))
      steerMotor.setVoltageCompensation(moduleConfiguration.steerVoltageComp);

    // Set drive motor current limit if provided
    if (!Double.isNaN(moduleConfiguration.driveCurrentLimit))
      driveMotor.setCurrentLimit(moduleConfiguration.driveCurrentLimit);

    // Set drive motor voltage compensation if provided
    if (!Double.isNaN(moduleConfiguration.driveVoltageComp))
      driveMotor.setVoltageCompensation(moduleConfiguration.driveVoltageComp);
  }

  /**
   * Returns current module position as a SwerveModulePosition
   *
   * @return Current module transposition
   */
  public SwerveModulePosition getModulePosition() {
    // Create a new SwerveModulePosition and return it
    return new SwerveModulePosition(
      driveMotor.getDistance(),
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
    // Set the drive motor PIDF setpoint
//    driveMotor.setSetpoint(speed * (driveInverted ? -1.0 : 1.0));
    driveMotor.setPercent(speed / 4.5 * (driveInverted ? -1.0 : 1.0));
    System.out.println(driveMotor.getVelocity());
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

      // Set drive inversion
      driveInverted = false;
    } else {
      // Set closestAngle to oppAngle
      closestAngle = oppAngle;

      // Set drive inversion
      driveInverted = true;
    }


    // Optimize closest angle rotation
    closestAngle = MechMath.optimizeRotation(curAngle, closestAngle);

    // Return the closest angle
    return closestAngle;
  }

  @Override
  public void periodic() {
    // Update curAngle
    curAngle = new Rotation2d(steerMotor.getPosition());

    // Output curAngle to SmartDashboard
    SmartDashboard.putNumber("[" + moduleName + "] Steer Angle", curAngle.getDegrees());

    // Optimize desired angle
    desiredAngle = optimizeAngle(desiredAngle);

    // Output desiredAngle to SmartDashboard
    SmartDashboard.putNumber("[" + moduleName + "] Desired Angle", desiredAngle.getDegrees());

    // Set the steer motor PIDF setpoint
    steerMotor.setSetpoint(desiredAngle.getRadians());

    // Run the steer motor PIDF controller
    steerMotor.periodicPIDF(curAngle.getRadians());

    // Get current velocity
    double curVelocity = driveMotor.getVelocity();

    // Check if there is a desired velocity
//    if (!MathUtil.isNear(0.0, driveMotor.getSetpoint(), 0.05)) {
//      // If so run the drive motor PIDF controller
////      driveMotor.periodicPIDF(curVelocity, driveFeedforward);
////      driveMotor.setPercent();
//    } else {
//      // Otherwise stop drive motor
//      driveMotor.setPercent(0.0);
//    }

    // Output module speed to SmartDashboard
    SmartDashboard.putNumber(
      "[" + moduleName + "] Speed",
      curVelocity
    );

    // Output module speed to SmartDashboard
    SmartDashboard.putNumber(
            "[" + moduleName + "] Desired Speed",
            driveMotor.getVelocity()
    );
  }
}
