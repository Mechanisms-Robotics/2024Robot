package com.mechlib.swerve;

import com.mechlib.hardware.BrushlessMotorController;
import com.mechlib.hardware.BrushlessMotorControllerType;
import com.mechlib.hardware.CANCoder;
import com.mechlib.hardware.SparkMax;
import com.mechlib.hardware.TalonFX;
import com.mechlib.util.MechMath;
import com.mechlib.util.MechUnits;
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

  // Steer motor inversion
  private final boolean steerInverted;

  // Module configuration
  private final SwerveModuleConfiguration moduleConfiguration;

  private Rotation2d curAngle = new Rotation2d(); // Current angle
  private Rotation2d desiredAngle = new Rotation2d(); // Desired angle

  private double desiredVelocity = 0.0; // Desired velocity (m/s)

  private boolean driveInverted = false; // Drive inverted
  private boolean driveClosedLoop = false; // Drive in closed-loop mode

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
   * Configures motors
   */
  private void configureMotors() {
    // Set the steer motor inversion and switch it to brake mode
    steerMotor.setInverted(steerInverted);
    steerMotor.brakeMode();

    // Set steer motor current limit if provided
    if (!Double.isNaN(moduleConfiguration.steerCurrentLimit))
      steerMotor.setCurrentLimit(moduleConfiguration.steerCurrentLimit);

    // Set steer motor voltage compensation if provided
    if (!Double.isNaN(moduleConfiguration.steerVoltageComp))
      steerMotor.setVoltageCompensation(moduleConfiguration.steerVoltageComp);

    // Configure the steer motor feedforward controller
    steerMotor.setFeedforwardController(moduleConfiguration.steerFeedforwardController);

    // Set the steer motor units functions
    steerMotor.setPositionUnitsFunction(MechUnits::rotationsToRadians);
    steerMotor.setVelocityUnitsFunction(MechUnits::rotationsToRadians);

    // Configure the steer motor PID controller
    steerMotor.setKP(moduleConfiguration.steerKP);
    steerMotor.setKI(moduleConfiguration.steerKI);
    steerMotor.setKD(moduleConfiguration.steerKD);

    // Set the steer motor PID tolerance
    steerMotor.setTolerance(moduleConfiguration.steerTolerance);
    steerMotor.setContinuous(-Math.PI, Math.PI);

    // Set the drive motor inversion and switch it to brake mode
    driveMotor.setInverted(driveInverted);
    driveMotor.brakeMode();

    // Set drive motor current limit if provided
    if (!Double.isNaN(moduleConfiguration.driveCurrentLimit))
      driveMotor.setCurrentLimit(moduleConfiguration.driveCurrentLimit);

    // Set drive motor voltage compensation if provided
    if (!Double.isNaN(moduleConfiguration.driveVoltageComp))
      driveMotor.setVoltageCompensation(moduleConfiguration.driveVoltageComp);

    // Configure the drive motor feedforward controller
    driveMotor.setFeedforwardController(moduleConfiguration.driveFeedforwardController);

    // Set the drive motor units functions
    if (driveMotor instanceof TalonFX) {
      driveMotor.setPositionUnitsFunction(
        (Double rotations) -> MechUnits.rotationsToMeters(
          rotations,
          moduleConfiguration.driveGearRatio,
          moduleConfiguration.wheelDiameter
        )
      );

      driveMotor.setVelocityUnitsFunction(
        (Double rotations) -> MechUnits.rotationsToMeters(
          rotations,
          moduleConfiguration.driveGearRatio,
          moduleConfiguration.wheelDiameter
        )
      );
    } else if (driveMotor instanceof SparkMax) {
      driveMotor.setPositionUnitsFunction(
        (Double rotations) -> MechUnits.rotationsToMeters(
          rotations,
          moduleConfiguration.driveGearRatio,
          moduleConfiguration.wheelDiameter
        )
      );

      driveMotor.setVelocityUnitsFunction(
        (Double rpm) -> MechUnits.rpmToMPS(
          rpm,
          moduleConfiguration.driveGearRatio,
          moduleConfiguration.wheelDiameter
        )
      );
    }

    // Configure the drive motor PIDF controller
    driveMotor.setKP(moduleConfiguration.driveKP);
    driveMotor.setKI(moduleConfiguration.driveKI);
    driveMotor.setKD(moduleConfiguration.driveKD);

    // Set the drive motor PIDF tolerance
    driveMotor.setTolerance(moduleConfiguration.driveTolerance);
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
   * Returns current module state as a SwerveModuleState
   *
   * @return Current module state
   */
  public SwerveModuleState getModuleState() {
    // Create a new SwerveModuleState and return it
    return new SwerveModuleState(
      driveMotor.getVelocity(),
      curAngle
    );
  }

  /**
   * Sets drive closed-loop flag
   *
   * @param driveClosedLoop Drive closed-loop flag
   */
  public void setDriveClosedLoop(boolean driveClosedLoop) {
    this.driveClosedLoop = driveClosedLoop;
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
   * Drives wheel at specified velocity
   *
   * @param desiredVelocity Desired velocity (m/s)
   */
  public void drive(double desiredVelocity) {
    // Set desired velocity
    this.desiredVelocity = desiredVelocity;

    // Check if drive closed-loop mode is enabled
    if (driveClosedLoop) {
      // Set the drive motor setpoint
      driveMotor.setSetpoint(desiredVelocity * (driveInverted ? -1.0 : 1.0));
    } else {
      // Set the drive motor percentage
      driveMotor.setPercent(desiredVelocity / 4.4 * (driveInverted ? -1.0 : 1.0));
    };
  }

  /**
   * Sets voltage of drive motor
   *
   * @param volts Volts
   */
  public void setVoltage(double volts) {
    driveMotor.setVoltage(volts);
  }

  /**
   * Sets voltage of steer motor
   *
   * @param volts Volts
   */
  public void setSteerVoltage(double volts) {
    steerMotor.setVoltage(volts);
  }

  /**
   * Sets the module state
   *
   * @param state SwerveModuleState
   */
  public void setState(SwerveModuleState state) {
    // Optimize state
    SwerveModuleState desiredState = SwerveModuleState.optimize(state, curAngle);

    // Steer to state angle
    steerTo(desiredState.angle);

    // Drive at state speed
    drive(desiredState.speedMetersPerSecond);
  }

  @Override
  public void periodic() {
    // Update curAngle
    curAngle = new Rotation2d(steerMotor.getAbsolutePosition());

    // Output current angle to SmartDashboard
    SmartDashboard.putNumber("[" + moduleName + "] Current Angle", curAngle.getDegrees());

    // Output desiredAngle to SmartDashboard
    SmartDashboard.putNumber("[" + moduleName + "] Desired Angle", desiredAngle.getDegrees());

    // Set the steer motor PIDF setpoint
    steerMotor.setSetpoint(desiredAngle.getRadians());

    // Run the steer motor PIDF controller
    steerMotor.periodicPIDF(curAngle.getRadians(), steerMotor.getVelocity());

    // Get current velocity
    double curVelocity = driveMotor.getVelocity();

    // Run drive motor PIDF in driveClosedLoop flag is true
    if (driveClosedLoop) {
      driveMotor.periodicPIDF(curVelocity);
    }

    // Output current velocity to SmartDashboard
    SmartDashboard.putNumber(
      "[" + moduleName + "] Current Velocity",
      curVelocity
    );

    // Output desired velocity to SmartDashboard
    SmartDashboard.putNumber(
      "[" + moduleName + "] Desired Velocity",
      desiredVelocity
    );

    // Output velocity error to SmartDashboard
    SmartDashboard.putNumber(
      "[" + moduleName + "] Velocity Error",
      Math.abs(desiredVelocity - curVelocity)
    );
  }
}
