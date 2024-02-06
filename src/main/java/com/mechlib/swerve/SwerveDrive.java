package com.mechlib.swerve;

import com.mechlib.hardware.BrushlessMotorControllerType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Robot;

/**
 * MechLib SwerveDrive class
 *
 * Contains all the code required for a generic Swerve Drive
 */
public class SwerveDrive extends SubsystemBase {
  // Constants
  private static final double MAX_ATTAINABLE_SPEED = 4.5; // (m/s)

  // Modules
  private final SwerveModule flModule; // Front left module
  private final SwerveModule frModule; // Front right module
  private final SwerveModule brModule; // Back right module
  private final SwerveModule blModule; // Back left module

  // Gyro
  private final Pigeon2 gyro;

  // Kinematics
  private final SwerveDriveKinematics kinematics;

  // Pose estimator
  private final SwerveDrivePoseEstimator poseEstimator;

  // Heading controller
  private final PIDController headingController;

  // PID constants struct for the heading controller
  public static class HeadingConstants {
    public static final HeadingConstants DEFAULT = new HeadingConstants(
            0.25, 0, 0
    );

    public final double kP;
    public final double kI;
    public final double kD;

    public HeadingConstants(double kP, double kI, double kD) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
    }
  }

  // Field2d
  private final Field2d field2d = new Field2d();

  // Module Locations
  private final Translation2d[] moduleLocations;

  // Field oriented driving
  private boolean fieldOriented = true;

  private Rotation2d prevOmega = new Rotation2d();
  private Rotation2d prevHeading = new Rotation2d();

  // Simulated heading
  private Rotation2d simHeading = new Rotation2d();

  // Swerve stopped
  private boolean stopped = true;

  /**
   * SwerveDrive constructor
   *
   * @param flSteerMotorID CAN ID of FL steer motor
   * @param flSteerEncoderID CAN ID of FL steer CANCoder
   * @param flDriveMotorID CAN ID of FL drive motor
   * @param flMagnetOffset Absolute position magnet offset of FL encoder
   *
   * @param frSteerMotorID CAN ID of FR steer motor
   * @param frSteerEncoderID CAN ID of FR steer CANCoder
   * @param frDriveMotorID CAN ID of FR drive motor
   * @param frMagnetOffset Absolute position magnet offset of FR encoder
   *
   * @param brSteerMotorID CAN ID of BR steer motor
   * @param brSteerEncoderID CAN ID of BR steer CANCoder
   * @param brDriveMotorID CAN ID of BR drive motor
   * @param brMagnetOffset Absolute position magnet offset of BR encoder
   *
   * @param blSteerMotorID CAN ID of BL steer motor
   * @param blSteerEncoderID CAN ID of BL steer CANCoder
   * @param blDriveMotorID CAN ID of BL drive motor
   * @param blMagnetOffset Absolute position magnet offset of BL encoder
   *
   * @param driveBrushlessMotorControllerType Drive brushless motor controller type
   * @param steerBrushlessMotorControllerType Steer brushless motor controller type
   *
   * @param moduleConfiguration Swerve module configuration
   *
   * @param flModuleLocation FL Module location (meters relative to center of robot)
   * @param frModuleLocation FR Module location (meters relative to center of robot)
   * @param brModuleLocation BR Module location (meters relative to center of robot)
   * @param blModuleLocation BL Module location (meters relative to center of robot)
   *
   * @param gyroID CAN ID of gyro
   */
  public SwerveDrive(
    int flSteerMotorID,
    int flSteerEncoderID,
    int flDriveMotorID,
    double flMagnetOffset,

    int frSteerMotorID,
    int frSteerEncoderID,
    int frDriveMotorID,
    double frMagnetOffset,

    int brSteerMotorID,
    int brSteerEncoderID,
    int brDriveMotorID,
    double brMagnetOffset,

    int blSteerMotorID,
    int blSteerEncoderID,
    int blDriveMotorID,
    double blMagnetOffset,

    BrushlessMotorControllerType driveBrushlessMotorControllerType,
    BrushlessMotorControllerType steerBrushlessMotorControllerType,

    SwerveModuleConfiguration moduleConfiguration,

    Translation2d flModuleLocation,
    Translation2d frModuleLocation,
    Translation2d brModuleLocation,
    Translation2d blModuleLocation,

    int gyroID,

    HeadingConstants headingConstants
  ) {
    // Instantiate FL module
    flModule = new SwerveModule(
      "FL Module",
      flSteerMotorID,
      flSteerEncoderID,
      flDriveMotorID,
      flMagnetOffset,
      driveBrushlessMotorControllerType,
      steerBrushlessMotorControllerType,
      moduleConfiguration
    );

    // Instantiate FR module
    frModule = new SwerveModule(
      "FR Module",
      frSteerMotorID,
      frSteerEncoderID,
      frDriveMotorID,
      frMagnetOffset,
      driveBrushlessMotorControllerType,
      steerBrushlessMotorControllerType,
      moduleConfiguration
    );

    // Instantiate BR module
    brModule = new SwerveModule(
      "BR Module",
      brSteerMotorID,
      brSteerEncoderID,
      brDriveMotorID,
      brMagnetOffset,
      driveBrushlessMotorControllerType,
      steerBrushlessMotorControllerType,
      moduleConfiguration
    );

    // Instantiate BL module
    blModule = new SwerveModule(
      "BL Module",
      blSteerMotorID,
      blSteerEncoderID,
      blDriveMotorID,
      blMagnetOffset,
      driveBrushlessMotorControllerType,
      steerBrushlessMotorControllerType,
      moduleConfiguration
    );

    // Instantiate gyro
    gyro = new Pigeon2(gyroID);

    // Instantiate kinematics
    kinematics = new SwerveDriveKinematics(
      flModuleLocation,
      frModuleLocation,
      brModuleLocation,
      blModuleLocation
    );

    // Instantiate pose estimator
    poseEstimator = new SwerveDrivePoseEstimator(
      kinematics,
      getHeading(),
      getModulePositions(),
      new Pose2d()
    );

    // Instantiate heading controller
    headingController = new PIDController(
            headingConstants.kP,
            headingConstants.kI,
            headingConstants.kD
    );

    // Put Field2d on SmartDashboard
    SmartDashboard.putData("Field", field2d);

    // Instantiate module locations array
    moduleLocations = new Translation2d[] {
      flModuleLocation,
      frModuleLocation,
      brModuleLocation,
      blModuleLocation
    };
  }

  /**
   * Zeroes gyro
   */
  public void zeroGyro() {
    // Check if this is a simulation
    if (Robot.isSimulation())
      // If so reset simulated heading
      simHeading = new Rotation2d();

    // Set gyro yaw to 0
    gyro.setYaw(0.0);
  }

  /**
   * Gets the current heading
   *
   * @return Current heading
   */
  public Rotation2d getHeading() {
    // Check if this is a simulation
    if (Robot.isSimulation())
      // If so return simulated heading
      return simHeading;

    // Get the angle from the gyro and create a Rotation2d with it
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  /**
   * Set whether to drive field oriented
   *
   * @param fieldOriented Field oriented driving
   */
  public void setFieldOriented(boolean fieldOriented) {
    // Set fieldOriented
    this.fieldOriented = fieldOriented;
  }

  /**
   * Toggles field oriented driving
   */
  public void toggleFieldOriented() {
    // Toggle fieldOriented
    this.fieldOriented = !fieldOriented;
  }

  /**
   * Gets the swerve module positions (in order of FL, FR, BR, FL)
   *
   * @return Swerve module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    // Create SwerveModulePosition array with all module positions, then return it
    return new SwerveModulePosition[] {
      flModule.getModulePosition(),
      frModule.getModulePosition(),
      brModule.getModulePosition(),
      blModule.getModulePosition()
    };
  }

  /**
   * Sets all module states
   *
   * @param desiredStates Desired module states
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    flModule.setState(desiredStates[0]);
    frModule.setState(desiredStates[1]);
    brModule.setState(desiredStates[2]);
    blModule.setState(desiredStates[3]);
  }

  /**
   * Gets estimated pose of the swerve drive
   *
   * @return Estimated pose
   */
  public Pose2d getEstimatedPose() {
    // Return estimated pose from pose estimator
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Locks wheels in X configuration
   */
  public void lock() {
    // Steer modules in an x configuration
    flModule.steerTo(new Rotation2d( Math.PI/4));
    frModule.steerTo(new Rotation2d( -Math.PI/4));
    brModule.steerTo(new Rotation2d( Math.PI/4));
    blModule.steerTo(new Rotation2d( -Math.PI/4));
  }

  /**
   * Drives the swerve drive given desired velocities
   *
   * @param vx Velocity in the X direction (m/s)
   * @param vy Velocity in the Y direction (m/s)
   * @param omega Angular velocity (rads/s)
   */
  public void drive(double vx, double vy, double omega) {
    // Check if all velocities are zero
    if (vx == 0 && vy == 0 && omega == 0) {
      // Check if stopped flag is set
      if (stopped)
        // If so return
        return;

      // Set stopped flag to true
      stopped = true;
    } else {
      // Set stopped flag to false
      stopped = false;
    }

    // Stabilize heading
    // author: Alex
    double stabilizedOmega = -headingController.calculate(
      getHeading().getRadians(),
      prevHeading.getRadians()
    );

    // Actual omega
    double actualOmega = omega;

    // Check if omega is zero and the translational velocity is greater than 0.25 m/s
    if (omega == 0 && Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2)) >= 0.25) {
      // If so change to stabilized omega
      actualOmega = stabilizedOmega;
    }

    // Output omega to SmartDashboard
    SmartDashboard.putNumber("[Swerve] Omega", actualOmega);

    // Get the desired swerve module states
    SwerveModuleState[] desiredStates =  kinematics.toSwerveModuleStates(
      (
        // Check if field relative is true
        fieldOriented ?
          // If so convert to robot relative speeds
          ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, actualOmega, getHeading()) :
          // Otherwise just use robot relative speeds
          new ChassisSpeeds(vx, vy, actualOmega)
      )
    );

    // Desaturate module speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_ATTAINABLE_SPEED);

    // Set module states to desired states
    setModuleStates(desiredStates);

    // Check if this is a simulation
    if (Robot.isSimulation()) {
      // If so update simulated heading depending on omega
      simHeading = simHeading.rotateBy(new Rotation2d(omega * Robot.kDefaultPeriod));
    }

    SmartDashboard.putNumber("prevheading", prevHeading.getDegrees());

    // Detect change in rotation input
    if (MathUtil.isNear(0.0, omega, 0.01)  && !MathUtil.isNear(0.0, prevOmega.getRadians(), 0.01))
      prevHeading = getHeading();

    prevOmega = Rotation2d.fromRadians(omega);
  }

  /**
   * Draws modules out to Field2d
   */
  private void drawModules() {
    // Draw FL module
    field2d.getObject("FL Module").setPose(
      new Pose2d(
        getEstimatedPose().getTranslation().plus(moduleLocations[0].rotateBy(getHeading())),
        flModule.getModulePosition().angle.rotateBy(getHeading())
      )
    );

    // Draw FR module
    field2d.getObject("FR Module").setPose(
      new Pose2d(
        getEstimatedPose().getTranslation().plus(moduleLocations[1].rotateBy(getHeading())),
        frModule.getModulePosition().angle.rotateBy(getHeading())
      )
    );

    // Draw BR module
    field2d.getObject("BR Module").setPose(
      new Pose2d(
        getEstimatedPose().getTranslation().plus(moduleLocations[2].rotateBy(getHeading())),
        brModule.getModulePosition().angle.rotateBy(getHeading())
      )
    );

    // Draw BL module
    field2d.getObject("BL Module").setPose(
      new Pose2d(
        getEstimatedPose().getTranslation().plus(moduleLocations[3].rotateBy(getHeading())),
        blModule.getModulePosition().angle.rotateBy(getHeading())
      )
    );
  }

  @Override
  public void periodic() {
    // Update pose estimator
    poseEstimator.update(getHeading(), getModulePositions());

    // Put estimated x position out to SmartDashboard
    SmartDashboard.putNumber(
      "[Swerve] Estimated X",
      getEstimatedPose().getX()
    );

    // Put estimated y position out to SmartDashboard
    SmartDashboard.putNumber(
      "[Swerve] Estimated Y",
      getEstimatedPose().getY()
    );

    // Put heading out to SmartDashboard
    SmartDashboard.putNumber(
      "[Swerve] Heading",
      getHeading().getDegrees()
    );

    // Update pose on Field2d
    field2d.setRobotPose(getEstimatedPose());

    // Draw modules to Field2d
    drawModules();
  }
}
