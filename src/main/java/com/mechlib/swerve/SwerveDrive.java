package com.mechlib.swerve;

import com.mechlib.hardware.BrushlessMotorControllerType;
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

  // Kinematics
  private final SwerveDriveKinematics kinematics;

  // Gyro
  private final Pigeon2 gyro;

  // Heading controller
  private final HeadingController headingController;

  // Pose estimator
  private final SwerveDrivePoseEstimator poseEstimator;

  // Field2d
  private final Field2d field2d = new Field2d();

  // Module Locations
  private final Translation2d[] moduleLocations;

  // Field oriented driving
  private boolean fieldOriented = true;

  // Simulated heading
  private Rotation2d simHeading = new Rotation2d();

  // Swerve stopped
  private boolean stopped = true;

  // Heading lock
  private boolean headingLocked = false;
  private Rotation2d desiredHeading = new Rotation2d();

  /**
   * SwerveDrive constructor
   *
   * @param flSteerMotorID CAN ID of FL steer motor
   * @param flSteerEncoderID CAN ID of FL steer CANCoder
   * @param flDriveMotorID CAN ID of FL drive motor
   *
   * @param frSteerMotorID CAN ID of FR steer motor
   * @param frSteerEncoderID CAN ID of FR steer CANCoder
   * @param frDriveMotorID CAN ID of FR drive motor
   *
   * @param brSteerMotorID CAN ID of BR steer motor
   * @param brSteerEncoderID CAN ID of BR steer CANCoder
   * @param brDriveMotorID CAN ID of BR drive motor
   *
   * @param blSteerMotorID CAN ID of BL steer motor
   * @param blSteerEncoderID CAN ID of BL steer CANCoder
   * @param blDriveMotorID CAN ID of BL drive motor
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
   *
   * @param headingControllerConfiguration Heading controller configuration
   */
  public SwerveDrive(
    int flSteerMotorID,
    int flSteerEncoderID,
    int flDriveMotorID,

    int frSteerMotorID,
    int frSteerEncoderID,
    int frDriveMotorID,

    int brSteerMotorID,
    int brSteerEncoderID,
    int brDriveMotorID,

    int blSteerMotorID,
    int blSteerEncoderID,
    int blDriveMotorID,

    BrushlessMotorControllerType driveBrushlessMotorControllerType,
    BrushlessMotorControllerType steerBrushlessMotorControllerType,

    SwerveModuleConfiguration moduleConfiguration,

    Translation2d flModuleLocation,
    Translation2d frModuleLocation,
    Translation2d brModuleLocation,
    Translation2d blModuleLocation,

    int gyroID,

    HeadingControllerConfiguration headingControllerConfiguration
  ) {
    // Instantiate FL module
    flModule = new SwerveModule(
      "FL Module",
      flSteerMotorID,
      flSteerEncoderID,
      flDriveMotorID,
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
      driveBrushlessMotorControllerType,
      steerBrushlessMotorControllerType,
      moduleConfiguration
    );

    // Instantiate kinematics
    kinematics = new SwerveDriveKinematics(
      flModuleLocation,
      frModuleLocation,
      brModuleLocation,
      blModuleLocation
    );

    // Instantiate gyro
    gyro = new Pigeon2(gyroID);

    // Instantiate heading controller
    headingController = new HeadingController(headingControllerConfiguration);

    // Instantiate pose estimator
    poseEstimator = new SwerveDrivePoseEstimator(
      kinematics,
      getHeading(),
      getModulePositions(),
      new Pose2d()
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
   * Locks heading to given value
   *
   * @param desiredHeading Desired heading
   */
  public void lockHeading(Rotation2d desiredHeading) {
    // Set heading lock
    this.headingLocked = true;
    this.desiredHeading = desiredHeading;
  }

  /**
   * Aims SwerveDrive at target pose
   *
   * @param target Target translation
   * @param relativeHeading Heading relative to target
   */
  public void aimAt(Translation2d target, Rotation2d relativeHeading) {
    // Calculate error vector
    Translation2d errorVec = target.minus(getEstimatedPose().getTranslation());

    // Calculate target heading
    Rotation2d targetHeading = errorVec.getAngle().rotateBy(relativeHeading);

    // Lock heading to target heading
    lockHeading(targetHeading);
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

    // Initialize actualOmega
    double actualOmega;

    // Check if headingLocked is true
    if (headingLocked) {
      // Lock heading
      actualOmega = headingController.lock(
        getHeading(),
        desiredHeading
      );

      // Check if omega is non-zero
      if (omega != 0)
        // If so disable heading lock
        headingLocked = false;
    } else {
      // Stabilize heading
      actualOmega = headingController.stabilize(
        vx,
        vy,
        omega,
        getHeading()
      );
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
      simHeading = simHeading.rotateBy(new Rotation2d(actualOmega * Robot.kDefaultPeriod));
    }
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
