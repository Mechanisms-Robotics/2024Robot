package com.mechlib.swerve;

import com.mechlib.hardware.BrushlessMotorControllerType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
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
  private static final double MAX_ATTAINABLE_SPEED = 4.25; // (m/s)

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

  // Wheels locked (in X configuration)
  private boolean wheelsLocked = true;

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
   * @param flMagnetOffset Absolute position magnet offset of FL encoder
   * @param flSteerMotorInverted Inversion of FL steer motor
   *
   * @param frSteerMotorID CAN ID of FR steer motor
   * @param frSteerEncoderID CAN ID of FR steer CANCoder
   * @param frDriveMotorID CAN ID of FR drive motor

   * @param frMagnetOffset Absolute position magnet offset of FR encoder
   * @param frSteerMotorInverted Inversion of FR steer motor
   *
   * @param brSteerMotorID CAN ID of BR steer motor
   * @param brSteerEncoderID CAN ID of BR steer CANCoder
   * @param brDriveMotorID CAN ID of BR drive motor

   * @param brMagnetOffset Absolute position magnet offset of BR encoder
   * @param brSteerMotorInverted Inversion of BR steer motor
   *
   * @param blSteerMotorID CAN ID of BL steer motor
   * @param blSteerEncoderID CAN ID of BL steer CANCoder
   * @param blDriveMotorID CAN ID of BL drive motor

   * @param blMagnetOffset Absolute position magnet offset of BL encoder
   * @param blSteerMotorInverted Inversion of BL steer motor
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

    double flMagnetOffset,
    boolean flSteerMotorInverted,

    int frSteerMotorID,
    int frSteerEncoderID,
    int frDriveMotorID,

    double frMagnetOffset,
    boolean frSteerMotorInverted,

    int brSteerMotorID,
    int brSteerEncoderID,
    int brDriveMotorID,

    double brMagnetOffset,
    boolean brSteerMotorInverted,

    int blSteerMotorID,
    int blSteerEncoderID,
    int blDriveMotorID,

    double blMagnetOffset,
    boolean blSteerMotorInverted,

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

      flMagnetOffset,
      flSteerMotorInverted,

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
      frSteerMotorInverted,

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
      brSteerMotorInverted,

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
      blSteerMotorInverted,

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

//    // Configure AutoBuilder holonomic control
    AutoBuilder.configureHolonomic(
      this::getEstimatedPose,
      this::setPose,
      this::getSpeeds,
      this::autoDrive,
      new HolonomicPathFollowerConfig(
        new PIDConstants(7, 0, 0),
        new PIDConstants(5, 0.0, 0.0),

        4,
        0.5388,

        new ReplanningConfig()
      ),
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
    );
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
    headingController.reset();
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
    // (The rotateBy is so that the angle is wrapped)
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()).rotateBy(new Rotation2d());
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
   * Gets the swerve module positions (in order of FL, FR, BR, BL)
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
   * Gets the current swerve module states (in order of FL, FR, BR, BL)
   *
   * @return Swerve module states
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      flModule.getModuleState(),
      frModule.getModuleState(),
      brModule.getModuleState(),
      blModule.getModuleState()
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
//    Pose2d scaled = new Pose2d(
//            new Translation2d(poseEstimator.getEstimatedPosition().getX(), poseEstimator.getEstimatedPosition().getY()),
//            poseEstimator.getEstimatedPosition().getRotation());
    // Return estimated pose from pose estimator
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Sets pose of swerve drive
   *
   * @param pose Pose2d instance
   */
  public void setPose(Pose2d pose) {
    // Reset pose
    poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
//    gyro.setYaw(pose.getRotation().getDegrees());

    // Reset simulated heading
    if (Robot.isSimulation()) {
      simHeading = pose.getRotation();
      System.out.println(pose.getRotation().getDegrees());
    }
  }

  /**
   * Get the current speeds of the swerve drive as ChassisSpeeds
   *
   * @return Current speeds as ChassisSpeeds
   */
  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Locks wheels in X configuration
   */
  public void lock() {
    // Steer modules in an X configuration
    flModule.steerTo(new Rotation2d( Math.PI/4));
    frModule.steerTo(new Rotation2d( -Math.PI/4));
    brModule.steerTo(new Rotation2d( Math.PI/4));
    blModule.steerTo(new Rotation2d( -Math.PI/4));

    // Set wheels locked to true
    wheelsLocked = true;
  }

  /**
   * Sets drive closed-loop mode to false
   */
  public void driveOpenLoop() {
    flModule.setDriveClosedLoop(false);
    frModule.setDriveClosedLoop(false);
    brModule.setDriveClosedLoop(false);
    blModule.setDriveClosedLoop(false);
  }

  /**
   * Sets drive closed-loop mode to true
   */
  public void driveClosedLoop() {
    flModule.setDriveClosedLoop(true);
    frModule.setDriveClosedLoop(true);
    brModule.setDriveClosedLoop(true);
    blModule.setDriveClosedLoop(true);
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
   * Used to drive in autonomous mode, drives robot relative without heading stabilization
   *
   * @param speeds Desired speeds as ChassisSpeeds
   */
  public void autoDrive(ChassisSpeeds speeds) {
    // If the robot is in simulation, divide the x and y velocities by 2
    if (Robot.isSimulation()) {
      speeds.vxMetersPerSecond /= 2.0;
      speeds.vyMetersPerSecond /= 2.0;
    }

    // Output velocities to SmartDashboard
    SmartDashboard.putNumber("[Swerve] vX", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("[Swerve] vY", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("[Swerve] Omega", speeds.omegaRadiansPerSecond);

    // Get the desired swerve module states
    SwerveModuleState[] desiredStates =  kinematics.toSwerveModuleStates(speeds);

    // Desaturate module speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_ATTAINABLE_SPEED);

    // Set module states to desired states
    setModuleStates(desiredStates);

    // If the robot is in simulation, update simulated heading depending on omega
    if (Robot.isSimulation()) {
      simHeading = simHeading.rotateBy(new Rotation2d(
        speeds.omegaRadiansPerSecond * Robot.kDefaultPeriod
      ));
    }
  }

  /**
   * Drives the swerve drive given desired velocities
   *
   * @param vx Velocity in the X direction (m/s)
   * @param vy Velocity in the Y direction (m/s)
   * @param omega Angular velocity (rads/s)
   */
  public void drive(double vx, double vy, double omega) {
    // Return if no desired velocity is given and wheels are locked
    if (vx == 0 && vy == 0 && omega == 0 && wheelsLocked) {
      return;
    // If wheels are locked but a desired velocity is given unlock wheels
    } else if (wheelsLocked) {
      wheelsLocked = false;
    }

    // Initialize actualOmega
    double actualOmega;

    // If headingLocked is true, lock heading
    if (headingLocked) {
      actualOmega = headingController.lock(
        getHeading(),
        desiredHeading
      );

      // If omega is non-zero (given as an argument) disable heading lock
      if (omega != 0)
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

    // Output velocities to SmartDashboard
    SmartDashboard.putNumber("[Swerve] vX", vx);
    SmartDashboard.putNumber("[Swerve] vY", vy);
    SmartDashboard.putNumber("[Swerve] Omega", actualOmega);

    // Get the desired swerve module states
    SwerveModuleState[] desiredStates =  kinematics.toSwerveModuleStates(
      (
        // If field relative is true, convert to robot relative speeds, else use robot relative speeds
        fieldOriented ?
          ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, actualOmega, getHeading()) :
          new ChassisSpeeds(vx, vy, actualOmega)
      )
    );

    // Desaturate module speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_ATTAINABLE_SPEED);

    // Set module states to desired states
    setModuleStates(desiredStates);

    // If robot is in simulation, update simulated heading depending on omega
    if (Robot.isSimulation()) {
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

  /**
   * Runs drive motors at specified voltage for translational system identification
   *
   * @param voltageMeasure Voltage measurement
   */
  private void translationSysID(Measure<Voltage> voltageMeasure) {
    flModule.steerTo(new Rotation2d());
    frModule.steerTo(new Rotation2d());
    brModule.steerTo(new Rotation2d());
    blModule.steerTo(new Rotation2d());

    flModule.setVoltage(voltageMeasure.baseUnitMagnitude());
    frModule.setVoltage(voltageMeasure.baseUnitMagnitude());
    brModule.setVoltage(voltageMeasure.baseUnitMagnitude());
    blModule.setVoltage(voltageMeasure.baseUnitMagnitude());
  }

  /**
   * Runs drive motors at specified voltage for rotational system identification
   *
   * @param voltageMeasure Voltage measurement
   */
  private void rotationSysID(Measure<Voltage> voltageMeasure) {
    flModule.steerTo(new Rotation2d(-Math.PI / 4.0));
    frModule.steerTo(new Rotation2d(Math.PI / 4.0));
    brModule.steerTo(new Rotation2d(-Math.PI / 4.0));
    blModule.steerTo(new Rotation2d(Math.PI / 4.0));

    flModule.setVoltage(voltageMeasure.baseUnitMagnitude());
    frModule.setVoltage(voltageMeasure.baseUnitMagnitude());
    brModule.setVoltage(voltageMeasure.baseUnitMagnitude());
    blModule.setVoltage(voltageMeasure.baseUnitMagnitude());
  }

  /**
   * Runs steer motors at specified voltage for steer system identification
   *
   * @param voltageMeasure Voltage measurement
   */
  private void steerSysID(Measure<Voltage> voltageMeasure) {
    flModule.setSteerVoltage(voltageMeasure.baseUnitMagnitude());
    frModule.setSteerVoltage(voltageMeasure.baseUnitMagnitude());
    brModule.setSteerVoltage(voltageMeasure.baseUnitMagnitude());
    blModule.setSteerVoltage(voltageMeasure.baseUnitMagnitude());
  }

  public double getPitch() {
    return gyro.getPitch().getValueAsDouble();
  }

  public double getRoll() {
    return gyro.getRoll().getValueAsDouble();
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
