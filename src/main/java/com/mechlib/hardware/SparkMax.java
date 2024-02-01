package com.mechlib.hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Robot;

/**
 * MechLib SparkMax class
 *
 * Contains all methods of BrushlessMotorController that interface with the hardware of a Spark Max
 */
public class SparkMax extends BrushlessMotorController {
  private final CANSparkMax sparkMax; // CANSparkMax instance

  private CANCoder canCoder; // CANCoder instance
  private boolean useInternalEncoder = true; // Internal encoder flag

  /**
   * SparkMax constructor
   *
   * @param id CAN ID of motor controller
   */
  public SparkMax(int id) {
    // Call super constructor
    super(id);

    // Instantiate CANSparkMax
    this.sparkMax = new CANSparkMax(id, MotorType.kBrushless);
  }

  /**
   * SparkMax constructor w/ CANCoder
   *
   * @param id CAN ID of motor controller
   * @param canCoder CANCoder instance
   */
  public SparkMax(int id, CANCoder canCoder) {
    // Call super constructor
    super(id);

    // Instantiate CANSparkMax
    this.sparkMax = new CANSparkMax(id, MotorType.kBrushless);

    // Set CANCoder and disable internal encoder
    this.canCoder = canCoder;
    useInternalEncoder = false;
  }

  /**
   * SparkMax constructor w/ CANCoder
   *
   * @param id CAN ID of motor controller
   * @param canCoderID CAN ID of CANCoder
   */
  public SparkMax(int id, int canCoderID) {
    // Call constructor with a newly instantiated CANCoder
    this(id, new CANCoder(canCoderID));
  }

  /**
   * SparkMax constructor w/ CANCoder and absolute offset
   *
   * @param id CAN ID of motor controller
   * @param canCoderID CAN ID of CANCoder
   * @param absoluteOffset Absolute offset of CANCoder
   */
  public SparkMax(int id, int canCoderID, double absoluteOffset) {
    // Call constructor with a newly instantiated CANCoder with a given absolute offset
    this(id, new CANCoder(canCoderID, absoluteOffset));
  }

  @Override
  public void setPercent(double percent) {
    // Set CANSparkMax percent output
    sparkMax.set(percent);

    // Update followers
    updateFollowers(percent);
  }

  @Override
  public void setInverted(boolean inverted) {
    // Set CANSparkMax inversion
    sparkMax.setInverted(inverted);
  }

  @Override
  public void invert() {
    // Toggle CANSparkMax inversion
    sparkMax.setInverted(!sparkMax.getInverted());
  }

  @Override
  public void coastMode() {
    // Set CANSparkMax to Coast mode
    sparkMax.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void brakeMode() {
    // Set CANSparkMax to Brake mode
    sparkMax.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void setCurrentLimit(double limit) {
    // Set CANSparkMax smart current limit
    sparkMax.setSmartCurrentLimit((int)limit, (int)limit);
  }

  @Override
  public void zero() {
    // Check if internal encoder is being used
    if (useInternalEncoder) {
      // If so zero the internal encoder
      sparkMax.getEncoder().setPosition(0);
    } else {
      // Otherwise zero the CANCoder
      canCoder.zero();
    }
  }

  @Override
  public double getPosition() {
    // Check if this is a simulation
    if (Robot.isSimulation())
      // If so just return the setpoint
      return pidController.getSetpoint();

    // Check if internal encoder is being used
    if (useInternalEncoder) {
      // If so return internal encoder position
      return sparkMax.getEncoder().getPosition();
    } else {
      // Otherwise return CANCoder angle in degrees
      return canCoder.getAngle().getDegrees();
    }
  }

  @Override
  public double getVelocity() {
    // Check if this is a simulation
    if (Robot.isSimulation()) {
      // Adjust simulated distance depending on PID velocity setpoint and motor inversion
      simDistance += pidController.getSetpoint() * Robot.kDefaultPeriod * 10 * (
        sparkMax.getInverted() ? -1.0 : 1.0
      );

      // If so just return the setpoint
      return pidController.getSetpoint();
    }

    // Return the internal encoder velocity
    return sparkMax.getEncoder().getVelocity();
  }
}
