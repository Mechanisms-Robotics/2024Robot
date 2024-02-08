package com.mechlib.hardware;

import com.revrobotics.CANSparkBase.SoftLimitDirection;
import frc.robot.Robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * MechLib SparkMax class
 *
 * Contains all methods of BrushlessMotorController that interface with the hardware of a Spark Max
 */
public class SparkMax extends BrushlessMotorController {
  private final CANSparkMax sparkMax; // CANSparkMax instance

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

    // Restore factory defaults
    sparkMax.restoreFactoryDefaults();
  }

  /**
   * SparkMax constructor w/ CANCoder
   *
   * @param id CAN ID of motor controller
   * @param canCoder CANCoder instance
   */
  public SparkMax(int id, CANCoder canCoder) {
    // Call super constructor
    super(id, canCoder);

    // Instantiate CANSparkMax
    this.sparkMax = new CANSparkMax(id, MotorType.kBrushless);
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
  public void setVoltageCompensation(double nominalVoltage) {
    // Enable voltage compensation on the CANSparkMax
    sparkMax.enableVoltageCompensation(nominalVoltage);
  }

  @Override
  public void setSoftLimits(double lowerLimit, double higherLimit) {
    // Set and enable reverse soft limit
    sparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float)lowerLimit);
    sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // Set and enable forward soft limit
    sparkMax.setSoftLimit(SoftLimitDirection.kForward, (float)higherLimit);
    sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
  }

  @Override
  public void zero() {
    // Check if internal encoder is being used
    if (canCoder == null) {
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
    if (canCoder == null) {
      // If so return internal encoder position
      return positionUnitsFunction.apply(sparkMax.getEncoder().getPosition());
    } else {
      // Otherwise return CANCoder position
      return canCoder.getPosition();
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

    if (canCoder == null) {
      // Return the internal encoder velocity
      return velocityUnitsFunction.apply(sparkMax.getEncoder().getVelocity());
    } else {
      // Return the CANCoder velocity
      return canCoder.getVelocity();
    }
  }
}
