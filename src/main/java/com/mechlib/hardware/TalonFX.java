package com.mechlib.hardware;

import frc.robot.Robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;

/**
 * MechLib TalonFX class
 *
 * Contains all methods of BrushlessMotorController that interface with the hardware of a TalonFX
 */
public class TalonFX extends BrushlessMotorController {
  private final com.ctre.phoenix6.hardware.TalonFX talonFX; // WPI_TalonFX instance

  private TalonFXConfiguration configuration = new TalonFXConfiguration();

  private CANCoder canCoder = null; // CANCoder instance
  private boolean useInternalEncoder = true; // Internal encoder flag

  /**
   * TalonFX constructor
   *
   * @param id CAN ID of motor controller
   */
  public TalonFX(int id) {
    // Call super constructor
    super(id);

    // Instantiate TalonFX
    this.talonFX = new com.ctre.phoenix6.hardware.TalonFX(id);
  }

  /**
   * TalonFX constructor w/ CANCoder
   *
   * @param id CAN ID of motor controller
   * @param canCoder CANCoder instance
   */
  public TalonFX(int id, CANCoder canCoder) {
    // Instantiate TalonFX
    this(id);

    // Set CANCoder and disable internal encoder
    this.canCoder = canCoder;
    useInternalEncoder = false;
  }

  /**
   * TalonFX constructor w/ CANCoder
   *
   * @param id CAN ID of motor controller
   * @param canCoderID CAN ID of CANCoder
   */
  public TalonFX(int id, int canCoderID) {
    // Call constructor with a newly instantiated CANCoder
    this(id, new CANCoder(canCoderID));
  }

  /**
   * TalonFX constructor w/ CANCoder and absoulte offset
   *
   * @param id CAN ID of motor controller
   * @param canCoderID CAN ID of CANCoder
   * @param absoluteOffset Absolute offset of CANCoder
   */
  public TalonFX(int id, int canCoderID, double absoluteOffset) {
    // Call constructor with a newly instantiated CANCoder with a given absolute offset
    this(id, new CANCoder(canCoderID, absoluteOffset));
  }

  @Override
  public void setPercent(double percent) {
    // Set TalonFX percent
    talonFX.set(percent);

    // Update followers
    updateFollowers(percent);
  }

  @Override
  public void setInverted(boolean inverted) {
    // Set TalonFX inversion
    talonFX.setInverted(inverted);
  }

  @Override
  public void invert() {
    // Toggle TalonFX inversion
    talonFX.setInverted(!talonFX.getInverted());
  }

  @Override
  public void coastMode() {
    // Set TalonFX to Coast mode
    talonFX.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void brakeMode() {
    // Set TalonFX to Brake mode
    talonFX.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void setCurrentLimit(double limit) {
    // Configure TalonFX supply current limit
    TalonFXConfigurator cfg = talonFX.getConfigurator();
    cfg.refresh(configuration.CurrentLimits);
    cfg.apply(
        configuration.CurrentLimits.withStatorCurrentLimit(limit)
                                   .withStatorCurrentLimitEnable(true));
  }

  @Override
  public void zero() {
    // Check if internal encoder is being used
    if (useInternalEncoder) {
      // If so zero the internal encoder
      talonFX.setPosition(0);
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
      return talonFX.getPosition().getValueAsDouble();
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
        talonFX.getInverted() ? -1.0 : 1.0
      );

      // If so just return the setpoint
      return pidController.getSetpoint();
    }

    // Return the internal encoder velocity
    return talonFX.getVelocity().getValueAsDouble();
  }
}
