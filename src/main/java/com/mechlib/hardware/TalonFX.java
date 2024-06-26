package com.mechlib.hardware;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import frc.robot.Robot;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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

  private boolean inverted = false; // Inverted flag
  private boolean sensorInverted = false; // Sensor inverted flag

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

    // Reset to factory defaults
    talonFX.getConfigurator().apply(new TalonFXConfiguration());
  }

  /**
   * TalonFX constructor w/ CANCoder
   *
   * @param id CAN ID of motor controller
   * @param canCoder CANCoder instance
   */
  public TalonFX(int id, CANCoder canCoder) {
    // Call super constructor
    super(id, canCoder);

    // Instantiate TalonFX
    this.talonFX = new com.ctre.phoenix6.hardware.TalonFX(id);

    // Reset to factory defaults
    talonFX.getConfigurator().apply(new TalonFXConfiguration());
  }

  @Override
  public void setPercent(double percent) {
    // Set TalonFX percent
    talonFX.set(percent);

    // Update followers
    updateFollowers(percent);
  }

  @Override
  public void setVoltage(double voltage) {
    // Set TalonFX voltage
    talonFX.setVoltage(voltage);

    // Update followers
    updateFollowersVoltage(voltage);
  }

  @Override
  public void setInverted(boolean inverted) {
    // Set inverted flag
    this.inverted = inverted;

    // Set TalonFX inversion
    talonFX.setInverted(inverted);
  }

  @Override
  public void setSensorInverted(boolean sensorInverted) {
    // Set sensor inverted flag
    this.sensorInverted = sensorInverted;

    // Check if CANCoder is not null
    if (canCoder == null) {
      System.out.println("[ERROR] TalonFX sensor in sync, invert motor instead!");
    } else {
      // Print error
      System.out.println("[ERROR] CANCoder inversion should be passed in to constructor!");
    }
  }

  @Override
  public void setInternalSensorPosition(double position) {
    // Set TalonFX encoder position
    talonFX.setPosition(position);
  }

  @Override
  public void invert() {
    // Toggle inverted
    setInverted(!inverted);
  }

  @Override
  public void invertSensor() {
    // Toggle sensor inverted
    setSensorInverted(!sensorInverted);
  }

  @Override
  public void coastMode() {
    // Set TalonFX to Coast mode
    talonFX.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void brakeMode() {
    // Set TalonFX to Brake mode
    talonFX.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void setCurrentLimit(double limit) {
    // Get configurator
    TalonFXConfigurator cfg = talonFX.getConfigurator();

    // Instantiate current limits configuration
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(limit)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(limit)
        .withStatorCurrentLimitEnable(true);

    // Apply configuration
    cfg.apply(currentLimits);
  }

  @Override
  public void setVoltageCompensation(double nominalVoltage) {
    // Get configurator
    TalonFXConfigurator cfg = talonFX.getConfigurator();

    // Instantiate voltage compensation configuration
    VoltageConfigs voltageComp = new VoltageConfigs()
      .withPeakReverseVoltage(nominalVoltage)
      .withPeakForwardVoltage(nominalVoltage);

    // Apply configuration
    cfg.apply(voltageComp);
  }

  @Override
  public void setSoftLimits(double lowerLimit, double higherLimit) {
    // Get configurator
    TalonFXConfigurator cfg = talonFX.getConfigurator();

    // Instantiate soft limits configuration
    SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs()
      .withReverseSoftLimitThreshold(lowerLimit)
      .withReverseSoftLimitEnable(true)
      .withForwardSoftLimitThreshold(higherLimit)
      .withForwardSoftLimitEnable(true);

    // Apply configuration
    cfg.apply(softLimits);
  }

  @Override
  public void zero() {
    // Check if internal encoder is being used
    if (canCoder == null) {
      // If so zero the internal encoder
      talonFX.setPosition(0);
    } else {
      // Otherwise zero the CANCoder
      canCoder.zero();
    }
  }

  @Override
  public double getRawPosition() {
    return talonFX.getPosition().getValueAsDouble();
  }

  @Override
  public double getRelativePosition() {
    // Check if this is a simulation
    if (Robot.isSimulation())
      // If so just return the setpoint
      return pidController.getSetpoint();

    // Check if CANCoder exists
    if (canCoder != null) {
      // Return CANCoder relative position
      return canCoder.getRelativePosition();
    } else {
      // Otherwise return the internal encoder position
      return positionUnitsFunction.apply(talonFX.getPosition().getValueAsDouble());
    }
  }

  @Override
  public double getAbsolutePosition() {
    // Check if this is a simulation
    if (Robot.isSimulation())
      // If so just return the setpoint
      return pidController.getSetpoint();

    // Check if CANCoder exists
    if (canCoder != null) {
      // Return CANCoder absolute position
      return canCoder.getAbsolutePosition();
    }

    // Print error
    System.out.println("[ERROR] No CANCoder provided!");

    // Return 0
    return 0.0;
  }

  @Override
  public double getVelocity() {
    // Check if this is a simulation
    if (Robot.isSimulation()) {
      // Adjust simulated distance depending on PID velocity setpoint and motor inversion
      simDistance += pidController.getSetpoint() * Robot.kDefaultPeriod * (
        inverted ? -1.0 : 1.0
      );

      // If so just return the setpoint
      return pidController.getSetpoint();
    }

    // Check if internal encoder is being used
    if (canCoder == null) {
      // If so return internal encoder velocity
      return velocityUnitsFunction.apply(talonFX.getVelocity().getValueAsDouble());
    } else {
      // Otherwise return CANCoder velocity
      return canCoder.getVelocity();
    }
  }
}
