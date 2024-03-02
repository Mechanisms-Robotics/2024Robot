package com.mechlib.hardware;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Function;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

/**
 * MechLib CANCoder class
 *
 * Contains some wrapper code for a CTRE CANCoder
 */
public class CANCoder extends SubsystemBase {
  // CANcoder
  private final CANcoder canCoder;

  // Magnet offset
  private double magnetOffset;

  // CANCoder config
  private CANcoderConfiguration config = new CANcoderConfiguration();

  // Unit functions
  private Function<Double, Double> positionUnitsFunction = (Double x) -> x;
  private Function<Double, Double> velocityUnitsFunction = (Double x) -> x;

  /**
   * CANCoder constructor
   *
   * @param id CAN ID of CANCoder
   */
  public CANCoder(int id) {
    this(
      id,
      0,
      AbsoluteSensorRangeValue.Signed_PlusMinusHalf,
      SensorDirectionValue.CounterClockwise_Positive
    );
  }

  /**
   * CANCoder constructor
   *
   * @param id CAN ID of CANCoder
   * @param magnetOffset Magnet offset (rotations)
   */
  public CANCoder(int id, double magnetOffset) {
    this(
      id,
      magnetOffset,
      AbsoluteSensorRangeValue.Signed_PlusMinusHalf,
      SensorDirectionValue.CounterClockwise_Positive
    );
  }

  /**
   * CANCoder constructor
   *
   * @param id CAN ID of CANCoder
   * @param magnetOffset Magnet offset (rotations)
   * @param sensorRange Range of absolute sensor (AbsoluteSensorRangeValue)
   */
  public CANCoder(int id, double magnetOffset, AbsoluteSensorRangeValue sensorRange) {
    this(
      id,
      magnetOffset,
      sensorRange,
      SensorDirectionValue.CounterClockwise_Positive
    );
  }

  /**
   * CANCoder constructor
   *
   * @param id CAN ID of CANCoder
   * @param magnetOffset Magnet offset (rotations)
   * @param sensorRange Range of absolute sensor (AbsoluteSensorRangeValue)
   * @param sensorDirection Direction of sensor (SensorDirectionValue)
   */
  public CANCoder(
    int id,
    double magnetOffset,
    AbsoluteSensorRangeValue sensorRange,
    SensorDirectionValue sensorDirection
  ) {
    // Instantiate CANCoder
    canCoder = new CANcoder(id);

    // Configure CANCoder
    configure(magnetOffset, sensorRange, sensorDirection);
  }

  /**
   * Configures CANCoder
   *
   * @param magnetOffset Magnet offset (rotations)
   * @param sensorRange Range of absolute sensor (AbsoluteSensorRangeValue)
   * @param sensorDirection Direction of sensor (SensorDirectionValue)
   */
  public void configure(
    double magnetOffset,
    AbsoluteSensorRangeValue sensorRange,
    SensorDirectionValue sensorDirection
  ) {
    // Set magnet offset
    this.magnetOffset = magnetOffset;
    CANcoderConfiguration currentConfig = new CANcoderConfiguration();
    canCoder.getConfigurator().refresh(currentConfig);

    // Set configuration
    config = config.withMagnetSensor(
      new MagnetSensorConfigs()
        .withAbsoluteSensorRange(sensorRange)
        .withSensorDirection(sensorDirection)
        .withMagnetOffset(currentConfig.MagnetSensor.MagnetOffset)
    );

    // Apply configuration
    canCoder.getConfigurator().apply(config);
  }

  /**
   * Sets the position units function
   *
   * @param positionUnitsFunction Function that takes in native units and returns desired units
   */
  public void setPositionUnitsFunction(Function<Double, Double> positionUnitsFunction) {
    this.positionUnitsFunction = positionUnitsFunction;
  }

  /**
   * Sets the velocity units function
   *
   * @param velocityUnitsFunction Function that takes in native units and returns desired units
   */
  public void setVelocityUnitsFunction(Function<Double, Double> velocityUnitsFunction) {
    this.velocityUnitsFunction = velocityUnitsFunction;
  }

  /**
   * Gets the relative position of the CANCoder
   *
   * @return Relative position (position units)
   */
  public double getRelativePosition() {
    return positionUnitsFunction.apply(
      canCoder.getPosition().getValueAsDouble() + magnetOffset
    );
  }

  /**
   * Gets the absolute position of the CANcoder
   *
   * @return Absolute position (position units)
   */
  public double getAbsolutePosition() {
    return positionUnitsFunction.apply(
      canCoder.getAbsolutePosition().getValueAsDouble() + magnetOffset
    );
  }

  /**
   * Gets the velocity of the CANcoder
   *
   * @return Velocity (velocity units)
   */
  public double getVelocity() {
    return velocityUnitsFunction.apply(canCoder.getVelocity().getValueAsDouble());
  }

  /**
   * Zeroes the CANCoder
   */
  public void zero() { setRelativePosition(-magnetOffset); }

  /**
   * Sets relative position of CANCoder in native units
   * 
   * @param position Position (rotations)
   */
  public void setRelativePosition(double position) {
      canCoder.setPosition(position);
  }

  /**
   * Sets relative position to absolute position
   */
  public void syncRelativePosition() {
    // Set relative position to absolute position
    canCoder.setPosition(canCoder.getAbsolutePosition().getValueAsDouble());
  }
}
