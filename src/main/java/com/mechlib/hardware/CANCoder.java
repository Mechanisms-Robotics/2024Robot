package com.mechlib.hardware;

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
public class CANCoder {
  // CANcoder
  private final CANcoder canCoder;

  // Unit functions
  private Function<Double, Double> positionUnitsFunction = (Double x) -> x;
  private Function<Double, Double> velocityUnitsFunction = (Double x) -> x;

  /**
   * CANCoder constructor
   *
   * @param id CAN ID of CANCoder
   */
  public CANCoder(int id) {
    this(id, SensorDirectionValue.CounterClockwise_Positive);
  }

  /**
   * CANCoder constructor
   *
   * @param id CAN ID of CANCoder
   * @param sensorDirection Direction of CANcoder
   */
  public CANCoder(int id, SensorDirectionValue sensorDirection) {
    // Instantiate CANCoder
    canCoder = new CANcoder(id);

    // Create configuration
    CANcoderConfiguration config = new CANcoderConfiguration().withMagnetSensor(
      new MagnetSensorConfigs().withAbsoluteSensorRange(
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf
      ).withSensorDirection(
        sensorDirection
      )
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
   * Gets the position of the CANcoder
   *
   * @return Position (position units)
   */
  public double getPosition() {
    return positionUnitsFunction.apply(canCoder.getPosition().getValueAsDouble());
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
   * Sets position of CANCoder in native units
   * 
   * @param position Position (rotations)
   */
  private void setPosition(double position) {
      canCoder.setPosition(position);
  }

  /**
   * Zeroes the CANCoder
   */
  public void zero() { setPosition(0.0); }
}
