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

  // Magnet offset
  private final double magnetOffset;

  // Sensor direction
  private SensorDirectionValue sensorDirection = SensorDirectionValue.CounterClockwise_Positive;

  // Unit functions
  private Function<Double, Double> positionUnitsFunction = (Double x) -> x;
  private Function<Double, Double> velocityUnitsFunction = (Double x) -> x;

  /**
   * CANCoder constructor
   *
   * @param id CAN ID of CANCoder
   */
  public CANCoder(int id) {
    this(id, SensorDirectionValue.CounterClockwise_Positive, 0.0);
  }

  /**
   * CANCoder constructor
   *
   * @param id CAN ID of CANCoder
   * @param magnetOffset Absolute position magnet offset (rotations)
   */
  public CANCoder(int id, double magnetOffset) {
    this(id, SensorDirectionValue.CounterClockwise_Positive, magnetOffset);
    System.out.println("Calling the CANCoder constructor");
  }

  /**
   * CANCoder constructor
   *
   * @param id CAN ID of CANCoder
   * @param sensorDirection Direction of CANcoder
   * @param magnetOffset Absolute position magnet offset (rotations)
   */
  public CANCoder(int id, SensorDirectionValue sensorDirection, double magnetOffset) {
    // Instantiate CANCoder
    canCoder = new CANcoder(id);

    // Set sensor direction
    this.sensorDirection = sensorDirection;

    // Create configuration
    CANcoderConfiguration config = new CANcoderConfiguration().withMagnetSensor(
      new MagnetSensorConfigs().withAbsoluteSensorRange(
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf
      ).withSensorDirection(
        sensorDirection
      )
    );

    // Set magnet offset
    this.magnetOffset = magnetOffset;

    // Apply configuration
    canCoder.getConfigurator().apply(config);

    // Reset relative position to absolute position
    canCoder.getAbsolutePosition().getValueAsDouble();
  }

  /**
   * Sets sensor direction inverted
   *
   * @param inverted Sensor direction inverted
   */
  public void setInverted(boolean inverted) {
    // Check if inverted is true
    if (inverted) {
      // Set sensor direction to clockwise
      sensorDirection = SensorDirectionValue.Clockwise_Positive;
    } else {
      // Set sensor direction to counter-clockwise
      sensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    }

    // Apply new magnet sensor configuration
    canCoder.getConfigurator().apply(
      new MagnetSensorConfigs().withAbsoluteSensorRange(
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf
      ).withSensorDirection(
        sensorDirection
      )
    );
  }

  /**
   * Inverts sensor direction
   */
  public void invert() {
    // Toggle inverted
    setInverted(sensorDirection != SensorDirectionValue.Clockwise_Positive);
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
   * Gets the absolute position of the CANcoder
   *
   * @return Absolute position (position units)
   */
  public double getAbsolutePosition() {
    System.out.println("Absolute position of CanCoder: " + canCoder.getAbsolutePosition().getValueAsDouble()
                        + " magnet offset: " + magnetOffset);
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
   * Sets position of CANCoder in native units
   * 
   * @param position Position (rotations)
   */
  private void setPosition(double position) {
    System.out.println("Position: " + position);
    canCoder.setPosition(position);
  }

  /**
   * Zeroes the CANCoder
   */
  public void zero() { setPosition(-magnetOffset); }
}
