package com.mechlib.hardware;

import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

/**
 * MechLib CANCoder class
 *
 * Contains some wrapper code for a CTRE CANCoder
 */
public class CANCoder {
  private final CANcoder canCoder; // CAN encoder

  private Rotation2d angle = new Rotation2d(); // Angle Rotation2d
  private double absoluteOffset = 0.0; // Absolute Offset

  /**
   * CANCoder constructor
   *
   * @param id CAN ID of CANCoder
   */
  public CANCoder(int id) {
    // Instantiate CANCoder
    canCoder = new CANcoder(id);

    // Create configuration
    CANcoderConfiguration config = new CANcoderConfiguration().withMagnetSensor(
      new MagnetSensorConfigs().withAbsoluteSensorRange(
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf
      )
    );

    // Apply configuration
    canCoder.getConfigurator().apply(config);
  }

  /**
   * CANCoder constructor
   *
   * @param id CAN ID of CANCoder
   * @param absoluteOffset Absolute offset of CANCoder
   */
  public CANCoder(int id, double absoluteOffset) {
    // Instantiate CANCoder and set absolute offset
    this(id);
    this.absoluteOffset = absoluteOffset;
  }

  /**
   * Sets position of CANCoder
   * 
   * @param position Position (degrees)
   */
  private void setPosition(double position) {
      canCoder.setPosition(position / 360.0);
  }

  /**
   * Zeroes the CANCoder
   */
  public void zero() { setPosition(-absoluteOffset); }

  /**
   * Get the position of the encoder
   *
   * @return Position
   */
  public Rotation2d getAngle() {
    // Get the absolute position of the CANCoder
    angle = Rotation2d.fromDegrees(canCoder.getAbsolutePosition().getValueAsDouble() * 360.0);
    // Rotate by the absolute offset value
    angle = angle.rotateBy(Rotation2d.fromDegrees(absoluteOffset));

    // Return angle
    return angle;
  }

  /**
   * Sets the absolute offset of the encoder
   *
   * @param absoluteOffset Absolute offset
   */
  public void setAbsoluteOffset(double absoluteOffset) {
    // Set the absolute offset
    this.absoluteOffset = absoluteOffset;
  }
}
