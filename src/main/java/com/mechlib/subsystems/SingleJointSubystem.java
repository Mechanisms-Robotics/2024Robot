package com.mechlib.subsystems;

import com.mechlib.hardware.BrushlessMotorController;
import com.mechlib.util.MechUnits;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.function.Function;

/**
 * MechLib SingleJointSubsystem class
 *
 * A single jointed subsystem using a main pivot motor and an arbitrary amount of follower motors.
 * (MAKE SURE TO PROVIDE CANCODERS TO ALL MOTORS ADDED)
 */
public class SingleJointSubystem extends SubsystemBase {
  // Motors
  protected ArrayList<BrushlessMotorController> motors = new ArrayList<>();

  // PID Constants
  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;

  // Max velocity and acceleration
  private double maxVelocity = 0.0;
  private double maxAcceleration = 0.0;

  // Pivot feedforward controller
  protected ArmFeedforward feedforwardController = new ArmFeedforward(
    0,
    0,
    0,
    0);

  protected Rotation2d desiredAngle = new Rotation2d(); // Desired angle

  // Subsystem state enumerator
  public enum SingleJointSubsystemState {
    OPEN_LOOP,
    CLOSED_LOOP
  };

  // Current subsystem state
  protected SingleJointSubsystemState state = SingleJointSubsystemState.CLOSED_LOOP;

  /**
   * Adds a motor to the SingleJointSubsystem
   *
   * @param motor BrushlessMotorController instance
   */
  protected void addMotor(BrushlessMotorController motor) {
    // Set default neutral mode and inversion
    motor.brakeMode();
    motor.setInverted(false);

    // Sync CANCoder relative position to absolute position
    motor.syncRelativePosition();

    // Add motor
    motors.add(motor);
  }

  /**
   * Adds a motor to the SingleJointSubsystem
   *
   * @param motor BrushlessMotorController instance
   * @param inversion Inversion of motor
   */
  protected void addMotor(BrushlessMotorController motor, boolean inversion) {
    // Set neutral mode and inversion
    motor.brakeMode();
    motor.setInverted(inversion);

    // Sync CANCoder relative position to absolute position
    motor.syncRelativePosition();

    // Add motor
    motors.add(motor);
  }

  /**
   * Sets state of SingleJointSubsystem
   *
   * @param state SingleJointSubsystemState
   */
  protected void setState(SingleJointSubsystemState state) {
    this.state = state;
  }

  /**
   * Sets all motors to coast mode
   */
  protected void coastMode() {
    // Loop over every motor
    for (BrushlessMotorController motor : motors) {
      // Set motor to coast mode
      motor.coastMode();
    }
  }

  /**
   * Sets all motors to brake mode
   */
  protected void brakeMode() {
    // Loop over every motor
    for (BrushlessMotorController motor : motors) {
      // Set motor to coast mode
      motor.brakeMode();
    }
  }

  /**
   * Sets current limit for all motors
   *
   * @param currentLimit Current limit (amps)
   */
  protected void setCurrentLimit(double currentLimit) {
    // Loop over every motor
    for (BrushlessMotorController motor : motors) {
      // Apply current limiting to motor
      motor.setCurrentLimit(currentLimit);
    }
  }

  /**
   * Sets voltage compensation for all motors
   *
   * @param nominalVoltage Nominal voltage (volts)
   */
  protected void setVoltageCompensation(double nominalVoltage) {
    // Loop over every motor
    for (BrushlessMotorController motor : motors) {
      // Apply voltage compensation to follower motor
      motor.setVoltageCompensation(nominalVoltage);
    }
  }

  /**
   * Sets arm motors to a specified voltage
   *
   * @param voltage Volts
   */
  public void setVoltage(double voltage) {
    // Loop over every motor
    for (BrushlessMotorController motor : motors) {
      // Set motor voltage
      motor.setVoltage(voltage);
    }
  }

  /**
   * Stops the motors
   */
  public void stop() {
    // Set voltage to zero
    setVoltage(0.0);
  }

  /**
   * Set position units function of all motors
   *
   * @param positionUnitsFunction Function that takes in native units and returns radians
   */
  protected void setPositionUnitsFunction(Function<Double, Double> positionUnitsFunction) {
    // Check if any motors have been added
    if (motors.size() == 0) {
      // Throw error and return
      System.out.println("[ERROR] No motors supplied to SingleJointSubsystem!");
      return;
    }

    // Loop over every motor
    for (BrushlessMotorController motor : motors) {
      // Set position units function of first motor
      motor.setPositionUnitsFunction(positionUnitsFunction);
    }
  }

  /**
   * Set velocity units function of pivot motor
   *
   * @param velocityUnitsFunction Function that takes in native units and returns radians per second
   */
  protected void setVelocityUnitsFunction(Function<Double, Double> velocityUnitsFunction) {
    // Check if any motors have been added
    if (motors.size() == 0) {
      // Throw error and return
      System.out.println("[ERROR] No motors supplied to SingleJointSubsystem!");
      return;
    }

    // Loop over every motor
    for (BrushlessMotorController motor : motors) {
      // Set velocity units function of first motor
      motor.setVelocityUnitsFunction(velocityUnitsFunction);
    }
  }

  /**
   * Set limits of mechanism
   *
   * @param reverseLimit Reverse limit
   * @param forwardLimit Forward Limit
   * @param gearRatio Gear ratio (x:1)
   */
  protected void setLimits(Rotation2d reverseLimit, Rotation2d forwardLimit, double gearRatio) {
    // Check if any motors have been added
    if (motors.size() == 0) {
      // Throw error and return
      System.out.println("[ERROR] No motors supplied to SingleJointSubsystem!");
      return;
    }

    // Loop over every motor
    for (BrushlessMotorController motor : motors) {
      // Set the motor's internal sensor position to CANCoder relative position
      motor.setInternalSensorPosition(
        MechUnits.radiansToRotations(motor.getRelativePosition(), gearRatio)
      );

      // Set motor soft limits
      motor.setSoftLimits(
        MechUnits.radiansToRotations(reverseLimit.getRadians(), gearRatio),
        MechUnits.radiansToRotations(forwardLimit.getRadians(), gearRatio)
      );
    }
  }

  /**
   * Sets feedforward gains
   *
   * @param kS S gain, overcomes static friction (volts)
   * @param kG G gain, overcomes gravitational force (volts)
   * @param kV V gain, models how voltage should affect velocity (volts * seconds / distance)
   * @param kA A gain, models how voltage should affect acceleration (volts * seconds^2 / distance)
   */
  protected void setFeedforwardGains(double kS, double kG, double kV, double kA) {
    feedforwardController = new ArmFeedforward(kS, kG, kV, kA);
  }

  /**
   * Sets PPID gains
   *
   * @param kP P gain
   * @param kI I gain
   * @param kD D gain
   */
  protected void setPPIDGains(double kP, double kI, double kD) {
    // Loop over every motor
    for (BrushlessMotorController motor : motors) {
      // Set PID gains
      motor.setPIDGains(kP, kI, kD);
    }
  }

  /**
   * Sets PPID constraints
   *
   * @param maxVelocity Max velocity (velocity units)
   * @param maxAcceleration Max acceleration (velocity units/s)
   */
  protected void setPPIDConstraints(double maxVelocity, double maxAcceleration) {
    // Loop over every motor
    for (BrushlessMotorController motor : motors) {
      // Set PPID trapezoidal motion profile constraints
      motor.setConstraints(maxVelocity, maxAcceleration);
    }
  }

  /**
   * Sets tolerance of PPID controller
   *
   * @param tolerance Tolerance (rads)
   */
  protected void setTolerance(double tolerance) {
    // Loop over every motor
    for (BrushlessMotorController motor : motors) {
      // Set PID tolerance
      motor.setTolerance(tolerance);
    }
  }

  /**
   * Gets the current angle as a Rotation2d
   *
   * @return Current angle
   */
  protected Rotation2d getAngle() {
    // Check if any motors have been added
    if (motors.size() == 0) {
      // Throw error and return
      System.out.println("[ERROR] No motors supplied to SingleJointSubsystem!");
      return new Rotation2d();
    }

    // Return the first motor's sensor position as a Rotation2d
    return new Rotation2d(motors.get(0).getRelativePosition());
  }

  /**
   * Gets the desired angle
   *
   * @return Desired angle
   */
  protected Rotation2d getDesiredAngle() {
    return desiredAngle;
  }

  /**
   * Pivots to a desired angle
   *
   * @param desiredAngle Desired angle
   */
  protected void pivotTo(Rotation2d desiredAngle) {
    // Set desired angle
    this.desiredAngle = desiredAngle;

    // Loop over every motor
    for (BrushlessMotorController motor : motors) {
      // Set setpoint to desired angle
      motor.setSetpoint(desiredAngle.getRadians());
    }
  }

  /**
   * Runs periodic PPIDF code
   */
  @Override
  public void periodic() {
    // Check if current state is closed loop
    if (state == SingleJointSubsystemState.CLOSED_LOOP) {
      // Loop over every motor
      for (BrushlessMotorController motor : motors) {
        // Run periodic PIDF code
        motor.periodicPIDF(
          motor.getRelativePosition(),
          motor.getVelocity(),
          feedforwardController.calculate(
            motor.getSetpoint(),
            motor.getVelocitySetpoint()
          )
        );
      }
    }
  }
}
