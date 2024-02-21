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
 * A single jointed subsystem using a main pivot motor and an arbitrary amount of follower motors
 */
public class SingleJointSubystem extends SubsystemBase {
  // Motors
  protected ArrayList<Pair<BrushlessMotorController, Boolean>> motors = new ArrayList<>();

  // Pivot PID controller
  ProfiledPIDController ppidController = new ProfiledPIDController(
    0,
    0,
    0,

    new Constraints(
      0, 0
    )
  );

  // Pivot feedforward controller
  protected ArmFeedforward feedforwardController = new ArmFeedforward(
    0,
    0,
    0,
    0);

  protected Rotation2d desiredAngle = new Rotation2d(); // Desired angle

  public enum SingleJointSubsystemState {
    OPEN_LOOP,
    CLOSED_LOOP
  }

  protected SingleJointSubsystemState state = SingleJointSubsystemState.CLOSED_LOOP;

  /**
   * Adds a motor to the SingleJointSubsystem
   *
   * @param motor BrushlessMotorController instance
   */
  protected void addMotor(BrushlessMotorController motor) {
    // Add motor
    motors.add(
      new Pair<>(
        motor,
        false
      )
    );
  }

  /**
   * Adds a motor to the SingleJointSubsystem
   *
   * @param motor BrushlessMotorController instance
   * @param inversion Inversion of motor
   */
  protected void addMotor(BrushlessMotorController motor, boolean inversion) {
    // Add motor
    motors.add(
      new Pair<>(
        motor,
        inversion
      )
    );
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
   * Sets arm motors to a specified voltage
   *
   * @param voltage Volts
   */
  public void setVoltage(double voltage) {
    // Loop over every motor
    for (Pair<BrushlessMotorController, Boolean> motorPair : motors) {
      BrushlessMotorController motor = motorPair.getFirst();
      boolean inversion = motorPair.getSecond();

      // Set motor voltage accounting for inversion
      motor.setVoltage(voltage * (inversion ? -1.0 : 1.0));
    }
  }

  /**
   * Configures all motors
   */
  protected void configureMotors() {
    // Check if any motors have been added
    if (motors.isEmpty()) {
      // Throw error and return
      System.out.println("[ERROR] No motors supplied to SingleJointSubsystem!");
      return;
    }

    // Loop over every motor
    for (Pair<BrushlessMotorController, Boolean> motorPair : motors) {
      // Get the motor
      BrushlessMotorController motor = motorPair.getFirst();

      // Set the motor to brake mode
      motor.brakeMode();

      // Apply current limiting and voltage compensation to motor
      motor.setCurrentLimit(40.0);
      motor.setVoltageCompensation(10.0);
    }

    // Set position and velocity units function of first motor
    motors.get(0).getFirst().setPositionUnitsFunction(MechUnits::rotationsToRadians);
    motors.get(0).getFirst().setVelocityUnitsFunction(MechUnits::rotationsToRadians);
  }

  /**
   * Sets current limit for all motors
   *
   * @param currentLimit Current limit (amps)
   */
  protected void setCurrentLimit(double currentLimit) {
    // Loop over every motor
    for (Pair<BrushlessMotorController, Boolean> motorPair : motors) {
      // Apply current limiting to motor
      motorPair.getFirst().setCurrentLimit(currentLimit);
    }
  }

  /**
   * Sets voltage compensation for all motors
   *
   * @param nominalVoltage Nominal voltage (volts)
   */
  protected void setVoltageCompensation(double nominalVoltage) {
    // Loop over every motor
    for (Pair<BrushlessMotorController, Boolean> motorPair : motors) {
      // Apply voltage compensation to follower motor
      motorPair.getFirst().setVoltageCompensation(nominalVoltage);
    }
  }

  /**
   * Set position units function of all motors
   *
   * @param positionUnitsFunction Function that takes in native units and returns radians
   */
  protected void setPositionUnitsFunction(Function<Double, Double> positionUnitsFunction) {
    // Check if any motors have been added
    if (motors.isEmpty()) {
      // Throw error and return
      System.out.println("[ERROR] No motors supplied to SingleJointSubsystem!");
      return;
    }

    // Set position units function of first motor
    motors.get(0).getFirst().setPositionUnitsFunction(positionUnitsFunction);
  }

  /**
   * Set velocity units function of pivot motor
   *
   * @param velocityUnitsFunction Function that takes in native units and returns radians per second
   */
  protected void setVelocityUnitsFunction(Function<Double, Double> velocityUnitsFunction) {
    // Check if any motors have been added
    if (motors.isEmpty()) {
      // Throw error and return
      System.out.println("[ERROR] No motors supplied to SingleJointSubsystem!");
      return;
    }

    // Set velocity units function of first motor
    motors.get(0).getFirst().setVelocityUnitsFunction(velocityUnitsFunction);
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
    if (motors.isEmpty()) {
      // Throw error and return
      System.out.println("[ERROR] No motors supplied to SingleJointSubsystem!");
      return;
    }

    // Get the first motor
    BrushlessMotorController motor = motors.get(0).getFirst();

    // Set internal sensor position based off CANCoder position
    motor.setInternalSensorPosition(
      MechUnits.radiansToRotations(motor.getPosition(), gearRatio)
    );

    // Set soft limits
    motor.setSoftLimits(
      MechUnits.radiansToRotations(reverseLimit.getRadians(), gearRatio),
      MechUnits.radiansToRotations(forwardLimit.getRadians(), gearRatio)
    );
  }

  /**
   * Sets feedforward controller

   * @param feedforwardController ArmFeedforward instance
   */
  protected void setFeedforwardController(ArmFeedforward feedforwardController) {
    this.feedforwardController = feedforwardController;
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
   * Sets S gain of feedforward controller
   *
   * @param kS S gain, overcomes static friction (volts)
   */
  protected void setKS(double kS) {
    feedforwardController = new ArmFeedforward(
      kS,
      feedforwardController.kg,
      feedforwardController.kv,
      feedforwardController.ka
    );
  }

  /**
   * Sets G gain of feedforward controller
   *
   * @param kG G gain, overcomes gravitational force (volts)
   */
  protected void setKG(double kG) {
    feedforwardController = new ArmFeedforward(
      feedforwardController.ks,
      kG,
      feedforwardController.kv,
      feedforwardController.ka
    );
  }

  /**
   * Sets V gain of feedforward controller
   *
   * @param kV V gain, models how voltage should affect velocity (volts * seconds / distance)
   */
  protected void setKV(double kV) {
    feedforwardController = new ArmFeedforward(
      feedforwardController.ks,
      feedforwardController.kg,
      kV,
      feedforwardController.ka
    );
  }

  /**
   * Sets A gain of feedforward controller
   *
   * @param kA A gain, models how voltage should affect acceleration (volts * seconds^2 / distance)
   */
  protected void setKA(double kA) {
    feedforwardController = new ArmFeedforward(
      feedforwardController.ks,
      feedforwardController.kg,
      feedforwardController.kv,
      kA
    );
  }

  /**
   * Sets PPID controller
   *
   * @param ppidController ProfiledPIDController instance
   */
  protected void setPPIDController(ProfiledPIDController ppidController) {
    this.ppidController = ppidController;
  }

  /**
   * Sets PPID gains
   *
   * @param kP P gain
   * @param kI I gain
   * @param kD D gain
   */
  protected void setPPIDGains(double kP, double kI, double kD) {
    ppidController.setP(kP);
    ppidController.setI(kI);
    ppidController.setD(kD);
  }

  /**
   * Sets PPID constraints
   *
   * @param constraints Constraints
   */
  protected void setPPIDConstraints(Constraints constraints) {
    ppidController.setConstraints(constraints);
  }

  /**
   * Sets max velocity of PPID controller
   *
   * @param maxVelocity Max velocity (rads/s)
   */
  protected void setMaxVelocity(double maxVelocity) {
    ppidController.setConstraints(
      new Constraints(
        maxVelocity,
        ppidController.getConstraints().maxAcceleration
      )
    );
  }

  /**
   * Sets max acceleration of PPID controller
   *
   * @param maxAcceleration Max acceleration (rads/s^2)
   */
  protected void setMaxAcceleration(double maxAcceleration) {
    ppidController.setConstraints(
      new Constraints(
        ppidController.getConstraints().maxVelocity,
        maxAcceleration
      )
    );
  }

  /**
   * Sets tolerance of PPID controller
   *
   * @param tolerance Tolerance (rads)
   */
  protected void setTolerance(double tolerance) {
    ppidController.setTolerance(tolerance);
  }

  /**
   * Gets the current angle as a Rotation2d
   *
   * @return Current angle
   */
  protected Rotation2d getAngle() {
    // Check if any motors have been added
    if (motors.isEmpty()) {
      // Throw error and return
      System.out.println("[ERROR] No motors supplied to SingleJointSubsystem!");
      return new Rotation2d();
    }

    // Return the current pivot motor sensor position as a Rotation2d
    return new Rotation2d(motors.get(0).getFirst().getPosition());
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

    // Set pivot motor setpoint
    ppidController.setGoal(desiredAngle.getRadians());
  }

  /**
   * Runs periodic PPIDF code
   */
  @Override
  public void periodic() {
    // Get current angle
    Rotation2d curAngle = getAngle();

    // Check if current state is closed loop
    if (state == SingleJointSubsystemState.CLOSED_LOOP) {
      // Calculate PPIDF output
      double ppidfOutput = ppidController.calculate(
        curAngle.getRadians()) + (
        feedforwardController.calculate(
          ppidController.getSetpoint().position,
          ppidController.getSetpoint().velocity
        )
      );

      // Set motor voltages to PPIDF output
      setVoltage(ppidfOutput);
    }
  }
}
