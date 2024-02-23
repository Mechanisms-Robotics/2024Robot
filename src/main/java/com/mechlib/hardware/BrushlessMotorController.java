package com.mechlib.hardware;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import java.util.function.Function;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;

/**
 * MechLib BrushlessMotorController Class
 *
 * Contains all of the abstract code for a brushless motor controller
 */
public abstract class BrushlessMotorController {
  // ID
  protected int id;

  // Followers
  protected ArrayList<Pair<BrushlessMotorController, Boolean>> followers = new ArrayList<>();

  // CANCoder
  protected CANCoder canCoder = null;

  // Unit functions
  protected Function<Double, Double> positionUnitsFunction = (Double x) -> x;
  protected Function<Double, Double> velocityUnitsFunction = (Double x) -> x;

  // Feedforward Controller
  protected SimpleMotorFeedforward feedforwardController = new SimpleMotorFeedforward(
    0.0,
    0.0,
    0.0
  );

  // PID Controller
  protected final PIDController pidController = new PIDController(
    0.0,
    0.0,
    0.0
  );

  // PPID Controller
  protected final ProfiledPIDController ppidController = new ProfiledPIDController(
    0.0,
    0.0,
    0.0,
    new Constraints(0.0, 0.0)
  );

  private double tolerance = 0.0; // Tolerance of PID controllers

  private double maxVelocity = 0.0; // Max velocity
  private double maxAcceleration = 0.0; // Max acceleration

  protected double simDistance = 0.0; // Simulated distance (ticks)

  /**
   * BrushlessMotorController constructor
   *
   * @param id CAN ID of motor controller
   */
  public BrushlessMotorController(int id) {
    // Set ID
    this.id = id;
  }

  /**
   * BrushlessMotorController constructor
   *
   * @param id CAN ID of motor controller
   * @param canCoder CANCoder to use instead of internal encoder
   */
  public BrushlessMotorController(int id, CANCoder canCoder) {
    // Set ID
    this.id = id;

    // Set CANCoder
    this.canCoder = canCoder;
  }

  /**
   * Sets the percentage of the motor to a given value
   *
   * @param percent Percent power to run motor at (-1.0 to 1.0)
   */
  public void setPercent(double percent) {}

  /**
   * Sets the voltage of the motor to a given value
   *
   * @param voltage Number of volts to run motor at. (-12.0 to 12.0)
   */
  public void setVoltage(double voltage) {}

  /**
   * Stops running motor
   */
  public void stop() {
    setPercent(0.0);
  }

  /**
   * Sets the motor inverted flag
   *
   * @param inverted Whether the motor is inverted
   */
  public void setInverted(boolean inverted) {}

  /**
   * Toggles motor inversion
   */
  public void invert() {}

  /**
   * Sets the sensor inverted flag
   *
   * @param sensorInverted Whether the sensor is inverted
   */
  public void setSensorInverted(boolean sensorInverted) {}

  /**
   * Toggles sensor inversion
   */
  public void invertSensor() {}

  /**
   * Toggles coast mode
   */
  public void coastMode() {}

  /**
   * Toggles brake mode
   */
  public void brakeMode() {}

  /**
   * Sets the current limit of the motor controller
   *
   * @param limit The amount of current to limit to (amps)
   */
  public void setCurrentLimit(double limit) {}

  /**
   * Sets voltage compensation for the motor controller
   *
   * @param nominalVoltage Nominal voltage to run motor at
   */
  public void setVoltageCompensation(double nominalVoltage) {}

  /**
   * Sets position units function
   *
   * @param positionUnitsFunction Function that takes in native units and returns desired units
   */
  public void setPositionUnitsFunction(Function<Double, Double> positionUnitsFunction) {
    // Set position units function
    this.positionUnitsFunction = positionUnitsFunction;

    // If using a CANCoder set its position units function
    if (canCoder != null)
      canCoder.setPositionUnitsFunction(positionUnitsFunction);
  }

  /**
   * Sets velocity units function
   *
   * @param velocityUnitsFunction Function that takes in native units and returns desired units
   */
  public void setVelocityUnitsFunction(Function<Double, Double> velocityUnitsFunction) {
    // Set velocity units function
    this.velocityUnitsFunction = velocityUnitsFunction;

    // If using a CANCoder set its velocity units function
    if (canCoder != null)
      canCoder.setVelocityUnitsFunction(velocityUnitsFunction);
  }

  /**
   * Sets feedforward controller
   *
   * @param feedforwardController SimpleMotorFeedforward instance
   */
  public void setFeedforwardController(SimpleMotorFeedforward feedforwardController) {
    this.feedforwardController = feedforwardController;
  }

  /**
   * Sets feedforward gains
   *
   * @param kS S gain, overcomes static friction (volts)
   * @param kV V gain, models how voltage should affect velocity (volts * seconds / distance)
   * @param kA A gain, models how voltage should affect acceleration (volts * seconds^2 / distance)
   */
  public void setFeedforwardGains(double kS, double kV, double kA) {
    this.feedforwardController = new SimpleMotorFeedforward(kS, kV, kA);
  }

  /**
   * Sets S gain of feedforward controller
   *
   * @param kS S gain, overcomes static friction (volts)
   */
  public void setKS(double kS) {
    this.feedforwardController = new SimpleMotorFeedforward(
      kS,
      this.feedforwardController.kv,
      this.feedforwardController.ka
    );
  }

  /**
   * Sets V gain of feedforward controller
   *
   * @param kV V gain, models how voltage should affect velocity (volts * seconds / distance)
   */
  public void setKV(double kV) {
    this.feedforwardController = new SimpleMotorFeedforward(
      this.feedforwardController.ks,
      kV,
      this.feedforwardController.ka
    );
  }

  /**
   * Sets A gain of feedforward controller
   *
   * @param kA A gain, models how voltage should affect acceleration (volts * seconds^2 / distance)
   */
  public void setKA(double kA) {
    this.feedforwardController = new SimpleMotorFeedforward(
      this.feedforwardController.ks,
      this.feedforwardController.kv,
      kA
    );
  }

  /**
   * Updates the PIDF and PPIDF, kP values
   *
   * @param kP The P gain
   */
  public void setKP(double kP) {
    // Set the P gains
    this.pidController.setP(kP);
    this.ppidController.setP(kP);
  }

  /**
   * Updates the PIDF and PPIDF, kI values
   *
   * @param kI The I gain
   */
  public void setKI(double kI) {
    // Set the I gains
    this.pidController.setI(kI);
    this.ppidController.setI(kI);
  }

  /**
   * Updates the PIDF and PPIDF, kD values
   *
   * @param kD The D gain
   */
  public void setKD(double kD) {
    // Set the D gains
    this.pidController.setD(kD);
    this.ppidController.setD(kD);
  }

  /**
   * Sets the max velocity of PPIDF controller
   *
   * @param maxVelocity Max velocity
   */
  public void setMaxVelocity(double maxVelocity) {
    // Set max velocity
    this.maxVelocity = maxVelocity;

    // Reset the PPID constraints with updated max velocity
    ppidController.setConstraints(
      new Constraints(
        maxVelocity,
        maxAcceleration
      )
    );
  }

  /**
   * Sets the max acceleration of PPIDF controller
   *
   * @param maxAcceleration Max acceleration (?/?^2)
   */
  public void setMaxAcceleration(double maxAcceleration) {
    // Set max acceleration
    this.maxAcceleration = maxAcceleration;

    // Reset the PPID constraints with updated max acceleration
    ppidController.setConstraints(
      new Constraints(
        maxVelocity,
        maxAcceleration
      )
    );
  }

  /**
   * Zeroes encoder
   */
  public void zero() {}

  /**
   * Sets position of internal sensor in rotations
   *
   * @param position Position (rotations)
   */
  public void setInternalSensorPosition(double position) {}

  /**
   * Sets the setpoint to a given value
   *
   * @param setpoint Setpoint value
   */
  public void setSetpoint(double setpoint) {
    // Set setpoints
    this.pidController.setSetpoint(setpoint);
    this.ppidController.setGoal(setpoint);
  }

  /**
   * Sets the tolerance of the PID controllers
   *
   * @param tolerance Tolerance of PID controllers
   */
  public void setTolerance(double tolerance) {
    // Set tolerance
    this.tolerance = tolerance;
  }

  /**
   * Adds a follower, will run all followers in sync
   * @param brushlessMotorController Follower motor controller
   * @param inverted Whether the follower motor controller is inverted
   */
  public void addFollower(BrushlessMotorController brushlessMotorController, boolean inverted) {
    // Add follower
    followers.add(new Pair<>(brushlessMotorController, inverted));
  }

  /**
   * Adds motor controller as a follower to a specified motor controller
   * @param brushlessMotorController Motor controller to follow
   * @param inverted Whether this motor controller is inverted
   */
  public void follow(BrushlessMotorController brushlessMotorController, boolean inverted) {
    // Add this as follower to brushlessMotorController
    brushlessMotorController.addFollower(this, inverted);
  }

  /**
   * Updates all followers given the main motor percent output
   *
   * @param percent Percent output of main motor
   */
  public void updateFollowers(double percent) {
    // Loop through followers
    for (Pair<BrushlessMotorController, Boolean> follower : followers) {
      // Get follower motor controller and inversion
      BrushlessMotorController followerMotorController = follower.getFirst();
      boolean inverted = follower.getSecond();

      // Set follower percent output
      followerMotorController.setPercent(percent * (inverted ? -1.0 : 1.0));
    }
  }

  /**
   * Updates all followers given the main motor voltage output
   *
   * @param voltage Voltage output of main motor
   */
  public void updateFollowersVoltage(double voltage) {
    // Loop through followers
    for (Pair<BrushlessMotorController, Boolean> follower : followers) {
      // Get follower motor controller and inversion
      BrushlessMotorController followerMotorController = follower.getFirst();
      boolean inverted = follower.getSecond();

      // Set follower voltage
      followerMotorController.setVoltage(voltage * (inverted ? -1.0 : 1.0));
    }
  }

  /**
   * Gets the current setpoint
   *
   * @return Current setpoint of PIDF controller
   */
  public double getSetpoint() {
    // Get the current setpoint
    return this.pidController.getSetpoint();
  }

  /**
   * Runs the periodic code for the PID controller given a current measurement
   *
   * @param currentMeasurement Current measurement
   */
  public void periodicPID(double currentMeasurement) {
    // Calculate PID output
    double pidOutput = pidController.calculate(currentMeasurement);

    // Set the percentage of the motor to the PIDF output
    setPercent(pidOutput);
  }

  /**
   * Runs the periodic code for the feedforward and PID controller given a current measurement
   *
   * @param currentMeasurement Current measurement
   */
  public void periodicPIDF(double currentMeasurement) {
    // Calculate PIDF output
    double pidfOutput = pidController.calculate(currentMeasurement) + (
      feedforwardController.calculate(
        pidController.getSetpoint()
      ) / RobotController.getBatteryVoltage()
    );

    // Set the percentage of the motor to the PIDF output
    setPercent(pidfOutput);
  }

  /**
   * Runs the periodic code for the feedforward and PID controller given a current position and
   * current velocity
   *
   * @param currentPosition Current position
   * @param currentVelocity Current velocity
   */
  public void periodicPIDF(double currentPosition, double currentVelocity) {
    // Calculate PIDF output
    double pidfOutput = pidController.calculate(currentPosition) + (
      feedforwardController.calculate(
        currentVelocity
      ) / RobotController.getBatteryVoltage()
    );

    // Set the percentage of the motor to the PIDF output
    setPercent(pidfOutput);
  }

  /**
   * Runs the periodic code for the feedforward and PPID controller given a current measurement
   *
   * @param currentMeasurement Current measurement
   */
  public void periodicPPIDF(double currentMeasurement) {
    // Calculate PPIDF output
    double ppidfOutput = ppidController.calculate(currentMeasurement) + (
      feedforwardController.calculate(
        pidController.getSetpoint()
      ) / RobotController.getBatteryVoltage()
    );

    // Set the percentage of the motor to the PPIDF output
    setPercent(ppidfOutput);
  }

  /**
   * Runs the periodic code for the feedforward and PPID controller given a current measurement
   *
   * @param currentPosition Current position
   * @param currentVelocity Current velocity
   */
  public void periodicPPIDF(double currentPosition, double currentVelocity) {
    // Calculate PPIDF output
    double ppidfOutput = ppidController.calculate(currentPosition) + (
      feedforwardController.calculate(
        currentVelocity
      ) / RobotController.getBatteryVoltage()
    );

    // Set the percentage of the motor to the PPIDF output
    setPercent(ppidfOutput);
  }

  /**
   * Sets soft limit on motor
   *
   * @param lowerLimit Lower limit (native units)
   * @param higherLimit Higher limit (native units)
   */
  public void setSoftLimits(double lowerLimit, double higherLimit) {}

  /**
   * Returns the relative position of the CANCoder if one exists or the position of the motor's
   * internal encoder.
   *
   * @return Position of encoder
   */
  public double getPosition() { return 0.0; }

  /**
   * Returns the absolute position of the CANCoder if one exists
   *
   * @return Absolute position of encoder
   */
  public double getAbsolutePosition() { return 0.0; }

  /**
   * Returns the distance the encoder has moved in ticks
   * (Equivalent to getPosition if not running in simulation)
   *
   * @return Distance
   */
  public double getDistance() {
    // Check if this is a simulation
    if (Robot.isSimulation())
      // If so return the simulated distance
      return simDistance;

    // Otherwise just return the getPosition output
    return getPosition();
  }

  /**
   * Returns the velocity of the encoder if one exists
   *
   * @return Velocity of encoder
   */
  public double getVelocity() { return 0.0; }
}
