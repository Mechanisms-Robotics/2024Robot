package com.mechlib.hardware;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
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

  protected double feedforward = 0.0; // Feedforward value
  private boolean directionalFeedforward = false; // Should be in the direction of PID

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
    this.id = id;
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
  public void setVoltage(double voltage) {
    // Convert voltage to percent output and pass to setPercent
    setPercent(voltage / RobotController.getBatteryVoltage());
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
   * Updates the PIDF and PPIDF, kF values
   *
   * @param kF the feedforward value (volts)
   */
  public void setKF(double kF) {
    // Set feedforward
    this.feedforward = kF;
  }

  /**
   * Sets whether the feedforward should be in the same direction as the PID
   *
   * @param directionalFeedforward Directional feedforward value
   */
  public void setDirectionalFeedforward(boolean directionalFeedforward) {
    // Set directionalFeedforward
    this.directionalFeedforward = directionalFeedforward;
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
   * Gets the current setpoint
   *
   * @return Current setpoint of PIDF controller
   */
  public double getSetpoint() {
    // Get the current setpoint
    return this.pidController.getSetpoint();
  }

  /**
   * Runs the periodic code for the PIDF controller using a provided current value
   */
  public void periodicPIDF(double currentValue) {
    // Check if within tolerance
    double error = getSetpoint() - currentValue;
    if (Math.abs(error) <= tolerance) {
      // If within tolerance check for directional feedforward
      if (directionalFeedforward) {
        // If directional feedforward apply feedforward in direction of error
        setPercent(feedforward * Math.signum(error));
        SmartDashboard.putNumber("[" + id + "] PIDF Output", feedforward * Math.signum(error));
      } else {
        // If standard feedforward apply feedforward
        setPercent(feedforward);
        SmartDashboard.putNumber("[" + id + "] PIDF Output", feedforward);
      }

      // Return without calculating PIDF output
      return;
    }

    // Calculate PIDF Output
    double pidfOutput = pidController.calculate(
      currentValue
    ) + (directionalFeedforward ? feedforward * Math.signum(error) : feedforward);

    // Set the percentage of the motor to the PIDF output
    setPercent(pidfOutput);
    SmartDashboard.putNumber("[" + id + "] PIDF Output", pidfOutput);
  }

  /**
   * Runs the periodic code for the PPIDF controller using a provided current value
   */
  public void periodicPPIDF(double currentValue) {
    // Check if within tolerance
    double error = getSetpoint() - currentValue;
    if (Math.abs(error) <= tolerance) {
      // If within tolerance check for directional feedforward
      if (directionalFeedforward) {
        // If directional feedforward apply feedforward in direction of error
        setPercent(feedforward * Math.signum(error));
        SmartDashboard.putNumber("[" + id + "] PPIDF Output", feedforward * Math.signum(error));
      } else {
        // If standard feedforward apply feedforward
        setPercent(feedforward);
        SmartDashboard.putNumber("[" + id + "] PPIDF Output", feedforward);
      }

      // Return without calculating PPIDF output
      return;
    }

    // Calculate PPIDF Output
    double ppidfOutput = ppidController.calculate(
      currentValue
    ) + (directionalFeedforward ? feedforward * Math.signum(error) : feedforward);

    // Set the percentage of the motor to the PPIDF output
    setPercent(ppidfOutput);
    SmartDashboard.putNumber("[" + id + "] PPIDF Output", ppidfOutput);
  }

  /**
   * Returns the position of the internal encoder if one exists
   *
   * @return Position of internal encoder (native units)
   */
  public double getPosition() { return 0.0; }

  /**
   * Returns the distance the encoder has moved in ticks
   * (Equivalent to getPosition if not running in simulation)
   *
   * @return Distance (ticks)
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
   * Returns the velocity of the internal encoder if one exists
   *
   * @return Velocity of internal encoder (native velocity units)
   */
  public double getVelocity() { return 0.0; }
}
