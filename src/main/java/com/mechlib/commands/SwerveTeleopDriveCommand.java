package com.mechlib.commands;

import com.mechlib.swerve.SwerveDrive;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

/**
 * MechLib SwerveTeleopDriveCommand
 *
 * Contains all the code required to drive a generic swerve drive
 */
public class SwerveTeleopDriveCommand extends Command {
  // Swerve instance
  private final SwerveDrive swerve;

  // Suppliers
  private final Supplier<Double> xSupplier;
  private final Supplier<Double> ySupplier;
  private final Supplier<Double> rSupplier;

  // Joystick deadband
  private final double joystickDeadband; // (percent)

  // Max velocities
  private final double maxVelocity; // (m/s)
  private final double maxAngularVelocity; // (rads/s)

  // Rate limiters
  private final SlewRateLimiter vxLimiter;
  private final SlewRateLimiter vyLimiter;
  private final SlewRateLimiter omegaLimiter;

  /**
   * SwerveTeleopDriveCommand constructor
   *
   * @param swerve Instance of SwerveDrive
   *
   * @param xSupplier Supplier of joystick left x value
   * @param ySupplier Supplier of joystick left y value
   * @param rSupplier Supplier of joystick right x value
   *
   * @param maxVelocity Max velocity (m/s)
   * @param maxAccel Max acceleration (m/s^2)
   *
   * @param maxAngularVelocity Max angular velocity (rads/s)
   * @param maxAngularAccel Max angular acceleration (rads/s^2)
   */
  public SwerveTeleopDriveCommand(
    SwerveDrive swerve,

    Supplier<Double> xSupplier,
    Supplier<Double> ySupplier,
    Supplier<Double> rSupplier,

    double joystickDeadband,

    double maxVelocity,
    double maxAccel,

    double maxAngularVelocity,
    double maxAngularAccel
  ) {
    // Set swerve
    this.swerve = swerve;

    // Set suppliers
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rSupplier = rSupplier;

    // Set joystick deadband
    this.joystickDeadband = joystickDeadband;

    // Set max velocities
    this.maxVelocity = maxVelocity;
    this.maxAngularVelocity = maxAngularVelocity;

    // Instantiate rate limiters
    this.vxLimiter = new SlewRateLimiter(maxAccel);
    this.vyLimiter = new SlewRateLimiter(maxAccel);
    this.omegaLimiter = new SlewRateLimiter(maxAngularAccel);

    // Add subsystem requirements
    addRequirements(swerve);
  }

  /**
   * Deadbands a joystick value
   *
   * @param value Joystick input value
   *
   * @return Deadbanded value
   */
  private double deadband(double value) {
    // Return value if the abs(value) is greater than JOYSTICK_DEADBAND, otherwise return 0
    return Math.abs(value) >= joystickDeadband ? value : 0.0;
  }

  @Override
  public void execute() {
    // Calculate velocities
    double vx = vxLimiter.calculate(deadband(xSupplier.get()) * maxVelocity);
    double vy = vyLimiter.calculate(deadband(ySupplier.get()) * maxVelocity);
    double omega = omegaLimiter.calculate(deadband(rSupplier.get()) * maxAngularVelocity);

    System.out.println(vx + " " + vy + " " + omega);
    // Drive swerve at calculated velocities
    swerve.drive(vx, vy, omega);
  }
}
