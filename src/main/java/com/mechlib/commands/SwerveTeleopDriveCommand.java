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
  // Constants
  private static final double MAX_VELOCITY = 1.5; // (m/s)
  private static final double MAX_ACCEL = 3.0; // (m/s^2)
  private static final double MAX_ANGULAR_VELOCITY = Math.PI; // (rads/s)
  private static final double MAX_ANGULAR_ACCEL = Math.PI; // (rads/s^2)

  private static final double JOYSTICK_DEADBAND = 0.1; // (percent)

  // Swerve instance
  private final SwerveDrive swerve;

  // Suppliers
  private final Supplier<Double> xSupplier;
  private final Supplier<Double> ySupplier;
  private final Supplier<Double> rSupplier;

  // X velocity rate limiter
  private final SlewRateLimiter vxLimiter = new SlewRateLimiter(
    MAX_ACCEL
  );

  // Y velocity rate limiter
  private final SlewRateLimiter vyLimiter = new SlewRateLimiter(
    MAX_ACCEL
  );

  // Angular velocity rate limiter
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(
    MAX_ANGULAR_ACCEL
  );

  /**
   * SwerveTeleopDriveCommand constructor
   *
   * @param swerve Instance of SwerveDrive
   */
  public SwerveTeleopDriveCommand(
    SwerveDrive swerve,
    Supplier<Double> xSupplier,
    Supplier<Double> ySupplier,
    Supplier<Double> rSupplier
  ) {
    // Set swerve
    this.swerve = swerve;

    // Set suppliers
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rSupplier = rSupplier;

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
    return Math.abs(value) >= JOYSTICK_DEADBAND ? value : 0.0;
  }

  @Override
  public void execute() {
    // Calculate velocities
    double vx = vxLimiter.calculate(deadband(xSupplier.get()) * MAX_VELOCITY);
    double vy = vyLimiter.calculate(deadband(ySupplier.get()) * MAX_VELOCITY);
    double omega = omegaLimiter.calculate(deadband(rSupplier.get()) * MAX_ANGULAR_VELOCITY);

    // Drive swerve at calculated velocities
    swerve.drive(vx, vy, omega);
  }
}
