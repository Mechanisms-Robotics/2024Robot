package com.mechlib.swerve;

import com.mechlib.util.MechMath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * MechLib HeadingController class
 *
 * Used to stabilize or lock the heading of a SwerveDrive
 */
public class HeadingController {
  private final PIDController stabilizeController; // Stabilization PID controller
  private final PIDController lockController; // Lock PID controller

  private final double maxAngularVelocity; // Max angular velocity (rads/s)
  private final SlewRateLimiter omegaLimiter; // Omega acceleration limiter

  private Rotation2d prevHeading = new Rotation2d(); // Previous heading
  private double prevOmega = 0; // Previous omega (rads/s)

  /**
   * HeadingController constructor
   *
   * @param configuration HeadingControllerConfiguration instance
   */
  public HeadingController(HeadingControllerConfiguration configuration) {
    // Instantiate stabilization PIDController
    stabilizeController = new PIDController(
      configuration.stabilizeKP,
      configuration.stabilizeKI,
      configuration.stabilizeKD
    );

    // Set stabilization tolerance
    stabilizeController.setTolerance(configuration.stabilizeTolerance);

    // Instantiate lock PIDController
    lockController = new PIDController(
      configuration.lockKP,
      configuration.lockKI,
      configuration.lockKD
    );

    // Set lock tolerance
    lockController.setTolerance(configuration.lockTolerance);

    // Set max angular velocity
    this.maxAngularVelocity = configuration.maxAngularVelocity;

    // Instantiate omega SlewRateLimiter
    omegaLimiter = new SlewRateLimiter(
      configuration.maxAngularAcceleration
    );
  }

  /**
   * Stabilizes the SwerveDrive heading
   *
   * @param vx Velocity in the x-axis (m/s)
   * @param vy Velocity in the y-axis (m/s)
   * @param omega Angular velocity (rads/s)
   * @param heading Heading
   *
   * @return Stabilized omega (rads/s)
   */
  public double stabilize(double vx, double vy, double omega, Rotation2d heading) {
    // Set stabilizedOmega to omega
    double stabilizedOmega = omega;

    // Check if omega went from non-zero to zero
    if (omega == 0 && prevOmega != 0) {
      // If so save the last heading
      prevHeading = heading;
    }

    // Save the current omega for next loop
    prevOmega = omega;
    SmartDashboard.putNumber("prevHeading", prevHeading.getDegrees());

    // Check if current omega is zero and translational velocity is at least 0.25 m/s
    if (omega == 0 && Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2)) >= 0.25) {
      if (Math.abs(prevHeading.getRadians() - heading.getRadians()) > 1 * Math.PI) {
        System.out.println("PrevHeading: " + prevHeading.getDegrees() + " heading: " + heading.getDegrees() + " diff: (" + Math.abs(prevHeading.getDegrees() - heading.getDegrees()) + ")");
        return 0;
      }
      // If so change omega such that the heading is stabilized
      stabilizedOmega = stabilizeController.calculate(
        heading.getRadians(),
        MechMath.optimizeRotation(
                heading,
                prevHeading
        ).getRadians()
      );
    }

    // Return stabilizedOmega
    return stabilizedOmega;
  }

  /**
   * Locks the SwerveDrive onto a given heading
   *
   * @param curHeading Current heading
   * @param desiredHeading Desired heading
   *
   * @return Omega (rads/s)
   */
  public double lock(Rotation2d curHeading, Rotation2d desiredHeading) {
    // Optimize desired heading to minimize travel distance
    Rotation2d optimalHeading = MechMath.optimizeRotation(
      curHeading,
      desiredHeading
    );

    // Calculate omega
    double omega = lockController.calculate(
      curHeading.getRadians(),
      optimalHeading.getRadians()
    );

    // Limit angular velocity
    omega = MathUtil.clamp(omega, -maxAngularVelocity, maxAngularVelocity);

    // Limit angular acceleration
    omega = omegaLimiter.calculate(omega);

    // Return omega
    return omega;
  }
}
