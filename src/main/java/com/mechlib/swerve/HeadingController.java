package com.mechlib.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * MechLib HeadingController class
 *
 * Used to stabilize or lock the heading of a SwerveDrive
 */
public class HeadingController {
  private final PIDController stabilizeController; // Stabilization PID controller
  private final PIDController lockController; // Lock PID controller

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

    // Instantiate lock PIDController
    lockController = new PIDController(
      configuration.lockKP,
      configuration.lockKI,
      configuration.lockKD
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

    // Check if current omega is zero and translational velocity is at least 0.25 m/s
    if (omega == 0 && Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2)) >= 0.25) {
      // If so change omega such that the heading is stabilized
      stabilizedOmega = -stabilizeController.calculate(
        heading.getRadians(),
        prevHeading.getRadians()
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
    // Calculate omega
    double omega = lockController.calculate(
      curHeading.getRadians(),
      desiredHeading.getRadians()
    );

    // Wrap around if needed
    if (Math.signum(curHeading.getRadians()) != Math.signum(desiredHeading.getRadians()))
      omega *= -1;

    // Return omega
    return omega;
  }
}
