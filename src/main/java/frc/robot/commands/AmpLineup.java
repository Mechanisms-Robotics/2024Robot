package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.LimeLight.Tag;
import frc.robot.subsystems.Swerve;

import java.util.function.Supplier;

/**
 * Turns toward the subwoofer on execute.
 * Decreased the drive speed to kMaxVelocity and overrides the turning.
 * The wrist aims to account for distance using and interpolated tree map.
 */
public class AmpLineup extends Command {
    private final Swerve swerve;
    private final LimeLight limeLight;
    private static final double kMaxVelocity = 1;
    private static final double kMaxAcceleration = 1;
    private static final SlewRateLimiter xLimiter = new SlewRateLimiter(kMaxAcceleration);
    private static final SlewRateLimiter yLimiter = new SlewRateLimiter(kMaxAcceleration);
    // TODO: make shu the rite cuh
    private static final Rotation2d kAmpHeading = Rotation2d.fromDegrees(-90);
    // TODO: make it lower hu
    private static final double kTolerance = 5;
    private final Supplier<Double> xVal;
    private final Supplier<Double> yVal;
    private static final double kDeadBand = 0.1;
    // TODO: tune PID controller
    private static final ProfiledPIDController strafeController = new ProfiledPIDController(.1, 0, 0, new Constraints(kMaxVelocity, kMaxAcceleration));

    /**
     * Initializes the swerve, limelight and armWrist and gets x, y and rotational joystick values
     *
     * @param swerve instance of swerve, needed for turning toward the target and driving
     * @param limeLight instance of limelight, needed for finding the aprilTag
     *
     * @param xVal x drive component
     * @param yVal y drive component
     */
    public AmpLineup(Swerve swerve, LimeLight limeLight,
                     Supplier<Double> xVal, Supplier<Double> yVal) {
        this.swerve = swerve;
        this.limeLight = limeLight;
        this.xVal = xVal;
        this.yVal = yVal;
        addRequirements(swerve, limeLight);
    }


    /**
     * Deadbands the controller.
     * Returns 0 if the input is not more than kDeadBand.
     * This is to account for joy con drift.
     *
     * @param val value to be deadbanded
     * @return deadbanded value
     */
    private double deadBand(double val) {
        return MathUtil.isNear(0, val, kDeadBand) ? 0 : val;
    }

    @Override
    public void execute() {
        double vx = xLimiter.calculate(deadBand(xVal.get()) * kMaxVelocity);
        double vy = yLimiter.calculate(deadBand(yVal.get()) * kMaxVelocity);
        Tag.data amp = limeLight.getData().ampTag();

        swerve.lockHeading(kAmpHeading);
        if (!MathUtil.isNear(kAmpHeading.getDegrees(), swerve.getHeading().getDegrees(), kTolerance)
            || !amp.hasTarget()) {
            swerve.drive(vx, vy, 0);
            return;
        }
        vx = strafeController.calculate(amp.yaw(), 0);
        swerve.drive(vx, vy, 0);
    }
}