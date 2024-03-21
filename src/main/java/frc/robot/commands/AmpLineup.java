package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmWrist;
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
    private static final double kMaxOmega = Math.PI / 8;
    private static final double kMaxOmegaAcceleration = Math.PI / 4;
    private static final SlewRateLimiter xLimiter = new SlewRateLimiter(kMaxAcceleration);
    private static final SlewRateLimiter yLimiter = new SlewRateLimiter(kMaxAcceleration);
    private static final SlewRateLimiter omegaLimiter = new SlewRateLimiter(kMaxOmegaAcceleration);
    private final Supplier<Double> xVal;
    private final Supplier<Double> yVal;
    private final Supplier<Double> rVal;
    private static final double kDeadBand = 0.1;
    private static final ProfiledPIDController controller = new ProfiledPIDController(.1, 0, 0, new Constraints(kMaxOmega, kMaxOmegaAcceleration));

    /**
     * Initializes the swerve, limelight and armWrist and gets x, y and rotational joystick values
     *
     * @param swerve instance of swerve, needed for turning toward the target and driving
     * @param limeLight instance of limelight, needed for finding the aprilTag
     *
     * @param xVal x drive component
     * @param yVal y drive component
     * @param rVal rotational drive component, used when there is no target found
     */
    public AmpLineup(Swerve swerve, LimeLight limeLight,
                     Supplier<Double> xVal, Supplier<Double> yVal, Supplier<Double> rVal) {
        this.swerve = swerve;
        this.limeLight = limeLight;
        this.xVal = xVal;
        this.yVal = yVal;
        this.rVal = rVal;
        addRequirements(swerve, limeLight);
    }

    /**
     * Initializes the swerve, limelight and armWrist and passes in 0 as joystick values
     *
     * @param swerve instance of swerve, needed for turning toward the target and driving
     * @param limeLight instance of limelight, needed for finding the aprilTag
     * @param armWrist instance of the armWrist, needed for pivoting the wrist to account for distance
     */
    public AmpLineup(Swerve swerve, LimeLight limeLight, ArmWrist armWrist) {
        this(swerve, limeLight, ()->0., ()->0., ()->0.);
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
        double vx;
        double vy;
        double vomega;
        Tag.data amp = limeLight.getData().ampTag().getData();

        if (!amp.hasTarget()){
            vomega = omegaLimiter.calculate(deadBand(rVal.get()) * kMaxOmega);
            vx = xLimiter.calculate(deadBand(xVal.get()) * kMaxVelocity);
            vy = yLimiter.calculate(deadBand(yVal.get()) * kMaxVelocity);
        } else {
            vomega = controller.calculate(amp.yaw(), 0);
            if (amp.aimed()) {
                vx = 1;
            } else {
                vx = xLimiter.calculate(deadBand(xVal.get()) * kMaxVelocity);
            }
            vy = yLimiter.calculate(deadBand(yVal.get()) * kMaxVelocity);
        }
        SmartDashboard.putNumber("[Amp Lineup] omega", vomega);
        swerve.drive(vx, vy, vomega);
    }
}