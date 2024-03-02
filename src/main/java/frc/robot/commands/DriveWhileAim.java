package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmWrist;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Swerve;

import java.util.function.Supplier;

public class DriveWhileAim extends Command {
    private final Swerve swerve;
    private final LimeLight limeLight;
    private final ArmWrist armWrist;
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
    private static final InterpolatingDoubleTreeMap wristAimMap = new InterpolatingDoubleTreeMap();
    static {
        wristAimMap.put(00.00, 90.);
        wristAimMap.put(1000., 90.);
    }
    private static final ProfiledPIDController controller = new ProfiledPIDController(.1, 0, 0, new Constraints(kMaxOmega, kMaxOmegaAcceleration));
    public DriveWhileAim(Swerve swerve, LimeLight limeLight, ArmWrist armWrist,
                         Supplier<Double> xVal, Supplier<Double> yVal, Supplier<Double> rVal) {
        this.swerve = swerve;
        this.limeLight = limeLight;
        this.armWrist = armWrist;
        this.xVal = xVal;
        this.yVal = yVal;
        this.rVal = rVal;
        addRequirements(swerve, limeLight, armWrist);
    }

    private double deadBand(double val) {
        return MathUtil.isNear(0, val, kDeadBand) ? 0 : val;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double vx = xLimiter.calculate(deadBand(xVal.get()) * kMaxVelocity);
        double vy = yLimiter.calculate(deadBand(yVal.get()) * kMaxVelocity);
        double vomega;
        if (!limeLight.hasTarget()) vomega = omegaLimiter.calculate(deadBand(rVal.get()) * kMaxOmega);
        else {
            vomega = controller.calculate(limeLight.getYaw(), 0);
            Rotation2d desiredArmRotation = Rotation2d.fromDegrees(95);
            Rotation2d desiredWristRotation = Rotation2d.fromDegrees(wristAimMap.get(limeLight.getArea()));
            armWrist.aim(desiredArmRotation, desiredWristRotation);
        }
        SmartDashboard.putNumber("[Drive While Aim] omega", vomega);
        swerve.drive(vx, vy, vomega);
    }
}
