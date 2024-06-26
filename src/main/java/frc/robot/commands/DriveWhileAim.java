package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmWrist;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.LimeLight.LimeLightData;
import frc.robot.subsystems.Swerve;

import java.util.function.Supplier;

/**
 * Turns toward the subwoofer on execute.
 * Decreased the drive speed to kMaxVelocity and overrides the turning.
 * The wrist aims to account for distance using and interpolated tree map.
 */
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
    private static final Rotation2d desiredArmRotation = Rotation2d.fromDegrees(60);
    /**
     * Wrist angle look up table ,
     * key: apriltag area (the percentage of the camera the april tag takes up which corresponds to distance),
     * values: wrist angle
     */
    private static final InterpolatingDoubleTreeMap wristAimMap = new InterpolatingDoubleTreeMap();
    // create the wristAimMap interpolating treemap
    static {
        wristAimMap.put(00.00, 115.);
        wristAimMap.put(0.22, 115.);
        wristAimMap.put(100.0, 115.);
    }
    private static final ProfiledPIDController controller = new ProfiledPIDController(.1, 0, 0, new Constraints(kMaxOmega, kMaxOmegaAcceleration));

    /**
     * Initializes the swerve, limelight and armWrist and gets x, y and rotational joystick values
     *
     * @param swerve instance of swerve, needed for turning toward the target and driving
     * @param limeLight instance of limelight, needed for finding the aprilTag
     * @param armWrist instance of the armWrist, needed for pivoting the wrist to account for distance
     *
     * @param xVal x drive component
     * @param yVal y drive component
     * @param rVal rotational drive component, used when there is no target found
     */
    public DriveWhileAim(Swerve swerve, LimeLight limeLight, ArmWrist armWrist,
                         Supplier<Double> xVal, Supplier<Double> yVal, Supplier<Double> rVal) {
        this.swerve = swerve;
        this.limeLight = limeLight;
        this.armWrist = armWrist;
        this.xVal = xVal;
        this.yVal = yVal;
        this.rVal = rVal;
        controller.setTolerance(2);
        addRequirements(swerve, limeLight, armWrist);
    }

    /**
     * Initializes the swerve, limelight and armWrist and passes in 0 as joystick values
     *
     * @param swerve instance of swerve, needed for turning toward the target and driving
     * @param limeLight instance of limelight, needed for finding the aprilTag
     * @param armWrist instance of the armWrist, needed for pivoting the wrist to account for distance
     */
    public DriveWhileAim(Swerve swerve, LimeLight limeLight, ArmWrist armWrist) {
        this(swerve, limeLight, armWrist, ()->0., ()->0., ()->0.);
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
        double vomega;
        LimeLightData limeLightData = limeLight.getData();

        if (!limeLightData.hasTarget()) vomega = omegaLimiter.calculate(deadBand(rVal.get()) * kMaxOmega);
        else {
            vomega = controller.calculate(limeLightData.yaw(), 0);
            Rotation2d desiredWristRotation = Rotation2d.fromDegrees(wristAimMap.get(limeLightData.area()));
            armWrist.aim(desiredArmRotation, desiredWristRotation);
        }
        SmartDashboard.putNumber("[Drive While Aim] omega", vomega);
        if (!DriverStation.isAutonomous()) swerve.drive(vx, vy, vomega);
        else swerve.drive(0, 0, vomega);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}