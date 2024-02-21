package frc.robot.subsystems;

import com.mechlib.hardware.CANCoder;
import com.mechlib.hardware.TalonFX;
import com.mechlib.subsystems.SingleJointSubystem;
import com.mechlib.util.MechUnits;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SingleJointSubystem {
    private static final double kMagnetOffset = 0;
    // right arm TalonFX motor (with the Can Encoder)
    private final TalonFX armMotor = new TalonFX(13, new CANCoder(13, kMagnetOffset));
    // left arm TalonFX motor
    private final TalonFX armFollower = new TalonFX(12);
    private static final ArmFeedforward kArmFeedForward = new ArmFeedforward(0.2, 0, 0, 0);
    private static final ProfiledPIDController kProfiledPIDController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(Math.PI / 4, 1));
    private static final double kTolerance = Math.toRadians(5);
    private static final Rotation2d kStowed = Rotation2d.fromDegrees(20);
    private static final Rotation2d kIntaking = Rotation2d.fromDegrees(30);
    private static final Rotation2d kShooting = Rotation2d.fromDegrees(10);
    private static final double kMotorRatio = 1;
    private static final double kSensorRatio = 1;
    private static final Rotation2d kForwardLimit = Rotation2d.fromDegrees(10000);
    private static final Rotation2d kReverseLimit = Rotation2d.fromDegrees(-10000);
    private static final double kLeftRightRatio = 0.8;

    public Arm() {
        addMotor(armMotor, true);
        addMotor(armFollower, false);
        setState(SingleJointSubsystemState.OPEN_LOOP);
        configureMotors();
    }

    @Override
    protected void configureMotors() {
        super.configureMotors();
        setFeedforwardController(kArmFeedForward);
        setPPIDController(kProfiledPIDController);
        armMotor.setPositionUnitsFunction((Double rotations) -> MechUnits.rotationsToRadians(rotations, kSensorRatio));
        armMotor.setVelocityUnitsFunction((Double rotations) -> MechUnits.rotationsToRadians(rotations, kSensorRatio));
        setLimits(kReverseLimit, kForwardLimit, kMotorRatio);
    }

    public void stow() {
        pivotTo(kStowed);
    }
    public void intake() {
        pivotTo(kIntaking);
    }
    public void shoot() {
        pivotTo(kShooting);
    }
    public void down() {
        double percent = -0.05;
        armMotor.setPercent(percent);
        armFollower.setPercent(-percent * kLeftRightRatio);
    }
    public void stop() {
        armMotor.setPercent(0);
        armFollower.setPercent(0);
    }
    public void up() {
        double percent = 0.05;
        armMotor.setPercent(percent);
        armFollower.setPercent(-percent * kLeftRightRatio);
    }
    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("[Arm] CurrentAngle", getAngle().getDegrees());
        SmartDashboard.putNumber("[Arm] DesiredAngle", getDesiredAngle().getDegrees());
    }
}
