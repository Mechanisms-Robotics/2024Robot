package frc.robot.subsystems;

import com.mechlib.hardware.CANCoder;
import com.mechlib.hardware.TalonFX;
import com.mechlib.util.MechUnits;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    private static final boolean kOpenLoop = true;

    private static final double kMagnetOffset = 0;
    // right arm TalonFX motor (with the Can Encoder)
    private final TalonFX wristMotor = new TalonFX(17, new CANCoder(17, kMagnetOffset));
    // left arm TalonFX motor
    private static final ArmFeedforward kArmFeedForward = new ArmFeedforward(0.2, 0, 0, 0);
    private static final ProfiledPIDController kPPIDController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(Math.PI / 4, 1));
    private static final double kTolerance = Math.toRadians(5);
    private static final Rotation2d kStowed = Rotation2d.fromDegrees(20);
    private static final Rotation2d kIntaking = Rotation2d.fromDegrees(30);
    private static final Rotation2d kShooting = Rotation2d.fromDegrees(10);
    private static final double kSensorRatio = 0;
    private static final double kMotorRatio = 0 * kSensorRatio;
    private static final Rotation2d kForwardLimit = Rotation2d.fromDegrees(10000);
    private static final Rotation2d kReverseLimit = Rotation2d.fromDegrees(-10000);


    public Wrist() {
        configureMotors();
    }

    protected void configureMotors() {
        wristMotor.brakeMode();

        wristMotor.setPositionUnitsFunction((Double rotations) -> MechUnits.rotationsToRadians(rotations, kSensorRatio));
        wristMotor.setVelocityUnitsFunction((Double rotations) -> MechUnits.rotationsToRadians(rotations, kSensorRatio));

        wristMotor.setSensorInverted(true);
        wristMotor.setInverted(true);

        setLimits();
    }

    private void pivotTo(Rotation2d rotation) {
        kPPIDController.setGoal(rotation.getRadians());
        double PPIDFOutput = kPPIDController.calculate(wristMotor.getRelativePosition()) + kArmFeedForward.calculate(kPPIDController.getSetpoint().position, kPPIDController.getSetpoint().velocity);
        wristMotor.setVoltage(PPIDFOutput);
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
    public void stop() {
        setVoltage(0);
    }

    public void setVoltage(double voltage) {
        wristMotor.setVoltage(voltage);
    }

    public void setLimits() {
        wristMotor.setInternalSensorPosition(MechUnits.radiansToRotations(wristMotor.getRelativePosition(), kMotorRatio));
        wristMotor.setSoftLimits(kReverseLimit.getRotations(), kForwardLimit.getRotations());
    }

    @Override
    public void periodic() {
        if (!kOpenLoop) {
            wristMotor.periodicPIDF(wristMotor.getRelativePosition());
        }
        SmartDashboard.putNumber("[Wrist] position", wristMotor.getRelativePosition());
    }
}
