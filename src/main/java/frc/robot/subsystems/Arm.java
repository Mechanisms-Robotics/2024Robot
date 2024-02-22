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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private static final boolean kOpenLoop = true;

    private static final double kLeftMagnetOffset = 0;
    private static final double kRightMagnetOffset = 0;
    // right arm TalonFX motor (with the Can Encoder)
    private final TalonFX rightArmMotor = new TalonFX(13, new CANCoder(13, kRightMagnetOffset));
    // left arm TalonFX motor
    private final TalonFX leftArmMotor = new TalonFX(12, new CANCoder(12, kLeftMagnetOffset));
    private static final ArmFeedforward kArmFeedForward = new ArmFeedforward(0.2, 0, 0, 0);
    private static final ProfiledPIDController kRightPPIDController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(Math.PI / 4, 1));
    private static final ProfiledPIDController kLeftPPIDController = new ProfiledPIDController(
            kRightPPIDController.getP(), kRightPPIDController.getI(), kRightPPIDController.getD(), kRightPPIDController.getConstraints());
    private static final double kTolerance = Math.toRadians(5);
    private static final Rotation2d kStowed = Rotation2d.fromDegrees(20);
    private static final Rotation2d kIntaking = Rotation2d.fromDegrees(30);
    private static final Rotation2d kShooting = Rotation2d.fromDegrees(10);
    private static final double kSensorRatio = 68.0/16.0;
    private static final double kMotorRatio = 60 * kSensorRatio;
    private static final Rotation2d kForwardLimit = Rotation2d.fromDegrees(10000);
    private static final Rotation2d kReverseLimit = Rotation2d.fromDegrees(-10000);
    private static final double kLeftRightRatio = 0.8;


    public Arm() {
        configureMotors();
    }

    protected void configureMotors() {
        rightArmMotor.brakeMode();
        leftArmMotor.brakeMode();

        rightArmMotor.setPositionUnitsFunction((Double rotations) -> MechUnits.rotationsToRadians(rotations, kSensorRatio));
        rightArmMotor.setVelocityUnitsFunction((Double rotations) -> MechUnits.rotationsToRadians(rotations, kSensorRatio));
        leftArmMotor.setPositionUnitsFunction((Double rotations) -> MechUnits.rotationsToRadians(rotations, kSensorRatio));
        leftArmMotor.setVelocityUnitsFunction((Double rotations) -> MechUnits.rotationsToRadians(rotations, kSensorRatio));

        rightArmMotor.setSensorInverted(true);
        leftArmMotor.setSensorInverted(false);
        rightArmMotor.setInverted(true);
        leftArmMotor.setInverted(false);

        setLimits();
    }

    private void pivotTo(Rotation2d rotation) {
        kRightPPIDController.setGoal(rotation.getRadians());
        kLeftPPIDController.setGoal(rotation.getRadians());
        double rightPIDFoutput = kRightPPIDController.calculate(rightArmMotor.getPosition()) + kArmFeedForward.calculate(kRightPPIDController.getSetpoint().position, kRightPPIDController.getSetpoint().velocity);
        double leftPIDFoutput = kLeftPPIDController.calculate(leftArmMotor.getPosition()) + kArmFeedForward.calculate(kLeftPPIDController.getSetpoint().position, kLeftPPIDController.getSetpoint().velocity);
        rightArmMotor.setVoltage(rightPIDFoutput);
        leftArmMotor.setVoltage(leftPIDFoutput);
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
        rightArmMotor.setVoltage(voltage);
        leftArmMotor.setVoltage(voltage);
    }

    public void setLimits() {
        rightArmMotor.setInternalSensorPosition(MechUnits.radiansToRotations(rightArmMotor.getPosition(), kMotorRatio));
        leftArmMotor.setInternalSensorPosition(MechUnits.radiansToRotations(leftArmMotor.getPosition(), kMotorRatio));
        rightArmMotor.setSoftLimits(kReverseLimit.getRotations(), kForwardLimit.getRotations());
        leftArmMotor.setSoftLimits(kReverseLimit.getRotations(), kForwardLimit.getRotations());
    }

    @Override
    public void periodic() {
        if (!kOpenLoop) {
            rightArmMotor.periodicPIDF(rightArmMotor.getPosition());
            leftArmMotor.periodicPIDF(leftArmMotor.getPosition());
        }
        SmartDashboard.putNumber("[Right Arm] position", rightArmMotor.getPosition());
        SmartDashboard.putNumber("[Left Arm] position", leftArmMotor.getPosition());
    }
}
