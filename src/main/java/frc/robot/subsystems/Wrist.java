package frc.robot.subsystems;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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

public class Wrist extends SingleJointSubystem {
    // magnet offset for the wrist motor
    private static final double kMagnetOffset = -0.0419;
    // wrist motor and can encoder
    private final TalonFX WristMotor = new TalonFX(17, new CANCoder(17, kMagnetOffset,
            AbsoluteSensorRangeValue.Unsigned_0To1, SensorDirectionValue.CounterClockwise_Positive));
    private static final double kTolerance = Math.toRadians(0.5);
    private static final Rotation2d kStowed = Rotation2d.fromDegrees(90);
    private static final Rotation2d kIntaking = Rotation2d.fromDegrees(95);
    private static final Rotation2d kShooting = Rotation2d.fromDegrees(90);
    private static final Rotation2d kSubwooferHigh = Rotation2d.fromDegrees(90);
    private static final Rotation2d kSubwooferLow = Rotation2d.fromDegrees(90);
    private static final Rotation2d kPodiumHigh = kSubwooferHigh;
    private static final Rotation2d kPodiumLow = kSubwooferLow;
    private static final Rotation2d kAmp = Rotation2d.fromDegrees(90);
    private static final double kSensorRatio = 1.0;
    private static final double kMotorRatio = 25 * kSensorRatio;
    private static final Rotation2d kForwardLimit = Rotation2d.fromDegrees(140);
    private static final Rotation2d kReverseLimit = Rotation2d.fromDegrees(90);


    public Wrist() {
        addMotor(WristMotor, true);
        setCurrentLimit(40.0);
        setVoltageCompensation(10.0);
        setState(SingleJointSubsystemState.CLOSED_LOOP);
        setPositionUnitsFunction((rotations) -> MechUnits.rotationsToRadians(rotations, kSensorRatio));
        setVelocityUnitsFunction((rotations) -> MechUnits.rotationsToRadians(rotations, kSensorRatio));
        setLimits(kReverseLimit, kForwardLimit, kMotorRatio);
        setFeedforwardGains(0.15, 0, 0.0, 0.0);
        setPPIDGains(0.5, 0.0, 0.0);
        setPPIDConstraints(Math.PI, 2*Math.PI);
        setTolerance(kTolerance);
    }
   
    /**
     * Set wrist to the stow position
     */
    public void stow() {
        pivotTo(kStowed);
    }

    /**
     * Set wrist to the intake position
     */
    public void intake() {
        pivotTo(kIntaking);
    }

    /**
     * Set arm to the shoot/amp position
     */
    public void shootHighSubwoofer() {
        pivotTo(kSubwooferHigh);
    }

    /**
     * Set arm to the shoot/amp position
     */
    public void shootLowSubwoofer() {
        pivotTo(kSubwooferLow);
    }

    /**
     * Set arm to the shoot/amp position
     */
    public void shootHighPodium() {
        pivotTo(kPodiumHigh);
    }

    /**
     * Set arm to the shoot/amp position
     */
    public void shootLowPodium() {
        pivotTo(kPodiumLow);
    }

    public void amp() {
        pivotTo(kAmp);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("[Wrist] Wrist position", WristMotor.getRawPosition());
        SmartDashboard.putNumber("[Wrist] current angle", getAngle().getDegrees());
        SmartDashboard.putNumber("[Wrist] desired angle", getDesiredAngle().getDegrees());
    }
}
