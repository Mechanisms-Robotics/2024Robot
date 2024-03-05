package frc.robot.subsystems;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.mechlib.hardware.CANCoder;
import com.mechlib.hardware.TalonFX;
import com.mechlib.subsystems.SingleJointSubystem;
import com.mechlib.util.MechUnits;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Controls the motor that pivots Gerald
 */
public class Wrist extends SingleJointSubystem {
    private static final double kSensorRatio = 38.0/16.0;
    private static final double kMotorRatio = 25.0 * kSensorRatio;
    private static final double kStartRotations = 0.29027778;
    // left arm motor magnet offset (acquired in Phoenix Tuner X)
    private static final double kMagnetOffset = (kSensorRatio * kStartRotations) - 0.609619;
    // right arm motor magnet offset
    // right arm TalonFX motor and it's can coder
    private final TalonFX WristMotor = new TalonFX(17, new CANCoder(17, kMagnetOffset, AbsoluteSensorRangeValue.Unsigned_0To1, SensorDirectionValue.CounterClockwise_Positive));
    // feed forward controller for the arm
    /* PID controller for the right and left arm, which will always have the same values they are different to account
       for different mechanical structures, such as belt tensioning */
    private static final double kTolerance = Math.toRadians(0.5);
    private static final Rotation2d kStowed = Rotation2d.fromDegrees(90);
    private static final Rotation2d kIntaking = Rotation2d.fromDegrees(90);
    private static final Rotation2d kSubwooferHigh = Rotation2d.fromDegrees(90);
    private static final Rotation2d kSubwooferLow = Rotation2d.fromDegrees(95);
    private static final Rotation2d kPodiumHigh = Rotation2d.fromDegrees(115);
    private static final Rotation2d kPodiumLow = kSubwooferLow;
    private static final Rotation2d kAmp = Rotation2d.fromDegrees(90);
    private static final Rotation2d kShuttle = Rotation2d.fromDegrees((125));
    private static final Rotation2d kForwardLimit = Rotation2d.fromDegrees(125);
    private static final Rotation2d kReverseLimit = Rotation2d.fromDegrees(85);
    private boolean disabled = false;
    private double wristAdjustment = 0;
    private final SendableChooser<Double> adjustmentAmount = new SendableChooser<>();

    public Wrist() {
        addMotor(WristMotor, true);
        setCurrentLimit(40.0);
        setVoltageCompensation(10.0);
        setState(SingleJointSubsystemState.CLOSED_LOOP);
        setPositionUnitsFunction((rotations) -> MechUnits.rotationsToRadians(rotations, kSensorRatio));
        setVelocityUnitsFunction((rotations) -> MechUnits.rotationsToRadians(rotations, kSensorRatio));
        setLimits(kReverseLimit, kForwardLimit, kMotorRatio);
        setFeedforwardGains(0.15, 0, 0.0, 0.0);
        setPPIDGains(0.4, 0.0, 0.0);
        setPPIDConstraints(Math.PI, 2*Math.PI);
        setTolerance(kTolerance);

        adjustmentAmount.addOption("^", -1./2.);
        adjustmentAmount.setDefaultOption("None", 0.);
        adjustmentAmount.addOption("v", 1./2.);
        SmartDashboard.putData("Shot Adjustment", adjustmentAmount);
    }

    /**
     * Set arm to the stow position
     */
    public void stow() {
        pivotTo(kStowed);
    }

    /**
     * Set arm to the intake position
     */
    public void intake() {
        pivotTo(kIntaking);
    }

    /**
     * Set arm to the shoot high subwoofer position
     */
    public void shootHighSubwoofer() {
        pivotTo(kSubwooferHigh.plus(Rotation2d.fromDegrees(wristAdjustment)));
    }

    /**
     * Set arm to the shoot low subwoofer position
     */
    public void shootLowSubwoofer() {
        pivotTo(kSubwooferLow.plus(Rotation2d.fromDegrees(wristAdjustment)));
    }

    /**
     * Set arm to the shoot high podium position
     */
    public void shootHighPodium() {
        pivotTo(kPodiumHigh.plus(Rotation2d.fromDegrees(wristAdjustment)));
    }

    /**
     * Set arm to the shoot low podium position
     */
    public void shootLowPodium() {
        pivotTo(kPodiumLow.plus(Rotation2d.fromDegrees(wristAdjustment)));
    }

    /**
     * Pivot to the amp position
     */
    public void amp() {
        pivotTo(kAmp);
    }

    public void shuttle() {
        pivotTo(kShuttle);
    }

    public void aim(Rotation2d rotation) {
        pivotTo(rotation.plus(Rotation2d.fromDegrees(wristAdjustment)));
    }

    /**
     * Periodically output the data (right and left arm position) to SmartDashBoard. Do not run the arms if the robot
     * is disabled. Runs the PIDFs if the robot is in closed loop.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("[wrist] Wrist position", WristMotor.getRawPosition());
        SmartDashboard.putNumber("[wrist] current angle", getAngle().getDegrees());
        SmartDashboard.putNumber("[wrist] desired angle", getDesiredAngle().getDegrees());
        if (getAngle().getDegrees() < 0) disabled = true;
        if (disabled) return;
        super.periodic();
        wristAdjustment = adjustmentAmount.getSelected();
    }
}
