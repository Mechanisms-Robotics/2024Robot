package frc.robot.subsystems;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.mechlib.hardware.CANCoder;
import com.mechlib.hardware.TalonFX;
import com.mechlib.subsystems.SingleJointSubystem;
import com.mechlib.util.MechUnits;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Controls the motor that pivots Gerald
 */
public class Wrist extends SingleJointSubystem {
    private static final double kSensorRatio = 38.0/16.0;
    private static final double kMotorRatio = 25.0 * kSensorRatio;
    // true if the arm runs in open loop, false if it runs in closed loop
    // left arm motor magnet offset (acquired in Phoenix Tuner X)
//    private static final double kMagnetOffset = -0.167969 - 0.25;
    private static final double kMagnetOffset = (kSensorRatio * 0.25) - 0.942871;
    // right arm motor magnet offset
    // right arm TalonFX motor and it's can coder
    private final TalonFX WristMotor = new TalonFX(17, new CANCoder(17, kMagnetOffset, AbsoluteSensorRangeValue.Unsigned_0To1, SensorDirectionValue.CounterClockwise_Positive));
    // feed forward controller for the arm
    /* PID controller for the right and left arm, which will always have the same values they are different to account
       for different mechanical structures, such as belt tensioning */
    private static final double kTolerance = Math.toRadians(0.5);
    private static final Rotation2d kStowed = Rotation2d.fromDegrees(90);
    private static final Rotation2d kIntaking = Rotation2d.fromDegrees(95);
    private static final Rotation2d kShooting = Rotation2d.fromDegrees(90);
    private static final Rotation2d kSubwooferHigh = Rotation2d.fromDegrees(90);
    private static final Rotation2d kSubwooferLow = Rotation2d.fromDegrees(90);
    private static final Rotation2d kPodiumHigh = kSubwooferHigh;
    private static final Rotation2d kPodiumLow = kSubwooferLow;
    private static final Rotation2d kAmp = Rotation2d.fromDegrees(90);
    private static final Rotation2d kForwardLimit = Rotation2d.fromDegrees(115);
    private static final Rotation2d kReverseLimit = Rotation2d.fromDegrees(90);
    private boolean disabled = false;

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
<<<<<<< HEAD
     * Set arm to the shoot position
     */
    public void shoot() {
        pivotTo(kShooting);
    }

    /**
=======
<<<<<<< HEAD
     * Set arm to the shoot high subwoofer position
     */
    public void shootHighSubwoofer() {
        pivotTo(kSubwooferHigh);
    }

    /**
     * Set arm to the shoot low subwoofer position
     */
    public void shootLowSubwoofer() {
        pivotTo(kSubwooferLow);
    }

    /**
     * Set arm to the shoot high podium position
     */
    public void shootHighPodium() {
        pivotTo(kPodiumHigh);
    }

    /**
     * Set arm to the shoot low podium position
     */
    public void shootLowPodium() {
        pivotTo(kPodiumLow);
    }

    /**
     * Pivot to the amp position
     */
    public void amp() {
        pivotTo(kAmp);
    }

    /**
>>>>>>> work_in_progress2/28/2024
     * Periodically output the data (right and left arm position) to SmartDashBoard. Do not run the arms if the robot
     * is disabled. Runs the PIDFs if the robot is in closed loop.
     */
    @Override
    public void periodic() {
        if (getAngle().getDegrees() < 0) disabled = true;
        if (disabled) return;
        super.periodic();
        SmartDashboard.putNumber("[wrist] Wrist position", WristMotor.getRawPosition());
        SmartDashboard.putNumber("[wrist] current angle", getAngle().getDegrees());
        SmartDashboard.putNumber("[wrist] desired angle", getDesiredAngle().getDegrees());
    }
}
