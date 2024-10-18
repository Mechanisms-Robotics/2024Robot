package frc.robot.subsystems;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.mechlib.hardware.CANCoder;
import com.mechlib.hardware.TalonFX;
import com.mechlib.subsystems.SingleJointSubystem;
import com.mechlib.util.MechUnits;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The arm that Gerald is attached to. It is directly connected to the swerve.
 * Controls the motor for the left and right arm.
 */
public class Arm extends SingleJointSubystem {
    // rotations as detected by the CanCoder at the start position if there was no offset
    private static final double kIdealStartRotation = 1;
    // left arm motor magnet offset (acquired in Phoenix Tuner X)
    private static final double kLeftMagnetOffset = kIdealStartRotation - 0.489258;
    // right arm motor magnet offset
    private static final double kRightMagnetOffset = kIdealStartRotation - 0.496338;
    // right arm TalonFX motor and it's can coder
    private final TalonFX rightArmMotor = new TalonFX(13, new CANCoder(13, kRightMagnetOffset, AbsoluteSensorRangeValue.Unsigned_0To1, SensorDirectionValue.Clockwise_Positive));
    // left arm TalonFX motor with its can coder
    private final TalonFX leftArmMotor = new TalonFX(12, new CANCoder(12, kLeftMagnetOffset, AbsoluteSensorRangeValue.Unsigned_0To1, SensorDirectionValue.CounterClockwise_Positive));
    private static final double kTolerance = Math.toRadians(2);
    private static final Rotation2d kStowed = Rotation2d.fromDegrees(30);
    /** Angle of the intake, as low as possible without touching the floor */
    private static final Rotation2d kIntaking = Rotation2d.fromDegrees(7);
    private static final Rotation2d kSubwooferHigh = Rotation2d.fromDegrees(94);
    private static final Rotation2d kSubwooferLow = Rotation2d.fromDegrees(15);
    private static final Rotation2d kPodiumHigh = kSubwooferHigh;
    private static final Rotation2d kPodiumLow = kSubwooferLow;
    private static final Rotation2d kAmp = Rotation2d.fromDegrees(94);
    /** Angle of the arm before it pulls down and climbs, should be high enough for the hooks to go above the chain */
    private static final Rotation2d kPrepClimb = Rotation2d.fromDegrees(90);
    /** Angle of the arm after it pulls down on the chain */
    private static final Rotation2d kClimb = Rotation2d.fromDegrees(40);
    /** Ratio of the can coder sensor to the arm shaft */
    private static final double kSensorRatio = 64.0/16.0;
    /** Ratio of the motor to the arm shaft */
    private static final double kMotorRatio = 60 * kSensorRatio;
    private static final Rotation2d kShuttle = Rotation2d.fromDegrees(60);
    private static final Rotation2d kForwardLimit = Rotation2d.fromDegrees(94);
    private static final Rotation2d kReverseLimit = Rotation2d.fromDegrees(2);
    private static final double kAllowableDifference = 7.5;
    // safety disable feature, triggered by the secondary driver when x is pressed
    private boolean disabled = false;

    public Arm() {
        addMotor(leftArmMotor, true);
        addMotor(rightArmMotor, false);
        setCurrentLimit(40.0);
        setVoltageCompensation(10.0);
        setState(SingleJointSubsystemState.CLOSED_LOOP);
        setPositionUnitsFunction((rotations) -> MechUnits.rotationsToRadians(rotations, kSensorRatio));
        setVelocityUnitsFunction((rotations) -> MechUnits.rotationsToRadians(rotations, kSensorRatio));
        setLimits(kReverseLimit, kForwardLimit, kMotorRatio);
        setFeedforwardGains(0.15, 0, 0.0, 0.0);
        setPPIDGains(1.0, 0.0, 0.0);
        setPPIDConstraints(Math.PI, 4*Math.PI);
        setTolerance(kTolerance);
        SmartDashboard.putBoolean("[Arm] disabled", disabled);
    }

    /** Set arm to the stow position */
    public void stow() {
        pivotTo(kStowed);
    }

    /** Set arm to the intake position */
    public void intake() {
        pivotTo(kIntaking);
    }

    /** Set arm to the shoot high subwoofer position */
    public void shootHighSubwoofer() {
        System.out.println("kSubwooferHigh: " + kSubwooferHigh.getDegrees());
        pivotTo(kSubwooferHigh);
    }

    /** Set arm to the shoot low subwoofer position */
    public void shootLowSubwoofer() {
        pivotTo(kSubwooferLow);
    }

    /** Set arm to the shoot high podium position */
    public void shootHighPodium() {
        pivotTo(kPodiumHigh);
    }

    /** Set arm to the shoot low podium position */
    public void shootLowPodium() {
        pivotTo(kPodiumLow);
    }

    /** Set the arm to the amp position */
    public void amp() {
        pivotTo(kAmp);
    }

    /**
     * Set the arm to the preparing climb position.
     * Preparing the climb brings the Arm to the position so that Gerald can go over the chain.
     */
    public void prepClimb() {
        pivotTo(kPrepClimb);
    }

    /**
     * Set the arm to the climb position.
     * This is when the arm folds down, bringing the robot up
     */
    public void climb() {
        pivotTo(kClimb);
    }

    /** Pivots to the shuttle position */
    public void shuttle() {
        pivotTo(kShuttle);
    }

    /** Stops voltage and disables all processes on the arm (PID etc.) */
    public void disable() {
        stop();
        disabled = true;
    }

    /** Un-disable the arm */
    public void unDisable() {
        disabled = false;
    }

    /** Sets the arm rotation to a given angle */
    public void aim(Rotation2d rotation) {
        pivotTo(rotation);
    }

    /** @return true if the arm is aimed at the desired angle */
    public boolean aimed() {
        return MathUtil.isNear(getDesiredAngle().getDegrees(), getAngle().getDegrees(), kTolerance + 2);
    }

    /**
     * Periodically output the data (right and left arm position) to SmartDashBoard. Do not run the arms if the robot
     * is disabled. Runs the PIDFs if the robot is in closed loop cuh.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("[Arm] Left position", leftArmMotor.getRawPosition());
        SmartDashboard.putNumber("[Arm] Right position", rightArmMotor.getRawPosition());
        SmartDashboard.putNumber("[Arm] current angle", getAngle().getDegrees());
        SmartDashboard.putNumber("[Arm] desired angle", getDesiredAngle().getDegrees());
        SmartDashboard.putBoolean("[Arm] aimed", aimed());
        SmartDashboard.putBoolean("[Arm] disabled", disabled);
        // if disabled, do not run any processes on the arm
        if (Math.abs(leftArmMotor.getRawPosition() -rightArmMotor.getRawPosition()) > kAllowableDifference)
            disable();
        if (disabled) return;
        super.periodic();
    }
}