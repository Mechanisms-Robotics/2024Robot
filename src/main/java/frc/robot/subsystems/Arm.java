package frc.robot.subsystems;

import com.mechlib.hardware.TalonFX;
import com.mechlib.subsystems.SingleJointSubystem;
import com.mechlib.util.MechUnits;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The arm that Gerald is attached to. It is directly connected to the swerve.
 */
public class Arm extends SingleJointSubystem {
    // rotations as detected by the CanCoder at the start position if there was no offset
    private static final double kIdealStartRotation = 1.0533;
    // left arm motor magnet offset (acquired in Phoenix Tuner X)
    private static final double kLeftMagnetOffset = kIdealStartRotation - 0.448486;
    // right arm motor magnet offset
    private static final double kRightMagnetOffset = kIdealStartRotation - 0.427734;
    // right arm TalonFX motor and it's can coder
    private final TalonFX rightArmMotor = new TalonFX(13);
    // left arm TalonFX motor with its can coder
    private final TalonFX leftArmMotor = new TalonFX(12);
    // feed forward controller for the arm
    /* PID controller for the right and left arm, which will always have the same values they are different to account
       for different mechanical structures, such as belt tightening */
    private static final double kTolerance = Math.toRadians(2);
    private static final Rotation2d kStowed = Rotation2d.fromDegrees(25);
    private static final Rotation2d kIntaking = Rotation2d.fromDegrees(7);
    private static final Rotation2d kSubwooferHigh = Rotation2d.fromDegrees(94);
    private static final Rotation2d kSubwooferLow = Rotation2d.fromDegrees(15);
    private static final Rotation2d kPodiumHigh = kSubwooferHigh;
    private static final Rotation2d kPodiumLow = kSubwooferLow;
    private static final Rotation2d kAmp = Rotation2d.fromDegrees(94);
    private static final Rotation2d kPrepClimb = Rotation2d.fromDegrees(90);
    private static final Rotation2d kClimb = Rotation2d.fromDegrees(40);
    private static final double kSensorRatio = 64.0/16.0;
    private static final double kMotorRatio = 60 * kSensorRatio;
    private boolean homed = false;
    // TODO: tune the home voltage
    private static final double homeVoltage = 0.5;
    private static final Rotation2d homedPosition = Rotation2d.fromDegrees(94.77);
    private static final double kHomeTime = 1;
    private final Timer homeTimer = new Timer();
    private boolean isHoming = false;
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
        setVoltageCompensation(0.5);
        setState(SingleJointSubsystemState.CLOSED_LOOP);
        setPositionUnitsFunction((rotations) -> MechUnits.rotationsToRadians(rotations, kMotorRatio));
        setVelocityUnitsFunction((rotations) -> MechUnits.rotationsToRadians(rotations, kMotorRatio));
        setFeedforwardGains(0.0, 0, 0.0, 0.0);
        setPPIDGains(0.0, 0.0, 0.0);
//        setFeedforwardGains(0.15, 0, 0.0, 0.0);
//        setPPIDGains(1.0, 0.0, 0.0);
        setPPIDConstraints(Math.PI, 4*Math.PI);
        setTolerance(kTolerance);
        SmartDashboard.putBoolean("[Arm] disabled", disabled);
    }

    /**
     * Returns true if the arm has been homed.
     * When the arm disables, the homed is set to false.
     *
     * @return true if homed
     */
    public boolean homed() {
        return homed;
    }

    /**
     * Home the arm, zeroing the position of the arms.
     * This process moves the arm back until the arms stop moving.
     * When the arm stops moving, it sets the motor positions to the homed position.
     * This is used to un-disable the arm, because the arm needs to be homed before it can be un-disabled because
     * when it does disable it is usually because a chain slipped.
     */
    public void home() {
        if (homed) return;
        if(!isHoming) {
            setLimits(Rotation2d.fromDegrees(-1000), Rotation2d.fromDegrees(1000), kMotorRatio);
            // starts moving the arm back to the hard stop
            leftArmMotor.setVoltage(homeVoltage);
            rightArmMotor.setVoltage(homeVoltage);
            homeTimer.restart();
            isHoming = true;
            return;
        } else if (!homeTimer.hasElapsed(kHomeTime)) {
            return;
        }
        /* If the motors have stoped (velocity of the motors is within tolerance) set the motor positions to the
           position of the home position, set the voltages to 0 and homed to true */
        if (MathUtil.isNear(0, leftArmMotor.getVelocity(), 0.01)
                && MathUtil.isNear(0, rightArmMotor.getVelocity(), 0.01)) {
            leftArmMotor.setInternalSensorPosition(MechUnits.radiansToRotations(
                    homedPosition.getRadians(), kMotorRatio));
            rightArmMotor.setInternalSensorPosition(MechUnits.radiansToRotations(
                    homedPosition.getRadians(), kMotorRatio));
            leftArmMotor.setVoltage(0);
            rightArmMotor.setVoltage(0);
            homed = true;
            disabled = false;
        }
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
        System.out.println("kSubwooferHigh: " + kSubwooferHigh.getDegrees());
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
     * Set the arm to the amp position
     */
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

    /**
     * Pivots to the shuttle position (kShuttle)
     */
    public void shuttle() {
        pivotTo(kShuttle);
    }

    /**
     * Stops voltage and disables all processes on the arm (PID etc.)
     */
    public void disable() {
        stop();
        homed = false;
        disabled = true;
    }

    public void unDisable() {
        disabled = false;
    }

    /**
     * Sets the arm rotation to a given angle
     *
     * @param rotation rotation to pivot to
     */
    public void aim(Rotation2d rotation) {
        pivotTo(rotation);
    }

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
        SmartDashboard.putBoolean("[Arm] is homing", isHoming);
        // if disabled, do not run any processes on the arm
        if (Math.abs(leftArmMotor.getRawPosition() -rightArmMotor.getRawPosition()) > kAllowableDifference)
            disable();
        if (disabled || !homed) return;
        super.periodic();
    }
}