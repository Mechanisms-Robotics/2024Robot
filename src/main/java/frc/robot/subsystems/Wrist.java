package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.mechlib.hardware.BrushlessMotorController;
import com.mechlib.hardware.TalonFX;
import com.mechlib.subsystems.SingleJointSubystem;
import com.mechlib.util.MechUnits;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.Supplier;

/**
 * Controls the motor that pivots Gerald
 */
public class Wrist extends SingleJointSubystem {
    private static final double kSensorRatio = 38.0/16.0;
    private static final double kMotorRatio = 125.0 * kSensorRatio;
    private static final double kStartRotations = 0.29027778;
    // wrist magnet offset
    private final TalonFX wristMotor = new TalonFX(17);
    private final Pigeon2 gyro = new Pigeon2(18);
    private Translation2d gravityVector = new Translation2d();
    // TODO: find the gravity offset of the gyro
    private static final Rotation2d gravityOffset = Rotation2d.fromDegrees(0); // 175
    private Rotation2d bootGravityAngle = new Rotation2d();
    private final Supplier<Double> swervePitch;
    private final Supplier<Double> swerveRoll;
    private static final double kAllowableTip = 5;
    private static final double kTolerance = Math.toRadians(0.5);
    private static final Rotation2d kStowed = Rotation2d.fromDegrees(90);
    private static final Rotation2d kIntaking = Rotation2d.fromDegrees(87.5);
    private static final Rotation2d kSubwooferHigh = Rotation2d.fromDegrees(92.5);
    private static final Rotation2d kSubwooferLow = Rotation2d.fromDegrees(97.5);
    private static final Rotation2d kPodiumHigh = Rotation2d.fromDegrees(120);
    private static final Rotation2d kPodiumLow = kPodiumHigh;
    private static final Rotation2d kAmp = Rotation2d.fromDegrees(90);
    private static final Rotation2d kPrepClimb = Rotation2d.fromDegrees(110);
    private static final Rotation2d kClimb = Rotation2d.fromDegrees(110);
    private static final Rotation2d kShuttle = Rotation2d.fromDegrees(140);
    private static final Rotation2d kForwardLimit = Rotation2d.fromDegrees(140);
    private static final Rotation2d kReverseLimit = Rotation2d.fromDegrees(80);
    private boolean disabled = false;
    private double wristAdjustment = 0;
    /** Shot adjustment for the wrist, used in SmartDashboard */
    private final SendableChooser<Double> adjustmentAmount = new SendableChooser<>();


    public Wrist(Supplier<Double> swervePitch, Supplier<Double> swerveRoll) {
        this.swervePitch = swervePitch;
        this.swerveRoll = swerveRoll;
        
        addMotor(wristMotor, true);
        setCurrentLimit(40.0);
        setVoltageCompensation(10.0);
        setState(SingleJointSubsystemState.CLOSED_LOOP);
        setPositionUnitsFunction((rotations) -> MechUnits.rotationsToRadians(rotations, kMotorRatio));
        setVelocityUnitsFunction((rotations) -> MechUnits.rotationsToRadians(rotations, kMotorRatio));
//        setLimits(kReverseLimit, kForwardLimit, kMotorRatio);
//        setFeedforwardGains(0.15, 0, 0.0, 0.0);
//        setPPIDGains(0.6, 0.0, 0.0);
        setFeedforwardGains(0.15, 0, 0.0, 0.0);
        setPPIDGains(0.6, 0.0, 0.0);
        setPPIDConstraints(Math.PI/4, Math.PI/2);
        setTolerance(kTolerance);
        pivotTo(Rotation2d.fromDegrees(90));

        adjustmentAmount.addOption("^^^", -7.5);
        adjustmentAmount.addOption("^^", -2.5);
        adjustmentAmount.addOption("^", -1.);
        adjustmentAmount.setDefaultOption("None", 0.);
        adjustmentAmount.addOption("v", 1.);
        adjustmentAmount.addOption("vv", 2.5);
        adjustmentAmount.addOption("vvv", 7.5);
        SmartDashboard.putData("Shot Adjustment", adjustmentAmount);
        gravityVector = new Translation2d(gyro.getGravityVectorX().getValueAsDouble(),
                gyro.getGravityVectorZ().getValueAsDouble());
        bootGravityAngle = new Rotation2d(Math.atan2(gyro.getGravityVectorZ().getValueAsDouble(),
                                 gyro.getGravityVectorX().getValueAsDouble()));
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

    /**
     * Pivots the wrist to the prepare climb position.
     * This position puts the hooks right above the chain so when the arm and wrist go down, it pulls against the chain.
     */
    public void prepClimb() {
        pivotTo(kPrepClimb);
    }

    /**
     * Pivots the wrist down to the climb position, pulling on the chain to lift the robot
     */
    public void climb() {
        pivotTo(kClimb);
    }

    /** Pivot to the shuttle position (kShuttle) */
    public void shuttle() {
        pivotTo(kShuttle);
    }

    /**
     * Pivot to the given rotation
     *
     * @param rotation angle fo the Wrist
     */
    public void aim(Rotation2d rotation) {
        pivotTo(rotation.plus(Rotation2d.fromDegrees(wristAdjustment)));
    }

    /**
     * Returns the angle of the arm.
     * Overrides the angle from CANCoder based → gyro based.
     *
     * @return angle of the wrist
     */
    @Override
    protected Rotation2d getAngle() {
        // TODO: figure out if the gyro should be inverted or not
        return Rotation2d.fromDegrees(gyro.getPitch().getValueAsDouble()).unaryMinus().rotateBy(bootGravityAngle);
    }

    /**
     * Periodically output the data (right and left arm position) to SmartDashBoard. Do not run the arms if the robot
     * is disabled. Runs the PIDFs if the robot is in closed loop.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("[Wrist] position", wristMotor.getRawPosition());
        SmartDashboard.putNumber("[Wrist] current angle", getAngle().getDegrees());
        SmartDashboard.putNumber("[Wrist] desired angle", getDesiredAngle().getDegrees());
        SmartDashboard.putNumber("[Wrist] gravity angle", bootGravityAngle.getDegrees());
        SmartDashboard.putNumber("[Wrist] pitch", gyro.getPitch().getValueAsDouble());
        SmartDashboard.putNumber("[Wrist] roll", gyro.getRoll().getValueAsDouble());
        SmartDashboard.putNumber("[Wrist] x gravity component", gravityVector.getX());
        SmartDashboard.putNumber("[Wrist] y gravity component", gravityVector.getY());

        if (getAngle().getDegrees() < 0) disabled = true; // if the angle of the arm is negative, disable it
        if (Math.abs(swerveRoll.get()) > kAllowableTip || Math.abs(swervePitch.get()) > kAllowableTip) {
            wristMotor.stop();
            return;
        }
        if (wristMotor.getSetpoint() > kForwardLimit.getRadians() || wristMotor.getSetpoint() < kReverseLimit.getRadians()) {
            wristMotor.stop();
            return;
        }
        if (disabled) return;
        // Check if current state is closed loop
        if (state == SingleJointSubsystemState.CLOSED_LOOP) {
            // Loop over every motor
            for (BrushlessMotorController motor : motors) {
                // Run periodic PIDF code
                motor.periodicPPIDF(
                        getAngle().getRadians(),
                        motor.getVelocity(),
                        feedforwardController.calculate(
                                motor.getSetpoint(),
                                motor.getVelocitySetpoint()
                        )
                );
            }
        }
        if (adjustmentAmount.getSelected() != null) {
            wristAdjustment = adjustmentAmount.getSelected();
        } else {
            System.out.println("adjustmentAmount.getSelected is null");
        }
    }
}
