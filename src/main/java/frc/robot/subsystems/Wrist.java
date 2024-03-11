package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.mechlib.hardware.BrushlessMotorController;
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
    private static final double kMotorRatio = 125.0 * kSensorRatio;
    private static final double kStartRotations = 0.29027778;
    // wrist magnet offset
    private final TalonFX WristMotor = new TalonFX(17);
    private final Pigeon2 gyro = new Pigeon2(18);
    private static final double kTolerance = Math.toRadians(0.5);
    private static final Rotation2d kStowed = Rotation2d.fromDegrees(90);
    private static final Rotation2d kIntaking = Rotation2d.fromDegrees(87.5);
    private static final Rotation2d kSubwooferHigh = Rotation2d.fromDegrees(90);
    private static final Rotation2d kSubwooferLow = Rotation2d.fromDegrees(95);
    private static final Rotation2d kPodiumHigh = Rotation2d.fromDegrees(115);
    private static final Rotation2d kPodiumLow = kPodiumHigh;
    private static final Rotation2d kAmp = Rotation2d.fromDegrees(90);
    private static final Rotation2d kPrepClimb = Rotation2d.fromDegrees(110);
    private static final Rotation2d kClimb = Rotation2d.fromDegrees(110);
    private static final Rotation2d kShuttle = Rotation2d.fromDegrees(130);
    private static final Rotation2d kForwardLimit = Rotation2d.fromDegrees(130);
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
        setPPIDGains(0.6, 0.0, 0.0);
        setPPIDConstraints(Math.PI/4, Math.PI/2);
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

    public void prepClimb() {
        pivotTo(kPrepClimb);
    }

    public void climb() {
        pivotTo(kClimb);
    }

    /**
     * Pivot to the shuttle position (kShuttle)
     *
     */
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
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
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
        wristAdjustment = adjustmentAmount.getSelected();
    }
}
