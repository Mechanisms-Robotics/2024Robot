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
    // true if the arm runs in open loop, false if it runs in closed loop
    private static final boolean kOpenLoop = true;
    // magnet offset (acquired in Phoenix Tuner X)
    private static final double kMagnetOffset = 0;
    // right arm TalonFX motor (with the Can Encoder)
    private final TalonFX wristMotor = new TalonFX(17, new CANCoder(17, kMagnetOffset));
    // feed forward controller for the wrist
    private static final ArmFeedforward kArmFeedForward = new ArmFeedforward(0.2, 0, 0, 0);
    // PPID controller for the wrist
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

        // sets the wrist motors position and velocity units function, accounting for sensor ratio
        wristMotor.setPositionUnitsFunction(
                (Double rotations) -> MechUnits.rotationsToRadians(rotations, kSensorRatio));
        wristMotor.setVelocityUnitsFunction(
                (Double rotations) -> MechUnits.rotationsToRadians(rotations, kSensorRatio));

        // inversion of the sensor
        wristMotor.setSensorInverted(true);
        wristMotor.setInverted(true);

        setLimits();
    }

    /**
     * Set the pivot position to a rotation and applies the given voltage
     *
     * @param rotation position for the wrist to be pivoted to
     */
    private void pivotTo(Rotation2d rotation) {
        kPPIDController.setGoal(rotation.getRadians());
        // set and apply the wrist voltage using the PPID controller and the feed forward controller
        double PPIDFOutput = kPPIDController.calculate(wristMotor.getRelativePosition())
                + kArmFeedForward.calculate(kPPIDController.getSetpoint().position,
                kPPIDController.getSetpoint().velocity);
        wristMotor.setVoltage(PPIDFOutput);
    }

    /**
     * Set the wrist to stow position
     */
    public void stow() {
        pivotTo(kStowed);
    }

    /**
     * Set the wrist to intake position
     */
    public void intake() {
        pivotTo(kIntaking);
    }

    /**
     * Set the wrist to shoot position
     */
    public void shoot() {
        pivotTo(kShooting);
    }

    /**
     * Sets the voltage of the wrist motor
     *
     * @param voltage voltage wrist motor
     */
    public void setVoltage(double voltage) {
        wristMotor.setVoltage(voltage);
    }

    /**
     * Set the wrist motor sensor position and the soft limit
     */
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
