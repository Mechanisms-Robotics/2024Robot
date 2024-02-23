package frc.robot.subsystems;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.mechlib.hardware.CANCoder;
import com.mechlib.hardware.TalonFX;
import com.mechlib.util.MechUnits;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private static final boolean kOpenLoop = true;
    // left arm motor magnet offset
    private static final double kLeftMagnetOffset = 0.206299;
    // right arm motor magnet offest
    private static final double kRightMagnetOffset = -0.336914;
    // right arm TalonFX motor and it's can coder
    private final TalonFX rightArmMotor = new TalonFX(13, new CANCoder(13, kRightMagnetOffset));
    // left arm TalonFX motor with it's can coder
    private final TalonFX leftArmMotor = new TalonFX(12, new CANCoder(12, kLeftMagnetOffset));
    // feed forward controller for the arm
    private static final ArmFeedforward kArmFeedForward = new ArmFeedforward(0.2, 0, 0, 0);
    /* PID controller for the right and left arm, which will always have the same values they are different to account
       for different mechanical structures, such as belt tensioning */
    private static final ProfiledPIDController kRightPPIDController = new ProfiledPIDController(0, 0, 0,
            new TrapezoidProfile.Constraints(Math.PI / 4, 1));
    private static final ProfiledPIDController kLeftPPIDController = new ProfiledPIDController(
            kRightPPIDController.getP(), kRightPPIDController.getI(), kRightPPIDController.getD(),
            kRightPPIDController.getConstraints());
    private static final double kTolerance = Math.toRadians(5);
    private static final Rotation2d kStowed = Rotation2d.fromDegrees(20);
    private static final Rotation2d kIntaking = Rotation2d.fromDegrees(30);
    private static final Rotation2d kShooting = Rotation2d.fromDegrees(10);
    private static final double kSensorRatio = 68.0/16.0;
    private static final double kMotorRatio = 60 * kSensorRatio;
    private static final Rotation2d kForwardLimit = Rotation2d.fromDegrees(10000);
    private static final Rotation2d kReverseLimit = Rotation2d.fromDegrees(-10000);
    private static final double kLeftRightRatio = 0.8;
    private boolean disabled = false;


    public Arm() {
        configureMotors();
    }

    /**
     * Configure the motors. Set the right and left arm: to brake mode, set the units functions, and set wether they
     * are inverted or not. Set the limits of the motors.
     */
    protected void configureMotors() {
        rightArmMotor.coastMode();
        leftArmMotor.coastMode();

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

    /**
     * Set the pivot position to a rotation and apply the voltage
     *
     * @param rotation position for the arm to be pivoted to
     */
    private void pivotTo(Rotation2d rotation) {
        kRightPPIDController.setGoal(rotation.getRadians());
        kLeftPPIDController.setGoal(rotation.getRadians());
        double rightPIDFoutput = kRightPPIDController.calculate(rightArmMotor.getPosition()) + kArmFeedForward.calculate(kRightPPIDController.getSetpoint().position, kRightPPIDController.getSetpoint().velocity);
        double leftPIDFoutput = kLeftPPIDController.calculate(leftArmMotor.getPosition()) + kArmFeedForward.calculate(kLeftPPIDController.getSetpoint().position, kLeftPPIDController.getSetpoint().velocity);
        rightArmMotor.setVoltage(rightPIDFoutput);
        leftArmMotor.setVoltage(leftPIDFoutput);
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
     * Set arm to the shoot position
     */
    public void shoot() {
        pivotTo(kShooting);
    }

    /**
     * Stop all voltage on the arm
     */
    public void stop() {
        setVoltage(0);
    }

    /**
     * Stops voltage and disables all processes on the arm (PID etc)
     */
    public void disable() {
        stop();
        disabled = true;
    }

    /**
     * Sets the voltage of the right and left arm
     *
     * @param voltage voltage of the left and right motors
     */
    public void setVoltage(double voltage) {
        if (disabled) return;
        rightArmMotor.setVoltage(voltage);
        leftArmMotor.setVoltage(voltage);
    }

    /**
     * Sets the right and left arm motors sensor positions and sets the right and left are soft limits
     */
    public void setLimits() {
        rightArmMotor.setInternalSensorPosition(MechUnits.radiansToRotations(rightArmMotor.getPosition(), kMotorRatio));
        leftArmMotor.setInternalSensorPosition(MechUnits.radiansToRotations(leftArmMotor.getPosition(), kMotorRatio));
        rightArmMotor.setSoftLimits(kReverseLimit.getRotations(), kForwardLimit.getRotations());
        leftArmMotor.setSoftLimits(kReverseLimit.getRotations(), kForwardLimit.getRotations());
    }

    /**
     * Periodically output the data (right and left arm position) to SmartDashBoard. Do not run the arms if the robot
     * is disabled. Runs the PIDFs if the robot is in closed loop.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("[Right Arm] position", new Rotation2d(rightArmMotor.getPosition()).getDegrees());
        SmartDashboard.putNumber("[Left Arm] position", new Rotation2d(leftArmMotor.getPosition()).getDegrees());
        // if disabled, do not run any processes on the arm
        if (disabled) return;
        // if in closed loop, move the right and left arm via PIDF
        if (!kOpenLoop) {
            rightArmMotor.periodicPIDF(rightArmMotor.getPosition());
            leftArmMotor.periodicPIDF(leftArmMotor.getPosition());
        }
    }
}
