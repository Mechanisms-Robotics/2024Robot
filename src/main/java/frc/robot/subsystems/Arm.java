package frc.robot.subsystems;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.mechlib.hardware.CANCoder;
import com.mechlib.hardware.TalonFX;
import com.mechlib.subsystems.SingleJointSubystem;
import com.mechlib.util.MechUnits;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SingleJointSubystem {
    // true if the arm runs in open loop, false if it runs in closed loop
    private static final boolean kOpenLoop = true;
    // left arm motor magnet offset (acquired in Phoenix Tuner X)
    private static final double kLeftMagnetOffset = 0.2326;
    // right arm motor magnet offset
    private static final double kRightMagnetOffset = 0.675;
    // right arm TalonFX motor and it's can coder
    private final TalonFX rightArmMotor = new TalonFX(13, new CANCoder(13, kRightMagnetOffset, AbsoluteSensorRangeValue.Unsigned_0To1, SensorDirectionValue.Clockwise_Positive));
    // left arm TalonFX motor with it's can coder
    private final TalonFX leftArmMotor = new TalonFX(12, new CANCoder(12, kLeftMagnetOffset, AbsoluteSensorRangeValue.Unsigned_0To1, SensorDirectionValue.CounterClockwise_Positive));
    // feed forward controller for the arm
    /* PID controller for the right and left arm, which will always have the same values they are different to account
       for different mechanical structures, such as belt tensioning */
    private static final double kTolerance = Math.toRadians(0.5);
    private static final Rotation2d kStowed = Rotation2d.fromDegrees(60);
    private static final Rotation2d kIntaking = Rotation2d.fromDegrees(3);
    private static final Rotation2d kShooting = Rotation2d.fromDegrees(95);
    private static final double kSensorRatio = 68.0/16.0;
    private static final double kMotorRatio = 60 * kSensorRatio;
    private static final Rotation2d kForwardLimit = Rotation2d.fromDegrees(95);
    private static final Rotation2d kReverseLimit = Rotation2d.fromDegrees(3);
    private static final double kAllowableDifference = 5.0;
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
        setPPIDConstraints(2*Math.PI, 8*Math.PI);
        setTolerance(kTolerance);
        SmartDashboard.putBoolean("[Arm] disabled", disabled);
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
     * Set arm to the shoot/amp position
     */
    public void shoot() {
        pivotTo(kShooting);
    }

    /**
     * Stops voltage and disables all processes on the arm (PID etc)
     */
    public void disable() {
        stop();
        disabled = true;
        SmartDashboard.putBoolean("[Arm] disabled", disabled);
    }

    /**
     * Periodically output the data (right and left arm position) to SmartDashBoard. Do not run the arms if the robot
     * is disabled. Runs the PIDFs if the robot is in closed loop.
     */
    @Override
    public void periodic() {
        // if disabled, do not run any processes on the arm
        if (Math.abs(leftArmMotor.getRawPosition() -rightArmMotor.getRawPosition()) > kAllowableDifference){
          disable();
         }
        if (disabled) return;
        super.periodic();
        SmartDashboard.putNumber("[Arm] Left position", leftArmMotor.getRawPosition());
        SmartDashboard.putNumber("[Arm] Right position", rightArmMotor.getRawPosition());
        SmartDashboard.putNumber("[Arm] current angle", getAngle().getDegrees());
        SmartDashboard.putNumber("[Arm] desired angle", getDesiredAngle().getDegrees());
    }
}
