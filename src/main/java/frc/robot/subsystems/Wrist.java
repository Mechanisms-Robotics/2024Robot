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
    // true if the arm runs in open loop, false if it runs in closed loop
    // left arm motor magnet offset (acquired in Phoenix Tuner X)
    private static final double kMagnetOffset = -0.0419;
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
     * Periodically output the data (right and left arm position) to SmartDashBoard. Do not run the arms if the robot
     * is disabled. Runs the PIDFs if the robot is in closed loop.
     */
    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("[wrist] Wrist position", WristMotor.getRawPosition());
        SmartDashboard.putNumber("[wrist] current angle", getAngle().getDegrees());
        SmartDashboard.putNumber("[wrist] desired angle", getDesiredAngle().getDegrees());
    }
}
