package frc.robot.subsystems;

import com.mechlib.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gerald extends SubsystemBase {
    // intake percent speed for the intake motor to run at
    private static final double INTAKE_SPEED = 0.4; // percent
    private static final double OUTTAKE_SPEED = -0.35; // percent
    private static final double SHOOTER_RPM = 6000; // rpm
    private static final double SHOOTER_SPEED = 1;
    private static final double FEED_SPEED = 0.3; // percent
    // PID values for the shooter
    private static final double SHOOTER_KP = 0.01;
    private static final double SHOOTER_KI = 0;
    private static final double SHOOTER_KD = 0;
    private static final double SHOOTER_KF = 0.01;
    private static final double SHOOTER_TOLERANCE = 100;

    // naming is based off of the unique function
    private final TalonFX intakeMotor = new TalonFX(14);
    // allows for amp and assists shooter
    private final TalonFX ampMotor = new TalonFX(15);
    private final TalonFX shooterMotor = new TalonFX(16);
    private static final double kPULLEY_RATIO = 1;

    private double rpsToRPM(double rps) {return (rps / kPULLEY_RATIO ) * 60;}
    public Gerald() {
        intakeMotor.brakeMode();
        ampMotor.brakeMode();
        shooterMotor.brakeMode();

        ampMotor.setKP(SHOOTER_KP);
        ampMotor.setKI(SHOOTER_KI);
        ampMotor.setKD(SHOOTER_KD);

        shooterMotor.setKP(SHOOTER_KP);
        shooterMotor.setKI(SHOOTER_KI);
        shooterMotor.setKD(SHOOTER_KD);

        ampMotor.setTolerance(SHOOTER_TOLERANCE);
        shooterMotor.setTolerance(SHOOTER_TOLERANCE);

        ampMotor.setVelocityUnitsFunction(this::rpsToRPM);
        shooterMotor.setVelocityUnitsFunction(this::rpsToRPM);

        ampMotor.setVoltageCompensation(10);
        shooterMotor.setVoltageCompensation(10);
    }

    public void intake() {
        intakeMotor.setPercent(INTAKE_SPEED);
    }

    public void outtake() {
        intakeMotor.setPercent(OUTTAKE_SPEED);
    }

    public void spinup() {
        SmartDashboard.putBoolean("Spinup", true);
//        ampMotor.setSetpoint(-SHOOTER_RPM);
//        shooterMotor.setSetpoint(SHOOTER_RPM);
//        SmartDashboard.putNumber("[Shooter Motor] Set Point", shooterMotor.getSetpoint());
        ampMotor.setPercent(-SHOOTER_SPEED);
        shooterMotor.setPercent(SHOOTER_SPEED);
//
//        ampMotor.periodicPIDF(ampMotor.getVelocity());
//        shooterMotor.periodicPIDF(shooterMotor.getVelocity());
    }

    public void shoot() {
        if (!MathUtil.isNear(SHOOTER_RPM, shooterMotor.getVelocity(), SHOOTER_TOLERANCE)) {
            spinup();
            return;
        }
        SmartDashboard.putBoolean("Spinup", false);
        intakeMotor.setPercent(FEED_SPEED);
    }

    public void stopIntake() {
        intakeMotor.setPercent(0);
    }
    public void stopShooter() {
        shooterMotor.setPercent(0);
        ampMotor.setPercent(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("[Intake Motor] Velocity", intakeMotor.getVelocity());
        SmartDashboard.putNumber("[Amp Motor] Velocity", ampMotor.getVelocity());
        SmartDashboard.putNumber("[Shooter Motor] Velocity", shooterMotor.getVelocity());
    }
}
