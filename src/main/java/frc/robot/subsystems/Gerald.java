package frc.robot.subsystems;

import com.mechlib.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gerald extends SubsystemBase {
    private static final double INTAKE_SPEED = 0;
    private static final double OUTTAKE_SPEED = 0;
    private static final double SHOOTER_RPM = 0;
    private static final double FEED_SPEED = 0;
    private static final double SHOOTER_KP = 0;
    private static final double SHOOTER_KI = 0;
    private static final double SHOOTER_KD = 0;
    private static final double SHOOTER_KF = 0;
    private static final double SHOOTER_TOLERANCE = 0;

    // naming is based off of the unique function
    private final TalonFX intakeMotor = new TalonFX(10);
    // allows for amp and assists shooter
    private final TalonFX ampMotor = new TalonFX(11);
    private final TalonFX shooterMotor = new TalonFX(12);

    private double rpsToRPM(double rps) {return rps * 60;}
    public Gerald() {
        intakeMotor.coastMode();
        ampMotor.coastMode();
        shooterMotor.coastMode();

        ampMotor.setKP(SHOOTER_KP);
        ampMotor.setKI(SHOOTER_KI);
        ampMotor.setKD(SHOOTER_KD);
        ampMotor.setKF(SHOOTER_KF);

        shooterMotor.setKP(SHOOTER_KP);
        shooterMotor.setKI(SHOOTER_KI);
        shooterMotor.setKD(SHOOTER_KD);
        shooterMotor.setKF(SHOOTER_KF);

        ampMotor.setTolerance(SHOOTER_TOLERANCE);
        shooterMotor.setTolerance(SHOOTER_TOLERANCE);

        ampMotor.setVelocityUnitsFunction(this::rpsToRPM);
        shooterMotor.setVelocityUnitsFunction(this::rpsToRPM);
    }

    public void intake() {
        intakeMotor.setPercent(INTAKE_SPEED);
    }

    public void outtake() {
        intakeMotor.setPercent(OUTTAKE_SPEED);
    }

    public void spinup() {
        ampMotor.setSetpoint(SHOOTER_RPM);
        shooterMotor.setSetpoint(SHOOTER_RPM);

        ampMotor.periodicPIDF(ampMotor.getVelocity());
        shooterMotor.periodicPIDF(shooterMotor.getVelocity());
    }

    public void shoot() {
        if (!MathUtil.isNear(SHOOTER_RPM, shooterMotor.getVelocity(), SHOOTER_TOLERANCE)) {
            spinup();
            return;
        }
        intakeMotor.setPercent(FEED_SPEED);
    }

    public void stopIntake() {
        intakeMotor.setPercent(0);
    }
    public void stopShooter() {
        shooterMotor.setPercent(0);
        ampMotor.setPercent(0);
    }
}
