package frc.robot.subsystems;

import com.mechlib.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gerald extends SubsystemBase {
    // intake percent speed for the intake motor to run at
    private static final double kIntakeSpeed = 0.4; // percent
    private static final double kOuttakeSpeed = -0.35; // percent
    private static final double kShooterRPM = 6000; // rpm
    private static final double kShooterSpeed = 1;
    private static final double kFeedSpeed = 0.3; // percent
    // PIDF values for the shooter
    private static final double kShooterKP = 0.01;
    private static final double SHOOTER_KI = 0;
    private static final double SHOOTER_KD = 0;
    private static final double SHOOTER_KF = 0.01;
    private static final double SHOOTER_TOLERANCE = 100; // rpm

    // naming is based off of the unique function
    private final TalonFX intakeMotor = new TalonFX(14);
    // allows for amp and assists shooter
    private final TalonFX ampMotor = new TalonFX(15);
    // only shoots
    private final TalonFX shooterMotor = new TalonFX(16);
    private static final double kPULLEY_RATIO = 1;

    /**
     * Converts rotations per second to rotations per minute
     * @param rps rps to be converted to rpm
     * @return rpm
     */
    private double rpsToRPM(double rps) {return (rps / kPULLEY_RATIO ) * 60;}
    public Gerald() {
        intakeMotor.brakeMode();
        ampMotor.brakeMode();
        shooterMotor.brakeMode();

        ampMotor.setKP(kShooterKP);
        ampMotor.setKI(SHOOTER_KI);
        ampMotor.setKD(SHOOTER_KD);

        shooterMotor.setKP(kShooterKP);
        shooterMotor.setKI(SHOOTER_KI);
        shooterMotor.setKD(SHOOTER_KD);

        ampMotor.setTolerance(SHOOTER_TOLERANCE);
        shooterMotor.setTolerance(SHOOTER_TOLERANCE);

        ampMotor.setVelocityUnitsFunction(this::rpsToRPM);
        shooterMotor.setVelocityUnitsFunction(this::rpsToRPM);

        ampMotor.setVoltageCompensation(10);
        shooterMotor.setVoltageCompensation(10);
    }

    /**
     * Set the intake motor speed (percent) to kIntakeSpeed
     */
    public void intake() {
        intakeMotor.setPercent(kIntakeSpeed);
    }

    /**
     * Sets the outtake motor speed (percent) to kOuttakeSpeed
     */
    public void outtake() {
        intakeMotor.setPercent(kOuttakeSpeed);
    }

    /**
     * Spins up the shooter and amp motor for shooting
     */
    public void spinup() {
        SmartDashboard.putBoolean("Spinup", true);
//        ampMotor.setSetpoint(-kShooterRPM);
//        shooterMotor.setSetpoint(kShooterRPM);
//        SmartDashboard.putNumber("[Shooter Motor] Set Point", shooterMotor.getSetpoint());
        ampMotor.setPercent(-kShooterSpeed);
        shooterMotor.setPercent(kShooterSpeed);
//
//        ampMotor.periodicPIDF(ampMotor.getVelocity());
//        shooterMotor.periodicPIDF(shooterMotor.getVelocity());
    }

    /**
     * If the shooter speed is not the target speed, spin. When it is up to speed, move the note
     * with the intakeMotor by setting the speed to the kFeedSpeed percent
     */
    public void shoot() {
        if (!MathUtil.isNear(kShooterRPM, shooterMotor.getVelocity(), SHOOTER_TOLERANCE)) {
            spinup();
            return;
        }
        SmartDashboard.putBoolean("Spinup", false);
        intakeMotor.setPercent(kFeedSpeed);
    }

    /**
     * Sets the intakeMotor to 0 percent
     */
    public void stopIntake() {
        intakeMotor.setPercent(0);
    }

    /**
     * Sets the motors for shooting (shooterMotor and ampMotor) to 0 percent
     */
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
