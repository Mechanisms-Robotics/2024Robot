package frc.robot.subsystems;

import com.mechlib.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gerald extends SubsystemBase {
    // intake percent speed for the intake motor to run at
    private static final double kIntakeVoltage = 5; // volts
    private static final double kOuttakeVoltage = -5; // volts
    private static final double kShooterVoltage = 5;
    private static final double kAmpVoltage = 5; // volts
    private static final double kIdleVoltage = 2;
    private static final double kShooterFeedVoltage = 3;
    private static final double kAmpFeedVoltage = 3;
    private static final double kIntakeDetectDelay = 0.0625;
    private static final double kShootDetectDelay = 0;
    private static final double kAmpDetectDelay = 0;
    private static final Timer detectDelayTimer = new Timer();

    private boolean lastDetected = false;

    // naming is based off of the unique function
    private final TalonFX intakeMotor = new TalonFX(14);
    // allows for amp and assists shooter
    private final TalonFX ampMotor = new TalonFX(15);
    // only shoots
    private final TalonFX shooterMotor = new TalonFX(16);
    private final DigitalInput noteSensor = new DigitalInput(0);

    public enum GeraldState {
        Idling,
        Intaking,
        PreparingShoot,
        Shooting,
        PreparingAmp,
        Amping
    }
    private GeraldState geraldState = GeraldState.Idling;


    public Gerald() {
        // set gerald motors to brake mode
        intakeMotor.brakeMode();
        ampMotor.brakeMode();
        shooterMotor.brakeMode();
        // set inversions (amp and shooter alternate)
        intakeMotor.setInverted(false);
        ampMotor.setInverted(true);
        shooterMotor.setInverted(false);
        // set motor voltage compensations and current limits
        intakeMotor.setVoltageCompensation(10);
        ampMotor.setVoltageCompensation(10);
        shooterMotor.setVoltageCompensation(10);
        intakeMotor.setCurrentLimit(40);
        ampMotor.setCurrentLimit(40);
        shooterMotor.setCurrentLimit(40);
    }

    /**
     * Set the intake motor speed (voltage) to kIntakeVoltage
     */
    public void intake() {
        if (!geraldState.equals(GeraldState.Intaking)) {
            intakeMotor.setVoltage(kIntakeVoltage);
            geraldState = GeraldState.Intaking;
        }
        // if the note was just detected on this cycle
        if (noteDetected() && !lastDetected) {
            detectDelayTimer.start();
            lastDetected = true;
        // if the note was not detected but was detected on the last cycle, set lastDetected to false;
        } else if (!noteDetected() && lastDetected) {
            lastDetected = false;
        }
        // if the timer has finished, idle gerald and stop the timer
        if (detectDelayTimer.hasElapsed(kIntakeDetectDelay)) {
            idle();
            detectDelayTimer.stop();
            detectDelayTimer.reset();
        }
    }

    /**
     * Sets the outtake motor speed (percent) to kOuttakeVoltage
     */
    public void outtake() {
        intakeMotor.setVoltage(kOuttakeVoltage);
    }

    /**
     * Set the amp and shooter motors to the shooter voltage
     */
    public void prepareShoot() {
        if (!geraldState.equals(GeraldState.PreparingShoot)) {
            shooterMotor.setVoltage(kShooterVoltage);
            ampMotor.setVoltage(kShooterVoltage);
            geraldState = GeraldState.PreparingShoot;
        }
    }

    /**
     * Set the shooter and amp motors to the amp voltage. The motors spin in the same direction
     */
    public void prepareAmp () {
        if (!geraldState.equals(GeraldState.PreparingAmp)) {
            // spins in the same direction as they are inverted and the amp motor is negative
            shooterMotor.setVoltage(kAmpVoltage);
            ampMotor.setVoltage(-kAmpVoltage);
            geraldState = GeraldState.PreparingAmp;
        }
    }

    /**
     * Feed the note into the shooter by setting the intake motor to the feed voltage.
     */
    public void shoot (){
        if (!geraldState.equals(GeraldState.Shooting)) {
            intakeMotor.setVoltage(kShooterFeedVoltage);
            geraldState = GeraldState.Shooting;
        }
        // if the note was just detected on this cycle
        if (!noteDetected() && lastDetected) {
            detectDelayTimer.start();
            lastDetected = false;
        // if the note was not detected but was detected on the last cycle, set lastDetected to false;
        } else if (noteDetected() && !lastDetected) {
            lastDetected = true;
        }
        // if the timer has finished, idle gerald and stop the timer
        if (detectDelayTimer.hasElapsed(kShootDetectDelay)) {
            idle();
            detectDelayTimer.stop();
            detectDelayTimer.reset();
        }
    }

    /**
     * Feed the note into the shooter by setting the intake motor to the feed voltage.
     */
    public void amp (){
        if (!geraldState.equals(GeraldState.Amping)) {
            intakeMotor.setVoltage(kAmpFeedVoltage);
            geraldState = GeraldState.Amping;
        }
        // if the note was just detected on this cycle
        if (!noteDetected() && lastDetected) {
            detectDelayTimer.start();
            lastDetected = false;
            // if the note was not detected but was detected on the last cycle, set lastDetected to false;
        } else if (noteDetected() && !lastDetected) {
            lastDetected = true;
        }
        // if the timer has finished, idle gerald and stop the timer
        if (detectDelayTimer.hasElapsed(kAmpDetectDelay)) {
            idle();
            detectDelayTimer.stop();
            detectDelayTimer.reset();
        }
    }

    public void idle() {
        if (!geraldState.equals(GeraldState.Idling)) {
            shooterMotor.setVoltage(kIdleVoltage);
            ampMotor.setVoltage(kIdleVoltage);
            intakeMotor.setVoltage(0);
            geraldState = GeraldState.Idling;
        }
    }

    public boolean noteDetected() {
        return !noteSensor.get();
    }

    @Override
    public void periodic() {
        switch (geraldState) {
            case Idling -> idle();
            case Amping -> amp();
            case Intaking -> intake();
            case Shooting -> shoot();
            case PreparingAmp -> prepareAmp();
            case PreparingShoot -> prepareShoot();
        }
    }
}
