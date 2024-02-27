package frc.robot.subsystems;

import com.mechlib.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.PrepareShoot;
import frc.util6328.Alert;
import frc.util6328.Alert.AlertType;

/**
 * The box of wheels that intakes, shoots, and amps the notes.
 */
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
    private final Alert unexpectedNote =
            new Alert("an unexpected note was detected in gerald after feeding it to the shooter",
                    AlertType.ERROR);
    private boolean lastDetected = false;

    // naming is based off of the unique function
    private final TalonFX intakeMotor = new TalonFX(14);
    // allows for amp and assists shooter
    private final TalonFX ampMotor = new TalonFX(15);
    // only shoots
    private final TalonFX shooterMotor = new TalonFX(16);

    private final DigitalInput noteSensor = new DigitalInput(0);

    public enum State {
        Idling,
        Feeding,
        Intaking,
        PreparingAmp,
        PreparingShoot
    }

    private State state = State.Idling;


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
        if (!state.equals(State.Intaking)) {
            intakeMotor.setVoltage(kIntakeVoltage);
            state = State.Intaking;
        }
        // if the note was just detected on this cycle
        if (noteDetected() && !lastDetected) {
            detectDelayTimer.restart();
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

    public void toggleIntake() {
        if (!state.equals(State.Intaking)) intake();
        else idle();
    }

    /**
     * Sets the outtake motor speed (voltage) to kOuttakeVoltage
     */
    public void outtake() {
        intakeMotor.setVoltage(kOuttakeVoltage);
    }

    /**
     * Set the amp and shooter motors to the shooter voltage
     */
    public void prepareShoot() {
        if (!state.equals(State.PreparingShoot)) {
            shooterMotor.setVoltage(kShooterVoltage);
            ampMotor.setVoltage(kShooterVoltage);
            state = State.PreparingShoot;
        }
    }

    /**
     * Set the shooter and amp motors to the amp voltage. The motors spin in the same direction
     */
    public void prepareAmp () {
        if (!state.equals(State.PreparingAmp)) {
            // spins in the same direction as they are inverted and the amp motor is negative
            shooterMotor.setVoltage(kAmpVoltage);
            ampMotor.setVoltage(-kAmpVoltage);
            state = State.PreparingAmp;
        }
    }

    public void toggleSpinupAmp() {
        if (state.equals(State.PreparingAmp)) idle();
        else prepareAmp();
    }

    public void toggleSpinupShoot() {
        if (state.equals(State.PreparingShoot)) idle();
        else prepareShoot();
    }

    /**
     * Feed the note into the shooter by setting the intake motor to the feed voltage.
     */
    public void feed (){
        /* if the state is not already in amping, set the state to amping and the voltage to kAmpFeedVoltage, which
           will feed the note into the shooter */
        if (!state.equals(State.Feeding)) {
            intakeMotor.setVoltage(kAmpFeedVoltage);
            state = State.Feeding;
        }
        /* if the note was not detected on the cycle (if all goes well mechanically, it just got shot out of gerald)
           set the timer for to idle the shooter */
        if (!noteDetected() && lastDetected) {
            detectDelayTimer.restart(); // restart instead of start in the case that a timer was already running
            lastDetected = false;
        /* if the note just detected on this cycle (which should not happen because the note shoot not come back into
           gerald after amping) set last detected to true */
        } else if (noteDetected() && !lastDetected) {
            lastDetected = true;
            unexpectedNote.set(true);
        }
        // if the timer has finished, idle gerald and stop the timer
        if (detectDelayTimer.hasElapsed(kAmpDetectDelay)) {
            idle();
            detectDelayTimer.stop();
            detectDelayTimer.reset();
        }
    }

    /**
     * Set the shooter and amp voltage to kIdleVoltage and stop the intakeMotor.
     * The shooter and amp motor idle voltage should be as high as possible without overloading the battery or
     * making the arm shake to much.
     */
    public void idle() {
        if (!state.equals(State.Idling)) {
            shooterMotor.setVoltage(kIdleVoltage);
            ampMotor.setVoltage(kIdleVoltage);
            intakeMotor.setVoltage(0);
            state = State.Idling;
        }
    }

    /**
     * Returns a boolean of whether the note sensor detected anything
     *
     * @return if the note sensor detected anything
     */
    public boolean noteDetected() {
        return !noteSensor.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Note Sensor", noteDetected()); // show on advantage scope
        switch (state) {
            case Idling -> idle();
            case Feeding -> feed();
            case Intaking -> intake();
            case PreparingAmp -> prepareAmp();
            case PreparingShoot -> prepareShoot();
        }
    }
}
