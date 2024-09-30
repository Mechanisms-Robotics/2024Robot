package frc.robot.subsystems;

import com.mechlib.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util6328.Alert;
import frc.util6328.Alert.AlertType;

/** The box of wheels that intakes, shoots, and amps the notes. */
public class Gerald extends SubsystemBase {
    private static final double kIntakeVoltage = 10; // volts
    private static final double kOuttakeVoltage = -12; // volts
    private static final double kShooterVoltage = 10; // volts
    private static final double kAmpVoltage = 8; // volts
    private static final double kIdleVoltage = 4; // volts
    private static final double kAmpFeedVoltage = 10; // volts
    private static final double kIntakeDetectDelay = 0.001; // seconds
    private static final double kSpinupRPM = 4000; // RPM
    private static final double kAmpSpinupRPM = 3450; // RPM
    private static final double kFeedDetectDelay = 1; // seconds
    private static final Timer detectDelayTimer = new Timer();
    private final DigitalInput noteSensor = new DigitalInput(8);
    private final DigitalInput noteSensor2 = new DigitalInput(9);
    private final Alert unexpectedNote =
            new Alert("an unexpected note was detected in gerald after feeding it to the shooter",
                    AlertType.ERROR);
    private boolean lastDetected = false;

    // naming is based off of the unique function
    /** Controls the intake wheels */
    private final TalonFX intakeMotor = new TalonFX(14);
    /**
     * Motor that controls the row of colsons on the bottom of the shooter (facing the limelight).
     * Helps shoot and allows for amp.
     */
    private final TalonFX ampMotor = new TalonFX(15);
    /**
     * Motor that controls the row of colsons on the top of the shooter (facing the limelight).
     * Used only for shooting.
     */
    private final TalonFX shooterMotor = new TalonFX(16);

    public enum State {
        /** Puts the intake motor in brake mode and spins the shooter and amp motors */
        Idling,
        /** Continues the intake motor to feed the note into the shooter */
        Feeding,
        /** Spins the intake motor to pick up a note */
        Intaking,
        /** Spins up the shooter and amp motor in the same direction for amping the note */
        PreparingAmp,
        /** Spins up the shooter and amp motor in opposite directions for shooting the note */
        PreparingShoot,
        /** Spins the intake in reverse to outtake the note */
        Outtaking
    }

    private State state = State.Idling;

    /** @return state of gerald */
    public State getState() {
        return state;
    }

    public Gerald() {
        // set gerald motors to brake mode
        intakeMotor.brakeMode();
        ampMotor.brakeMode();
        shooterMotor.brakeMode();
        // set motor voltage compensations and current limits
        intakeMotor.setVoltageCompensation(10);
        ampMotor.setVoltageCompensation(10);
        shooterMotor.setVoltageCompensation(10);
        intakeMotor.setCurrentLimit(40);
        ampMotor.setCurrentLimit(40);
        shooterMotor.setCurrentLimit(40);
        shooterMotor.setVelocityUnitsFunction((Double rps) -> rps * 60);
        ampMotor.setVelocityUnitsFunction((Double rps) -> rps * 60);
    }

    /**
     * Intakes the note.
     * Stops the note as soon as the note is detected to it does not feed into the shooter.
     */
    public void intake() {
        if (state != State.Intaking) {
            // increased the intake voltage in autonomous because it was intaking better in teleop
            // intake voltage for autonomous was lowered because the note was shooting out at gwinnett
            if (DriverStation.isAutonomous()) intakeMotor.setVoltage(8);
            else intakeMotor.setVoltage(kIntakeVoltage);
            state = State.Intaking;
        }
        // if the note was just detected on this cycle, start the timer
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

    /** Spins the intake motors in reverse */
    public void outtake() {
        if (state!=State.Outtaking) {
            intakeMotor.setVoltage(kOuttakeVoltage);
            state = State.Outtaking;
        }
    }

    /** If the state is in intaking, it intakes, otherwise it idles */
    public void toggleIntake() {
        if (state != State.Intaking) intake();
        else idle();
    }

    /** Spins the top shooter wheels */
    public void prepareShoot() {
        if (state != State.PreparingShoot) {
            shooterMotor.setVoltage(kShooterVoltage);
            ampMotor.setVoltage(-kShooterVoltage);
            state = State.PreparingShoot;
        }
    }

    /** Spins the shooter wheels for amping. The motors spin in the same direction */
    public void prepareAmp () {
        if (state != State.PreparingAmp) {
            shooterMotor.setVoltage(kAmpVoltage);
            ampMotor.setVoltage(kAmpVoltage);
            state = State.PreparingAmp;
        }
    }

    /** If the state is in preparing amp, it idles, otherwise it spins shooter motors for amping */
    public void toggleSpinupAmp() {
        if (state == State.PreparingAmp) idle();
        else prepareAmp();
    }

    /** If the state is in PreparShoot in idles, otherwise it spins up for shooting */
    public void toggleSpinupShoot() {
        if (state == State.PreparingShoot) idle();
        else prepareShoot();
    }

    /**
     * Returns true if the state is in PreparingShoot and the shooter is spunup.
     * Also returns true if the state is in PreparingiAmp and the shooter is spunup for amping.
     * The RPM for spunup is different for amp and shooter (kSpinupRPM, kAmpSpinupRPM).
     *
     * @return true if it is spunup, else false
     */
    public boolean spunUp() {
        if (state == State.PreparingShoot)
            return Math.abs(shooterMotor.getVelocity()) > kSpinupRPM;
        if (state == State.PreparingAmp)
            return Math.abs(shooterMotor.getVelocity()) > kAmpSpinupRPM;
        return false;
    }

    /**
     * Feed the note into the shooter by setting the intake motor to the feed voltage.
     */
    public void feed (){
        /* if the state is not already in amping, set the state to amping and the voltage to kAmpFeedVoltage, which
           will feed the note into the shooter */
        if (state != State.Feeding) {
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
        if (detectDelayTimer.hasElapsed(kFeedDetectDelay)) {
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
        if (state != State.Idling) {
            shooterMotor.setVoltage(kIdleVoltage);
            ampMotor.setVoltage(-kIdleVoltage);
            intakeMotor.setVoltage(0);
            state = State.Idling;
        }
    }

    /**
     * Returns a boolean of whether the note sensors detected anything.
     * Either sensor needs to detect for it to return true.
     *
     * @return if the note sensor detected anything
     */
    public boolean noteDetected() {
        return !noteSensor.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("[Gerald] state", state.toString());
        SmartDashboard.putBoolean("[Gerald] spun up", spunUp());
        SmartDashboard.putNumber("[Gerald] shooter RPM", shooterMotor.getVelocity());
        SmartDashboard.putNumber("[Gerald] amp RPM", ampMotor.getVelocity());
        SmartDashboard.putBoolean("[Note Sensor] both", noteDetected()); // show on advantage scope
        SmartDashboard.putBoolean("[Note Sensor] 1", !noteSensor.get());
        SmartDashboard.putBoolean("[Note Sensor] 2", !noteSensor2.get());

        // if only the first sensor is detected, low the intake voltage
//        if (!noteSensor.get() && noteSensor2.get() && state == State.Intaking)
//            intakeMotor.setVoltage(0);
//        if (!noteSensor.get() && !noteSensor2.get() && state == State.Intaking)
//            intakeMotor.setVoltage(0);

        switch (state) {
            case Idling -> idle();
            case Feeding -> feed();
            case Intaking -> intake();
            case PreparingAmp -> prepareAmp();
            case PreparingShoot -> prepareShoot();
            case Outtaking -> outtake();
        }
    }
}
