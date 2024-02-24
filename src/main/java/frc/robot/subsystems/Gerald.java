package frc.robot.subsystems;

import com.mechlib.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gerald extends SubsystemBase {
    // intake percent speed for the intake motor to run at
    private static final double kIntakeVoltage = 5.0; // volts
    private static final double kOuttakeVoltage = -5; // volts
    private static final double kShooterVoltage = 6;
    private static final double kFeedVoltage = 3; // volts
    private static final double kAmpVoltage = 5; // volts

    // naming is based off of the unique function
    private final TalonFX intakeMotor = new TalonFX(14);
    // allows for amp and assists shooter
    private final TalonFX ampMotor = new TalonFX(15);
    // only shoots
    private final TalonFX shooterMotor = new TalonFX(16);


    public Gerald() {
        intakeMotor.brakeMode();
        ampMotor.brakeMode();
        shooterMotor.brakeMode();
        intakeMotor.setInverted(false);
        ampMotor.setInverted(true);
        shooterMotor.setInverted(false);


        intakeMotor.setVoltageCompensation(10);
        ampMotor.setVoltageCompensation(10);
        shooterMotor.setVoltageCompensation(10);
        intakeMotor.setCurrentLimit(40);
        ampMotor.setCurrentLimit(40);
        shooterMotor.setCurrentLimit(40);

    }

    /**
     * Set the intake motor speed (percent) to kIntakeSpeed
     */
    public void intake() {
        intakeMotor.setVoltage(kIntakeVoltage);
    }

    /**
     * Sets the outtake motor speed (percent) to kOuttakeSpeed
     */
    public void outtake() {
        intakeMotor.setVoltage(kOuttakeVoltage);
    }


    /**
     * If the shooter speed is not the target speed, spin. When it is up to speed, move the note
     * with the intakeMotor by setting the speed to the kFeedSpeed percent
     */
    public void shoot() {
        shooterMotor.setVoltage(kShooterVoltage);
        ampMotor.setVoltage(kShooterVoltage);
        
    }
    public void amp () {
        shooterMotor.setVoltage(kAmpVoltage);
        ampMotor.setVoltage(-kAmpVoltage);
    }
    public void feed (){
        intakeMotor.setVoltage(kFeedVoltage);

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

}
