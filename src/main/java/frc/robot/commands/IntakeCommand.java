package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gerald;
import frc.robot.subsystems.Wrist;

public class IntakeCommand extends Command{
    private static final double kDelayTime = 0.0625;
    private final Arm arm;
    private final Wrist wrist;
    private final Gerald gerald;
    private final DigitalInput notesensor = new DigitalInput(0);
    private final Timer delayTimer = new Timer();
    private boolean delayStarted = false;

    public IntakeCommand(Arm arm, Wrist wrist, Gerald gerald)
    {
        this.arm=arm;
        this.wrist=wrist;
        this.gerald=gerald;
        addRequirements(arm, wrist, gerald);  //subsystems require these commands
    }

    @Override
    public void initialize (){
        arm.intake();
        wrist.intake();
        gerald.intake();

    }
    @Override
    public void execute(){
        SmartDashboard.putBoolean("Note Sensor", !notesensor.get()); //show on advantage scope
        if(!notesensor.get() && !delayStarted){
            delayTimer.start();
            delayStarted = true;
        }
    }

    @Override
    public boolean isFinished() {
        return delayTimer.hasElapsed(kDelayTime);
    }


    @Override
    public void end(boolean interrupted) {
        arm.stow();
        wrist.stow();
        gerald.stopIntake();
        delayTimer.stop();
        delayTimer.reset();
        delayStarted = false;
    }
}