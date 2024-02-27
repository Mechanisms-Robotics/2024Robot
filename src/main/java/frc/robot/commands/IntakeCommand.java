package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmWrist;
import frc.robot.subsystems.Gerald;

public class IntakeCommand extends Command{
    private static final double kDelayTime = 0.0625;
    private final ArmWrist armWrist;
    private final Gerald gerald;
    private final DigitalInput notesensor = new DigitalInput(0);
    private final Timer delayTimer = new Timer();
    private boolean delayStarted = false;

    public IntakeCommand(ArmWrist armWrist, Gerald gerald)
    {
        this.armWrist = armWrist;
        this.gerald = gerald;
        addRequirements(armWrist, gerald);  //subsystems require these commands
    }

    @Override
    public void initialize (){
        armWrist.intake();
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
        armWrist.stow();
        gerald.idle();
        delayTimer.stop();
        delayTimer.reset();
        delayStarted = false;
    }
}