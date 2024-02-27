package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmWrist;
import frc.robot.subsystems.Gerald;

public class PrepareAmp extends Command {
    private final Gerald gerald;
    private final ArmWrist armWrist;
    public PrepareAmp(Gerald gerald, ArmWrist armWrist) {
        this.gerald = gerald;
        this.armWrist = armWrist;
        addRequirements(gerald, armWrist);
    }

    @Override
    public void initialize() {
        gerald.amp();
        armWrist.amp();
    }
}
