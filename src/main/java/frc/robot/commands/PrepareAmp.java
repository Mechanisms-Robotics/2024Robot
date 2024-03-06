package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmWrist;
import frc.robot.subsystems.Gerald;

public class PrepareAmp extends ParallelCommandGroup {
    private final Gerald gerald;
    public PrepareAmp(Gerald gerald) {
        this.gerald = gerald;
        addCommands(new InstantCommand(gerald::prepareAmp));
    }
}
