package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmWrist;
import frc.robot.subsystems.Gerald;

public class PrepareShoot extends ParallelCommandGroup {
    private final Gerald gerald;
    public PrepareShoot(Gerald gerald) {
        this.gerald = gerald;
        addCommands(new InstantCommand(gerald::prepareShoot));
    }
}
