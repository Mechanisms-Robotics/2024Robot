package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

public class UnDisable extends InstantCommand {
    public UnDisable(Arm arm) {
        super(arm::unDisable);
    }
}
