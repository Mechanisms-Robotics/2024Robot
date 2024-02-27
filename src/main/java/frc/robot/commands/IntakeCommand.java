package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmWrist;
import frc.robot.subsystems.Gerald;

public class IntakeCommand extends InstantCommand {

    public IntakeCommand(Gerald gerald) {
        super(gerald::intake);
    }
}