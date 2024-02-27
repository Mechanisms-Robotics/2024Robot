package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmWrist;

public class IntakePosition extends InstantCommand {
    public IntakePosition(ArmWrist armWrist) {
        super(armWrist::intake);
    }
}
