package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Gerald;

public class Idle extends InstantCommand {
    public Idle(Gerald gerald) {
        super(gerald::idle);
    }
}
