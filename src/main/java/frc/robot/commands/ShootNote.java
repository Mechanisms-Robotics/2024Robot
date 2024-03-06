package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Gerald;

public class ShootNote extends InstantCommand {
    public ShootNote(Gerald gerald) {
        super(gerald::feed);
    }
}
