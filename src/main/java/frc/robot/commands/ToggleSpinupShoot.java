package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Gerald;

public class ToggleSpinupShoot extends InstantCommand {
    public ToggleSpinupShoot(Gerald gerald) {
        super(gerald::toggleSpinupShoot);
    }
}
