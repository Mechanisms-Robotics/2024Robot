package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Gerald;

public class ToggleSpinupAmp extends InstantCommand {
    public ToggleSpinupAmp(Gerald gerald) {
        super(gerald::toggleSpinupAmp);
    }
}
