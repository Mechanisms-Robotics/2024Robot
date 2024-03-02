package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmWrist;

public class SubwooferHighPosition extends InstantCommand {
    public SubwooferHighPosition(ArmWrist armWrist) {
        super(() -> armWrist.subwooferShoot(false));
    }
}
