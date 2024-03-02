package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmWrist;

public class HighSubwooferShoot extends InstantCommand {
    public HighSubwooferShoot(ArmWrist armWrist) {
        super(() -> armWrist.subwooferShoot(false));
    }
}
