package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmWrist;

public class SubwooferLowPosition extends InstantCommand {
    public SubwooferLowPosition(ArmWrist armWrist) {
        super(() -> armWrist.subwooferShoot(true));
    }
}
