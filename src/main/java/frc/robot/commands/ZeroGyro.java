package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

public class ZeroGyro extends InstantCommand {
    public ZeroGyro(Swerve swerve) {
        super(swerve::zeroGyro);
    }
}
