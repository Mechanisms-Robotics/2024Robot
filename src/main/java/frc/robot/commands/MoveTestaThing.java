package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class MoveTestaThing extends Command {
    private final Swerve swerve;

    public MoveTestaThing(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.drive(0.1, 0.0, 0.0);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0);
    }
}
