package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gerald;
import frc.robot.subsystems.Swerve;

public class TimedShootLeave extends SequentialCommandGroup {
    public TimedShootLeave(Swerve swerve, Gerald gerald, Arm arm) {
        addCommands(
                new InstantCommand(arm::shoot),
                new WaitCommand(0.5),
                new InstantCommand(gerald::shoot),
                new WaitCommand(1),
                new InstantCommand(gerald::feed),
                new WaitCommand(1),
                new InstantCommand(gerald::stopIntake),
                new InstantCommand(gerald::stopShooter),
                new InstantCommand(arm::stow),
                new TimedLeave(swerve).withTimeout(2)
        );
        addRequirements(swerve, gerald, arm);
    }

}
