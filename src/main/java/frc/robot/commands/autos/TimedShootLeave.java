package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.HighSubwooferShoot;
import frc.robot.commands.PrepareShoot;
import frc.robot.subsystems.ArmWrist;
import frc.robot.subsystems.Gerald;
import frc.robot.subsystems.Swerve;

/** Shoots subwoofer high, and does a timed leave */
public class TimedShootLeave extends SequentialCommandGroup {
    public TimedShootLeave(Swerve swerve, Gerald gerald, ArmWrist armWrist) {
        addCommands(
                new HighSubwooferShoot(armWrist),
                new PrepareShoot(gerald),
                new WaitCommand(2),
                new InstantCommand(gerald::feed),
                new WaitCommand(1),
                new InstantCommand(gerald::idle),
                new InstantCommand(armWrist::stow),
                new TimedLeave(swerve).withTimeout(2)
        );
        addRequirements(swerve, gerald, armWrist);
    }

}
