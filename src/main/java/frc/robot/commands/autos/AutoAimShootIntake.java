package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveWhileAim;
import frc.robot.commands.PrepareShoot;
import frc.robot.subsystems.ArmWrist;
import frc.robot.subsystems.Gerald;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Swerve;

public class AutoAimShootIntake extends SequentialCommandGroup {
    private static final double kShootWaitTime = 1.;
    public AutoAimShootIntake(ArmWrist armWrist, Gerald gerald, Swerve swerve, LimeLight limeLight) {
        addCommands(new PrepareShoot(gerald), new DriveWhileAim(swerve, limeLight, armWrist),
                    new WaitCommand(kShootWaitTime));
    }
}
