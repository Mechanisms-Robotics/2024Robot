package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class Aim extends ParallelCommandGroup {
    public Aim (Arm arm, Wrist wrist, double armAngle, double wristAngle) {
        addCommands(
                new InstantCommand(() -> arm.aim(Rotation2d.fromDegrees(armAngle))),
                new InstantCommand(() -> wrist.aim(Rotation2d.fromDegrees(wristAngle)))
        );
    }
}
