package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmWrist;

public class Aim extends ParallelCommandGroup {
    // TODO: make this function set the state of the arm and wrist
    public Aim (ArmWrist armWrist, double armAngle, double wristAngle) {
        addCommands(
                new InstantCommand(() -> armWrist.aim(Rotation2d.fromDegrees(armAngle),
                        Rotation2d.fromDegrees(wristAngle)))
        );
    }
}
