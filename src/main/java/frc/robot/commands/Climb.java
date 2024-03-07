package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmWrist;

/**
 * Puts the arm up so that it is ready to climb.
 * When the command ends, it pulls the robot down.
 */
public class Climb extends Command {
    private final ArmWrist armWrist;
    public Climb(ArmWrist armWrist) {
        this.armWrist = armWrist;
        addRequirements(armWrist);
    }

    @Override
    public void initialize() {
        armWrist.prepClimb();
    }

    @Override
    public void end(boolean interrupted) {
        armWrist.climb();
    }
}
