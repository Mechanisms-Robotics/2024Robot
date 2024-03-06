package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmWrist;

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
