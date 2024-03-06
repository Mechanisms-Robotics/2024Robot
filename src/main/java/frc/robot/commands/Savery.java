package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmWrist;

public class Savery extends Command {
    private final ArmWrist armWrist;
    public Savery(ArmWrist armWrist) {
        this.armWrist = armWrist;
    }

    @Override
    public void initialize() {
        armWrist.enableSafeMode();
    }

    @Override
    public void end(boolean interrupted) {
        armWrist.disableSafeMode();
    }
}
