package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

/**
 * Undisables the arm
 */
public class Home extends Command {
    private final Arm arm;

    public Home(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
//        arm.home();
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}
