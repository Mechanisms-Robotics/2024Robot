package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmWrist;

/**
 * Safety that sets the armWrist to the intake position.
 * No commands will run in armWrist while this command is being executed.
 * When the command is done, it disables safe mode.
 */
public class Savery extends Command {
    private final ArmWrist armWrist;

    /**
     * Instantiates armWrist
     * @param armWrist instance of armWrist
     */
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
