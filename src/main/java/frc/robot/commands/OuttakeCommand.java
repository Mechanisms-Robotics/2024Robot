package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Gerald;

/** Outtakes notes by running the gerald intake motors in reverse. */
public class OuttakeCommand extends InstantCommand {
    /**
     * Calls gerald.outtake to outtake a note
     *
     * @param gerald an instance of gerald
     */
    public OuttakeCommand(Gerald gerald) {
        super(gerald::outtake);
    }
}
