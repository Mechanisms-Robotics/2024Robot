package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmWrist;
import frc.robot.subsystems.Gerald;

/**
 * Spins up the intake for intaking notes.
 * If armWrist is passed in, it will also pivot armWrist.
 */
public class IntakeCommand extends ParallelCommandGroup {
    /**
     * Spins up the intake motors.
     *
     * @param gerald instance of gerald
     */
    public IntakeCommand(Gerald gerald) {
        addCommands(new InstantCommand(gerald::intake));
    }

    /**
     * Pivots to the intake position and spins up gerald
     *
     * @param armWrist instance of armWrist, used for pivoting
     * @param gerald instance of gerald
     */
    public IntakeCommand(ArmWrist armWrist, Gerald gerald) {
        addCommands(new IntakePosition(armWrist), new IntakeCommand(gerald));
    }
}