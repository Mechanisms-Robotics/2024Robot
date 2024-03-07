package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Gerald;

/**
 * Spins up the shooter motors.
 * Used for shooting in the subwoofer motor.
 */
public class PrepareShoot extends InstantCommand {

    /**
     * Calls gerald.prepareShoot to spin up the shooter motors
     *
     * @param gerald
     */
    public PrepareShoot(Gerald gerald) {
        super(gerald::prepareShoot);
    }
}
