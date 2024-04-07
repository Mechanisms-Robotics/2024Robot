package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

/** Zeros the swerve gyro for field oriented */
public class ZeroGyro extends InstantCommand {
    /**
     * Calls swerve.zeroGyro
     *
     * @param swerve instance of swerve
     */
    public ZeroGyro(Swerve swerve) {
        super(swerve::zeroGyro);
    }
}
