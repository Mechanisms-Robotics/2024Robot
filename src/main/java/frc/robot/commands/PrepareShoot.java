package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gerald;
import frc.robot.subsystems.Wrist;

/**
 * Prepares for shooting by moving the arm and wrist to the proper position and spins up the shoot motors
 */
public class PrepareShoot extends Command {
    private final Gerald gerald;
    private final Arm arm;
    private final Wrist wrist;
    public PrepareShoot(Gerald gerald, Arm arm, Wrist wrist) {
        this.gerald = gerald;
        this.arm = arm;
        this.wrist = wrist;
        addRequirements(gerald, arm, wrist);
    }

    @Override
    public void initialize() {
        gerald.shoot();
        arm.shoot();
        wrist.shoot();
    }
}
