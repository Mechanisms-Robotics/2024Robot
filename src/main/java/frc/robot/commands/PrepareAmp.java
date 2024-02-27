package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gerald;
import frc.robot.subsystems.Wrist;

public class PrepareAmp extends Command {
    private final Gerald gerald;
    private final Arm arm;
    private final Wrist wrist;
    public PrepareAmp(Gerald gerald, Arm arm, Wrist wrist) {
        this.gerald = gerald;
        this.arm = arm;
        this.wrist = wrist;
        addRequirements(gerald, arm, wrist);
    }

    @Override
    public void initialize() {
        gerald.amp();
        arm.shoot();
        wrist.shoot();
    }
}
