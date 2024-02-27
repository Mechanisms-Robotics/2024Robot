package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmWrist;
import frc.robot.subsystems.Gerald;

public class PrepareShootHighSubwoofer extends Command {
    private final Gerald gerald;
    private final ArmWrist armWrist;
    public PrepareShootHighSubwoofer(Gerald gerald, ArmWrist armWrist) {
        this.gerald = gerald;
        this.armWrist = armWrist;
        addRequirements(gerald, armWrist);
    }

    @Override
    public void initialize() {
        gerald.shoot();
        armWrist.subwooferShoot(false);
    }
}
