package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmWrist;
import frc.robot.subsystems.Gerald;

public class ShootNote extends Command {
    private final Gerald gerald;
    private final ArmWrist armWrist;
    private final Timer feedTimer = new Timer();
    protected double feedTime = 1.0;
    public ShootNote(Gerald gerald, ArmWrist armWrist) {
        this.gerald = gerald;
        this.armWrist = armWrist;
        addRequirements(gerald, armWrist);
    }

    @Override
    public void initialize() {
        gerald.shoot();
        feedTimer.start();
    }

    @Override
    public boolean isFinished() {
        return feedTimer.hasElapsed(feedTime);
    }

    @Override
    public void end(boolean interrupted) {
        gerald.idle();
        armWrist.stow();
        feedTimer.stop();
        feedTimer.reset();
    }
}
