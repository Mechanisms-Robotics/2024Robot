package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gerald;
import frc.robot.subsystems.Wrist;

public class ShootNote extends Command {
    private final Gerald gerald;
    private final Arm arm;
    private final Wrist wrist;
    private final Timer feedTimer = new Timer();
    protected double feedTime = 1.0;
    public ShootNote(Gerald gerald, Arm arm, Wrist wrist) {
        this.gerald = gerald;
        this.arm = arm;
        this.wrist = wrist;
        addRequirements(gerald, arm, wrist);
    }

    @Override
    public void initialize() {
        gerald.feed();
        feedTimer.start();
    }

    @Override
    public boolean isFinished() {
        return feedTimer.hasElapsed(feedTime);
    }

    @Override
    public void end(boolean interrupted) {
        gerald.stopIntake();
        gerald.stopShooter();
        arm.stow();
        wrist.stow();
        feedTimer.stop();
        feedTimer.reset();
    }
}
