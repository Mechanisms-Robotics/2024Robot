package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmWrist;
import frc.robot.subsystems.Gerald;

/** Shoot the preload note */
public class Preload extends SequentialCommandGroup {
    /**
     * 1. Spinup the shooter
     * 2. Point the arm and the wrist in the subwoofer high position
     * 3. Wait 3 second
     * 4. Feed the note in the shooter to shoot the note
     *
     * @param gerald used for spinning up and feeding the note
     * @param armWrist used for pointing the arm and wrist in the subwoofer high position
     */
    public Preload(Gerald gerald, ArmWrist armWrist) {
        addCommands(
                new InstantCommand(gerald::prepareShoot),
                new InstantCommand(() -> armWrist.subwooferShoot(false)),
                new WaitCommand(3),
                new FeedNote(gerald)
        );
    }
}
