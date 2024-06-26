package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmWrist;
import frc.robot.subsystems.Gerald;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Swerve;

/**
 * Spins up the shooter, aims, shoots, stows.
 * Used for autos to group the spinup shoot and intake for making autos.
 */
public class AutoAimShootStow extends SequentialCommandGroup {
    private static final double kShootWaitTime = 1.;
    private static final double kFeedTime = .5;

    /**
     * Spins up the shooter, aims at the target, shoots, and intakes.
     * Calls PrepareShoot -> DriveWhileAim -> FeedNote -> WaitCommand -> StowPosition
     *
     * @param armWrist instance of armWrist, use for aiming at the target and stowing
     * @param gerald instance of gerald, used for shooting
     * @param swerve instance of swerve, used to aim at the target
     * @param limeLight instance of limeLight, used to locate the target
     */
    public AutoAimShootStow(ArmWrist armWrist, Gerald gerald, Swerve swerve, LimeLight limeLight) {
        addCommands(new PrepareShoot(gerald), // spins up the shooter
                    new DriveWhileAim(swerve, limeLight, armWrist).withTimeout(kShootWaitTime), // aims at the target
                    new FeedNote(gerald), // feeds the note into the shooter
                    new WaitCommand(kFeedTime), // waits for the note to exit the shooter
                    new StowPosition(armWrist)); // stows arm and wrist
    }

    /**
     * Spins up the shooter, aims at the target, shoots, and intakes.
     * Calls PrepareShoot -> SelectedAim -> WaitCommand -> FeedNote -> WaitCommand -> StowPosition
     *
     * @param armWrist instance of armWrist, use for aiming at the target and stowing
     * @param gerald instance of gerald, used for shooting
     * @param subWoofer true: subWoofer, false: podium
     * @param lowMode true: low mode, false: high mode
     */
    public AutoAimShootStow(ArmWrist armWrist, Gerald gerald, boolean subWoofer, boolean lowMode) {
        addCommands(new PrepareShoot(gerald), // spins up the shooter
                Commands.either(
                        Commands.either(
                                new SubwooferLowPosition(armWrist), // if subWoofer: true && lowMode: true
                                new SubwooferHighPosition(armWrist), // if subWoofer: true && lowMode: false
                                () -> lowMode),
                        Commands.either(
                                new PodiumLowPosition(armWrist), // if subWoofer: false && lowMode: true
                                new PodiumHighPosition(armWrist), // if subWoofer: false && lowMode: false
                                () -> lowMode),
                        () -> subWoofer),
                new WaitCommand(kShootWaitTime), // waits for the spin up and aim
                new FeedNote(gerald), // feeds the note into the shooter
                new WaitCommand(kFeedTime), // waits for the note to exit the shooter
                new StowPosition(armWrist)); // stows the arm and wrist
    }
}
