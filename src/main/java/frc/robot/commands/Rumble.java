package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Rumble extends SequentialCommandGroup {
    private static final double kValue = 1.0;
    private static final double kSeconds = 0.25;
    /**
     * Rumbles the controller at kValue and kSeconds
     *
     * @param controller controller for the rumble to be applied to
     */
    public Rumble(XboxController controller) {
        addCommands(
                new InstantCommand(
                        () -> controller.setRumble(GenericHID.RumbleType.kBothRumble, kValue)),
                new WaitCommand(kSeconds),
                new InstantCommand(
                        () -> controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0))
        );
    }

    /**
     * Rumbles the two given controllers at kValue and kSeconds
     *
     * @param controller1 controller for the rumble to be applied to
     * @param controller2 second controller for the rumble to be applied to
     */
    public Rumble(XboxController controller1, XboxController controller2) {
        addCommands(
                new InstantCommand(
                        () -> controller1.setRumble(GenericHID.RumbleType.kBothRumble, kValue)),
                new InstantCommand(
                        () -> controller2.setRumble(GenericHID.RumbleType.kBothRumble, kValue)),
                new WaitCommand(kSeconds),
                new InstantCommand(
                        () -> controller2.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)),
                new InstantCommand(
                        () -> controller2.setRumble(GenericHID.RumbleType.kBothRumble, 0.0))
        );
    }
}
