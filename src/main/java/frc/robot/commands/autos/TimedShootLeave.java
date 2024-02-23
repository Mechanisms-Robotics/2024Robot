//package frc.robot.commands.autos;
//
//import edu.wpi.first.wpilibj2.command.*;
//import frc.robot.subsystems.Arm;
//import frc.robot.subsystems.Gerald;
//import frc.robot.subsystems.Swerve;
//
//public class TimedShootLeave extends SequentialCommandGroup {
//    public TimedShootLeave(Swerve swerve, Gerald gerald, Arm arm) {
//        addCommands(
//                new InstantCommand(arm::shoot),
//                new WaitCommand(1),
//                new FunctionalCommand(
//                        () -> {},
//                        gerald::shoot,
//                        (interupted) -> {gerald.stopShooter();},
//                        () -> false
//                ).withTimeout(1),
//                new InstantCommand(arm::stow),
//                new TimedLeave(swerve)
//        );
//        addRequirements(swerve, gerald, arm);
//    }
//
//}
