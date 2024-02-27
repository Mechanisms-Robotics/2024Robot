package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gerald;
import frc.robot.subsystems.Wrist;

public class AmpNote extends ShootNote {
    public AmpNote(Gerald gerald, Arm arm, Wrist wrist) {
        super(gerald, arm, wrist);
        feedTime = 1.0;
    }
}
