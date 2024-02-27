package frc.robot.commands;

import frc.robot.subsystems.ArmWrist;
import frc.robot.subsystems.Gerald;

public class AmpNote extends ShootNote {
    public AmpNote(Gerald gerald, ArmWrist armWrist) {
        super(gerald, armWrist);
        feedTime = 1.0;
    }
}
