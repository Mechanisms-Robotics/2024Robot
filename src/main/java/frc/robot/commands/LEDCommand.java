package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmWrist;
import frc.robot.subsystems.Gerald;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LimeLight;

import java.util.function.Supplier;

public class LEDCommand extends Command {
    private final LED led;
    private final Gerald gerald;
    private final Supplier<LimeLight.LimeLightData> limeLightData;
    private final ArmWrist armWrist;

    public LEDCommand(LED led, Gerald gerald, Supplier<LimeLight.LimeLightData> limeLightData, ArmWrist armWrist) {
        this.led = led;
        this.gerald = gerald;
        this.limeLightData = limeLightData;
        this.armWrist = armWrist;
        addRequirements(led);
    }

    @Override
    public void execute() {
        SmartDashboard.putString("[LED] color", led.getLEDData().color());
        // default to off
        if (!gerald.spunUp()) led.off();
        if (gerald.getState() == Gerald.State.Intaking) {
            led.yellow();
            return;
        }
        if (gerald.noteDetected() && gerald.getState() == Gerald.State.Idling) {
            led.green();
            return;
        }
        // if the shooter is spun-up set it to red
        if (gerald.spunUp()) {
            led.red();
            // if the states is podium or aiming and the shooter is spun-up, set the light to blue
            if ((armWrist.getState() == ArmWrist.State.ShootingPodium
                    || armWrist.getState() == ArmWrist.State.Aiming)
                    && limeLightData.get().aimed() && armWrist.aimed()) {
                    led.blue();
            }
        }
    }
}

