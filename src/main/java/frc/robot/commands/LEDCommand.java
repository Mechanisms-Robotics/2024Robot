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
        if (gerald.getState() == Gerald.State.Intaking) {
            led.yellow();
            return;
        }
        if (gerald.noteDetected() && gerald.getState() == Gerald.State.Idling) {
            led.green();
            return;
        }
        // if the shooter is spun up
        if (gerald.spunUp()) {
            // if the states is podium or aiming
            if (armWrist.getState() == ArmWrist.State.ShootingPodium
                    || armWrist.getState() == ArmWrist.State.Aiming) {
                // if it is aimed set it to blue else red
                if (limeLightData.get().aimed()) {
                    led.blue();
                } else { led.red(); }
            // if it is not podium or aiming, set the light to blue
            } else { led.blue(); }
        // if the shooter is not spun up, set the led to red
        } else { led.red(); }
    }
}

