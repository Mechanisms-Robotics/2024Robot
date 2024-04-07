package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private static final DigitalOutput red = new DigitalOutput(1);
    private static final DigitalOutput blue = new DigitalOutput(2);
    private static final DigitalOutput green = new DigitalOutput(0);
    private static final DigitalOutput yellow = new DigitalOutput(3);

    public record LEDData(
            boolean red,
            boolean green,
            boolean blue,
            boolean yellow,
            boolean off,
            String color
    ) {}

    /** @return LEDData, data of the light */
    public LEDData getLEDData() {
        String color = "off";
        if (red.get()) color = "red";
        if (green.get()) color = "green";
        if (blue.get()) color = "blue";
        if (yellow.get()) color = "yellow";
        return new LEDData(red.get(), green.get(), blue.get(), yellow.get(),
                           !red.get() && !green.get() && !blue.get() && !yellow.get(), color);
    }

    /** Set the light to red */
    public void red() {
        red.set(true); // trun red on
        // turn all other lights off
        green.set(false);
        blue.set(false);
        yellow.set(false);
    }

    /** Set the light to green */
    public void green() {
        green.set(true); // turn green on
        // turn all other lights off
        red.set(false);
        blue.set(false);
        yellow.set(false);
    }

    /** Set the light to blue */
    public void blue() {
        blue.set(true); // turn blue on
        // turn all other lights off
        red.set(false);
        green.set(false);
        yellow.set(false);
    }

    /** Set the light to yellow */
    public void yellow() {
        yellow.set(true); // turn yellow on
        // turn all other lights off
        red.set(false);
        green.set(false);
        blue.set(false);
    }

    /** Turn the light off */
    public void off() {
        // turns all lights off
        red.set(false);
        green.set(false);
        blue.set(false);
        yellow.set(false);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Color", getLEDData().color);
    }
}
