package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class AutoDown implements Led {
    public void start(AddressableLEDBuffer buffer, int length) {
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 255, 0);
        }

        // yellow 250, 90, 0 (but divide)
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}