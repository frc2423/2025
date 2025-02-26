package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Purple implements Led {
    public void start(AddressableLEDBuffer buffer, int length) {
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 200, 00, 200);
        }
        // yellow 250, 90, 0 (but divide)
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}