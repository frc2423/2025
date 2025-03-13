package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class FRCcolors implements Led {
    public void start(AddressableLEDBuffer buffer, int length) {
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        for (var i = 0; i < buffer.getLength() / 3; i++) {
            buffer.setRGB(i, 200, 00, 00);
        }

        for (var i = buffer.getLength() / 3; i < (buffer.getLength() / 3) * 2; i++) {
            buffer.setRGB(i, 00, 00, 00);
        }

        for (var i = (buffer.getLength() / 3) * 2; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 00, 00, 200);
        }
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}