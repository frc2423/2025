package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Dark implements Led {
    public void start(AddressableLEDBuffer buffer, int length) {
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0);
        }
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}