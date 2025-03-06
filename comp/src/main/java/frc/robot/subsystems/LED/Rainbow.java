package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Rainbow implements Led {
    int huePosition = 0;

    public void start(AddressableLEDBuffer buffer, int length) {
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        for (int i = 0; i < buffer.getLength(); i++) {
            int hue = (huePosition + i) % 360; // Calculate the hue (note: hue can go from 0 to 360)
            buffer.setHSV(i, hue, 255, 255); // Assuming setHSV(index, hue, saturation, value) is available
        }
        huePosition = (huePosition + 2) % 360; // Move the rainbow
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}