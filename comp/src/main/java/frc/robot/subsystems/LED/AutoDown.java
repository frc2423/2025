package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class AutoDown implements Led {
    private static final int DURATION = 15; // Total seconds before LEDs turn off
    private int timeElapsed = 0; // Tracks seconds elapsed

    public void start(AddressableLEDBuffer buffer, int length) {
        setBrightness(buffer, 255); // Start with full brightness
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        // Calculate new brightness based on elapsed time
        int brightness = Math.max(0, 255 - (timeElapsed * (255 / DURATION)));

        setBrightness(buffer, brightness);

        if (timeElapsed < DURATION) {
            timeElapsed++; // Increment each second
        }
    }

    public void end(AddressableLEDBuffer buffer, int length) {
        setBrightness(buffer, 0); // Ensure LEDs are off when ending
    }

    private void setBrightness(AddressableLEDBuffer buffer, int brightness) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setHSV(i, 120, 255, brightness); // 120 is green hue
        }
    }
}