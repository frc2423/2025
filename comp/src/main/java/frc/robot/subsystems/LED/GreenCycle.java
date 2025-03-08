package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class GreenCycle implements Led {
    private int position = 0;
    private static final int FADE_LENGTH = 20; // Adjust for a smoother or sharper fade

    public void start(AddressableLEDBuffer buffer, int length) {
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        int stripLength = buffer.getLength();

        for (int i = 0; i < stripLength; i++) {
            // Calculate shortest distance considering wrap-around
            int distance = Math.min(
                    Math.abs(i - position),
                    Math.min(
                            Math.abs(i - (position + stripLength)),
                            Math.abs(i - (position - stripLength))));

            // Create a smooth fade effect with brightness based on distance
            int brightness = Math.max(0, 255 - (distance * (255 / FADE_LENGTH)));

            buffer.setHSV(i, 240, 255, brightness);
        }

        // Move position forward to animate the effect
        position = (position + 1) % stripLength;
    }

    public void end(AddressableLEDBuffer buffer, int length) {
    }
}
