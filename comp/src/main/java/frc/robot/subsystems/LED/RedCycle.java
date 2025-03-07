package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class RedCycle implements Led {
    private int position = 0;
    private static final int FADE_LENGTH = 10; // Adjust for a smoother or sharper fade

    public void start(AddressableLEDBuffer buffer, int length) {
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        for (int i = 0; i < buffer.getLength(); i++) {
            int distance = Math.abs(i - position);

            // Create a smooth fade effect with brightness based on distance
            int brightness = Math.max(0, 255 - (distance * (255 / FADE_LENGTH)));

            buffer.setHSV(i, 0, 255, brightness); // 120 is the hue for green
        }

        // Move position forward to animate the effect
        position = (position + 1) % buffer.getLength();
    }

    public void end(AddressableLEDBuffer buffer, int length) {
    }
}
