package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class AutoDown implements Led {
    private static final double DURATION = 32.0; // Total seconds before LEDs turn off
    private final Timer timer = new Timer();

    public AutoDown() {

    }

    public void start(AddressableLEDBuffer buffer, int length) {
        timer.reset();
        timer.start();
        // Start all LEDs at full brightness
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setHSV(i, 240, 255, 255);
        }
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        double timeElapsed = timer.get();
        // if (timeElapsed > DURATION) {
        // return;
        // }

        // Calculate how many LEDs should be off based on time
        int ledsToTurnOff = (int) ((timeElapsed / DURATION) * buffer.getLength());

        // Turn off LEDs one by one from the start
        for (int i = 0; i < buffer.getLength(); i++) {
            if (i < ledsToTurnOff) {
                // LED should be off
                buffer.setHSV(i, 240, 255, 0);
            } else {
                // LED should still be on
                buffer.setHSV(i, 240, 255, 255);
            }
        }
    }

    public void end(AddressableLEDBuffer buffer, int length) {
        // timer.stop();
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setHSV(i, 240, 255, 0);
        }
    }
}
