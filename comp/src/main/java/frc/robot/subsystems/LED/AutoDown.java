package frc.robot.subsystems.LED;
import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class AutoDown implements Led {
    public void start(AddressableLEDBuffer buffer, int length) {
    }

    private Timer timer = new Timer();

    

    public void run(AddressableLEDBuffer buffer, int length) {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 255, 0);
        }

        for(int i = buffer.getLength(); i > 0; i -= 2) {
            buffer.setRGB(i, 0, 0, 0);
            
            // Thread.sleep(1000);
        }

        // yellow 250, 90, 0 (but divide)
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}