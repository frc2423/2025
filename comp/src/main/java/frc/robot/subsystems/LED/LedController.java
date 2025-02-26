package frc.robot.subsystems.LED;

import java.util.HashMap;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedController {
    private HashMap<String, Led> leds = new HashMap<String, Led>();
    private AddressableLED led = new AddressableLED(1);
    private AddressableLEDBuffer ledBuffer;
    private String currentLed;

    public LedController(int length) {
        ledBuffer = new AddressableLEDBuffer(length);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    public void add(String name, Led led) {
        leds.put(name, led);
    }

    public void set(String name) {
        if (!leds.containsKey(name)) {
            return;
        }
        if (currentLed != null) {
            leds.get(currentLed).end(ledBuffer, ledBuffer.getLength());
        }
        leds.get(name).start(ledBuffer, ledBuffer.getLength());
        currentLed = name;
        led.setData(ledBuffer);
    }

    public void run() {
        if (currentLed != null && leds.containsKey(currentLed)) {
            // [System.out.println("it does things");
            leds.get(currentLed).run(ledBuffer, ledBuffer.getLength());
            led.setData(ledBuffer);
        }
    }
}