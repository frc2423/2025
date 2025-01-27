package frc.robot.subsystems.Elevator;

import java.util.Date;

public class ElevatorSim {
    double initialHeight = 100;
    double height = 0;
    double voltage = 0;
    Date updateTime = new Date();

    public ElevatorSim() {
    }

    public double getHeight() {
        return height;
    }

    public void setVoltage(double incomingVoltage) {
        voltage = incomingVoltage;
    }

    public void periodic() {
        Date timePeriodic = new Date();
        double deltaT = timePeriodic.getTime() - updateTime.getTime();
        if (voltage > 0) {

        }

        else if (voltage == 0) {
            if (height < 0) {
                height = initialHeight - (0.5 * 9.8) * Math.pow(deltaT / 1000, 2);
                System.out.println(height);
                System.out.println(deltaT / 1000);
            }
            // height = -(0.5*9.8)*Math.pow(deltaT/1000,2);
        }

        else {
            System.out.println("WRONG >:(");
        }
    }
}
