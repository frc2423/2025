package frc.robot.subsystems.Claw;

import edu.wpi.first.wpilibj.Servo;

public class ClawSubsystem {
    public double releaseSetpoint = 50;
    public double resetSetpoint = 0;
    public double currentSetpoint;
    // To-Do: Test the actual angle that it should be set to to hold the claw open
    // and to not be tensioning the string
    private Servo motor = new Servo(3);

    public ClawSubsystem() {
        stop();
    }

    public double getSetpoint() {
        return currentSetpoint;
    }

    public void release() {
        motor.setAngle(releaseSetpoint);
        currentSetpoint = releaseSetpoint;
    }

    public void stop() {
        motor.setAngle(resetSetpoint);
        currentSetpoint = resetSetpoint;
    }
}