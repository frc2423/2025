package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private SparkMax motor1 = new SparkMax(22, MotorType.kBrushless);
    private double setpoint = 0;
    private double speed = .1;

    public ClimberSubsystem() {
        motor1.getEncoder().setPosition(0);
        setDefaultCommand(climbStop());
    }

    public void periodic() {
        if (Math.abs(setpoint - motor1.getEncoder().getPosition()) < .1) {
            motor1.set(0);
        } else if (motor1.getEncoder().getPosition() < setpoint) {
            motor1.set(-speed);
        } else if (motor1.getEncoder().getPosition() > setpoint) {
            motor1.set(speed);
        }
    }

    private void setSetpoint(double inputposition, double inputspeed) {
        setpoint = inputposition;
        speed = inputspeed;
    }

    // private void go(double speed) {
    // motor1.set(speed);
    // }

    // private void stop() { // for manual control, sick
    // motor1.set(0);
    // }

    public Command climb() {
        var command = run(() -> {
            setSetpoint(0, .1);
        });
        command.setName("Climber going up");
        return command;
    }

    public Command deClimb() {
        var command = run(() -> {
            setSetpoint(0, .1);
        });
        command.setName("Climber going down");
        return command;
    }

    public Command climbStop() {
        var command = run(() -> setSetpoint(motor1.getEncoder().getPosition(), 0));
        command.setName("Climber Stop");
        return command;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("climbPose", () -> motor1.getAbsoluteEncoder().getPosition(), null);
        builder.addDoubleProperty("climbSetpoint", () -> setpoint, null);
        builder.addDoubleProperty("climbSpeed", () -> speed, null);
    } // -147.353760 start /

}
