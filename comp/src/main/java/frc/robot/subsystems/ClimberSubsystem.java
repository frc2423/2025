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
        setDefaultCommand(climbStop());
    }

    public void periodic() {
        double position = getPosition();
        if (Math.abs(setpoint - position) < .01) {
            motor1.set(0);
        } else if (position < setpoint) {
            motor1.set(speed);
        } else if (position > setpoint) {
            motor1.set(-speed);
        }
    }

    public double getPosition() {
        return motor1.getAbsoluteEncoder().getPosition();
    }

    private void setSetpoint(double inputposition, double inputspeed) {
        setpoint = inputposition;
        speed = inputspeed;
    }

    // private void setSetpoint(double inputspeed) {
    // speed = inputspeed;
    // }

    public Command goLittleUp(double constant) {
        // for manual control, sick
        return runOnce(() -> {
            setSetpoint(getPosition() + constant, 1);
        });
    }

    public Command goLittleDown(double constant) {
        // for manual control, sick
        return runOnce(() -> {
            setSetpoint(getPosition() - constant, 1);
        });
    }

    public Command climb() {
        var command = run(() -> {
            setSetpoint(.407, 1);
        });
        command.setName("Climber going up");
        return command;
    }

    public Command deClimbMore() {
        var command = run(() -> {
            setSetpoint(.85, 1);
        });
        command.setName("Climber going up");
        return command;
    }

    public Command deClimb() {
        var command = run(() -> {
            setSetpoint(.8, 1);
        });
        command.setName("Climber going down");
        return command;
    }

    public Command deClimbSpecial() {
        var command = Commands.sequence(deClimbMore(), deClimb());
        command.setName("Climber special");
        return command;
    }

    public Command climbStop() {
        var command = run(() -> setSetpoint(getPosition(), 0));
        command.setName("Climber Stop");
        return command;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("climbPose", () -> getPosition(), null);
        builder.addDoubleProperty("climbSetpoint", () -> setpoint, null);
        builder.addDoubleProperty("climbSpeed", () -> speed, null);
    } // -147.353760 start /

}
