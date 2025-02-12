package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {    
    private SparkMax climbMotor = new SparkMax(22, MotorType.kBrushless);
    private double speed = .1;
    private double position;
    private double minPosition = 0.098; //all the way out
    private double maxPosition = 0.522; //all the way in
    private double deadband = .05;
    private double outPosition = .5;
    private double inPosition = .1;
    private double startPosition = .168;
  
    public ClimberSubsystem() {
        climbMotor.getEncoder().setPosition(0);
        setDefaultCommand(climbStop());
    }

    @Override
    public void periodic() {
        position = getPosition();

        if (position <= minPosition) {
            goToSetpoint(minPosition);
        } else if (position > maxPosition) {
            goToSetpoint(maxPosition);
        }
    }

    private void go(double speed){
        climbMotor.set(speed);
    }

    private void stop() {
        climbMotor.set(0);
    }

    private double getPosition(){
        return climbMotor.getAbsoluteEncoder().getPosition();
    }

    private boolean isAtSetpoint(double setpoint){
        return (getPosition() < setpoint + deadband) && (getPosition() > setpoint - deadband);
    }

    public Command goToSetpoint(double setpoint){
        var command = run(() -> {
            if(setpoint >= position)
                go(speed);
            else if (setpoint < position)
                go(-speed);
        }).until(() -> isAtSetpoint(setpoint));
        command.setName("Going to setpoint");
        return command;
    }

    public Command goToStart(){
        var command = goToSetpoint(startPosition);
        command.setName("Start Position");
        return command;
    }

    public Command goToClimb(){
        var command = goToSetpoint(inPosition);
        command.setName("Climb Position");
        return command;
    }

    public Command goToOut(){
        var command = goToSetpoint(outPosition);
        command.setName("Out Position");
        return command;
    }

    public Command in(){
        var command = run(() -> {
            go(-speed);
        });
        command.setName("Climber going up");
        return command;
    }

    public Command out(){
        var command = run(() -> {
            go(speed);
        });
        command.setName("Climber going down");
        return command;
    }

    public Command climbStop() {
        var command = run(() -> stop());
        command.setName("Climber Stop");
        return command;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("setpoint", () -> getPosition(), null);
    }
}
