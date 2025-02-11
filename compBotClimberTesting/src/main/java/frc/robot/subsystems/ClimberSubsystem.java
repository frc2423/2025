package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {    
    private SparkMax motor1 = new SparkMax(22, MotorType.kBrushless);

  
    public ClimberSubsystem() {
        motor1.getEncoder().setPosition(0);
        setDefaultCommand(climbStop());
    }

    private void go(double speed){
        motor1.set(speed);
    }

    private void stop() { // for manual control, sick
        motor1.set(0);
    }

    public Command climb(){
        var command = run(() -> {
            go(-.1);
        });
        command.setName("Climber going up");
        return command;
    }

    public Command deClimb(){
        var command = run(() -> {
            go(.1);
        });
        command.setName("Climber going down");
        return command;
    }

    public Command climbStop() {
        var command = run(() -> stop());
        command.setName("Climber Stop");
        return command;
    }
}
