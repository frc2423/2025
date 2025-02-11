package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FunnelSubsystem extends SubsystemBase {
    
    private SparkMax orangeMotor = new SparkMax(11, MotorType.kBrushless);
    private SparkMax greenMotor = new SparkMax(12, MotorType.kBrushless);

    public FunnelSubsystem() {
        setDefaultCommand(stop());
    }

    private void spinInBoth(double speed){
        orangeMotor.set(-speed);
        greenMotor.set(-speed);

    }

    private void spinOutBoth(double speed){
        orangeMotor.set(speed);
        greenMotor.set(speed);

    }

    private void spinOutOrange(double speed){
        orangeMotor.set(speed);
  

    }

    private void spinOutGreen(double speed){
        greenMotor.set(speed);
  

    }

    private void spinInGreen(double speed){
        greenMotor.set(-speed);
  

    }

    private void spinInOrange(double speed){
        orangeMotor.set(-speed);
  

    }

    private void funnelStop(){
        orangeMotor.set(0);
        greenMotor.set(0);

  

    }

     public Command spinInBoth(){
        var command = run(() -> {
            spinInBoth(0.5);
        });
        command.setName("Spin IN BOTH (funnel)");
        return command;
    }

    public Command spinOutBoth(){
        var command = run(() -> {
            spinOutBoth(0.1);
        });
        command.setName("Spin OUT BOTH (funnel)");
        return command;
    }

    public Command spinOutOrange(){
        var command = run(() -> {
            spinOutOrange(0.1);
        });
        command.setName("Spin OUT ORANGE");
        return command;
    }

    public Command spinOutGreen(){
        var command = run(() -> {
            spinOutGreen(0.1);
        });
        command.setName("Spin OUT GREEN");
        return command;
    }

    public Command spinInGreen(){
        var command = run(() -> {
            spinInGreen(0.1);
        });
        command.setName("Spin IN GREEN");
        return command;
    }

    public Command spinInOrange(){
        var command = run(() -> {
            spinInOrange(0.1);
        });
        command.setName("Spin IN ORANGE");
        return command;
    }

    public Command stop(){
        var command = run(() -> {
            funnelStop();
        });
        command.setName("STOP");
        return command;
    }

}
