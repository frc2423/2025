package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FunnelSubsystem extends SubsystemBase {

    private SparkMax motor = new SparkMax(11, MotorType.kBrushless);

    public FunnelSubsystem() {
        setDefaultCommand(stop());
    }

    private void spinIn(double speed) {
        motor.set(speed);

    }

    private void spinOut(double speed) {
        motor.set(speed);

    }

    private void funnelStop() {
        motor.set(0);

    }

    public Command spinIn() {
        var command = run(() -> {
            spinIn(-1.0);
        });
        command.setName("Spin IN (funnel)");
        return command;
    }

    public Command spinOut() {
        var command = run(() -> {
            spinOut(0.1);
        });
        command.setName("Spin OUT (funnel)");
        return command;
    }

    public Command spinOutOnce() {
        var command = run(() -> {
            spinOut(0.3);
        });
        command.setName("Spin OUT (funnel) but run once");
        return command;
    }

    public Command stop() {
        var command = runOnce(() -> {
            funnelStop();
        });
        command.setName("STOP");
        return command;
    }

}
