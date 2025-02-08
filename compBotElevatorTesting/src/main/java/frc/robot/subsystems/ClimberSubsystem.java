package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ClimberSubsystem extends SubsystemBase {
    private double maxVel = .05;
    private double maxAccel = .1;
    private SparkMax motor1 = new SparkMax(22, MotorType.kBrushless);
    private double setpoint = 0;

  
    public ClimberSubsystem() {
        motor1.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        double calculatedPID = calculatePid(setpoint);

        motor1.set(calculatedPID);
        motor2.set(-calculatedPID); // ONE OF THEM IS NEGITIVE
    }

    public Command goDown() { // for manual control, sick
        return goToSetpoint(lowestPoint);
    }

    public Command goUp() {
        // for manual control, sick
        return goToSetpoint(highestPoint);
    }

    public Command goLittleUp(double constant) {
        // for manual control, sick
        return runOnce(() -> {
            setSetpoint(elevatorCurrentPose + constant);
        });
    }

    public Command stopElevator() { // for manual control, sick
        return runOnce(() -> {
            setSetpoint(elevatorCurrentPose);
        });
    }

    public void resetElevatorPosition() {
        setpoint = 0;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // This is used to add things to NetworkTables
        super.initSendable(builder);

        builder.addDoubleProperty("calculatePid", () -> calculatePid(setpoint), null);
        builder.addDoubleProperty("setpoint", () -> setpoint, null);
    }
}
