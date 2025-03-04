package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkFlex;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;

public class ElevatorSubsystem extends SubsystemBase {
    private double maxVel = 120;
    private double maxAccel = 180;
    ProfiledPIDController elevator_PID = new ProfiledPIDController(2, 0, 0,
            new TrapezoidProfile.Constraints(maxVel, maxAccel));// noice
    private double elevatorCurrentPose = 0;
    private double setpoint = 0;
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0.07, 0.015, 0, 0);
    private SparkFlex motor1 = new SparkFlex(24, MotorType.kBrushless);
    private SparkFlex motor2 = new SparkFlex(26, MotorType.kBrushless);
    private double highestPoint = 63.5;
    private double lowestPoint = 0.00;
    private final double MAX_VOLTAGE = 1.1;

    private ElevatorSimulation elevatorSim = new ElevatorSimulation(motor1);
    private ArmSubsystem arm;

    double elevatorVoltage = 0;
    double calculatedPid = 0;

    public ElevatorSubsystem(ArmSubsystem arm) {
        this.arm = arm;
        motor1.getEncoder().setPosition(0);
        motor2.getEncoder().setPosition(0);

        // elevatorSimMotor.setInput(0);

        elevator_PID.setTolerance(3);
    }

    @Override
    public void simulationPeriodic() {
        elevatorSim.simPeriodic();
    }

    @Override
    public void periodic() {
        elevatorCurrentPose = motor1.getEncoder().getPosition();
        calculatedPid = calculatePid(setpoint);
        elevatorVoltage = calculatedPid;

        if (elevatorVoltage > MAX_VOLTAGE) {
            elevatorVoltage = MAX_VOLTAGE;
        } else if (elevatorVoltage < -MAX_VOLTAGE) {
            elevatorVoltage = -MAX_VOLTAGE;
        }

        if (elevatorCurrentPose > highestPoint) {
            elevatorVoltage = Math.min(elevatorVoltage, 0);
        } else if (elevatorCurrentPose < lowestPoint) {
            elevatorVoltage = Math.max(elevatorVoltage, 0);
        }

        if (elevatorCurrentPose < 7) {
            elevatorVoltage = Math.max(-.1, elevatorVoltage);
        }

        if (!arm.isInSafeArea() && !Robot.isSimulation()) {
            elevatorVoltage = m_feedforward.calculate(0, 0);
        }

        motor1.set(elevatorVoltage);
        motor2.set(-elevatorVoltage);

        elevatorSim.periodic();
    }

    private double calculatePid(double position) {
        // updatePivotAngle();
        double pid = elevator_PID.calculate(elevatorCurrentPose, position);
        var setpoint = elevator_PID.getSetpoint();

        double feedforward = m_feedforward.calculate(setpoint.velocity, 0);
        return (feedforward + pid) / RobotController.getBatteryVoltage(); // +pid
    }

    public Command goDown() { // for manual control, sick
        Command command = Commands.sequence(arm.goToSetpoint(Constants.ArmConstants.OUTSIDE_ELEVATOR), runOnce(() -> {
            setSetpoint(lowestPoint + 2);
        }), Commands.waitUntil(() -> {
            return isAtSetpoint();
        }), arm.goToSetpoint(Constants.ArmConstants.HANDOFF_POSE));
        command.setName("goDown");
        return command;
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

    public Command goLittleDown(double constant) {
        // for manual control, sick
        return runOnce(() -> {
            setSetpoint(elevatorCurrentPose - constant);
        });
    }

    public Command goToSetpoint(double position) {
        return Commands.sequence(arm.goToSetpoint(Constants.ArmConstants.OUTSIDE_ELEVATOR), runOnce(() -> {
            setSetpoint(position);
        }));
    }

    private void setSetpoint(double position) {
        if (position <= highestPoint && position >= lowestPoint) {
            setpoint = position;
        }
    }

    public Command stopElevator() { // for manual control, sick
        return runOnce(() -> {
            setSetpoint(elevatorCurrentPose);
        });
    }

    public double getElevatorVelocity() { // for manual control, sick
        return motor1.getEncoder().getVelocity();
    }

    public void resetElevatorPosition() {
        setpoint = 0;
    }

    public double getHeight() {
        return elevatorCurrentPose;
    }

    public boolean isAtSetpoint() {
        return (Math.abs(getHeight() - setpoint) < 2);
        // return elevator_PID.atGoal();
    }

    // public double getHeightSim() {

    // }

    @Override
    public void initSendable(SendableBuilder builder) {
        // This is used to add things to NetworkTables
        super.initSendable(builder);

        builder.addDoubleProperty("elevatorVoltage", () -> elevatorVoltage, null);
        builder.addDoubleProperty("calculatedPid", () -> calculatedPid, null);
        builder.addDoubleProperty("setpoint", () -> setpoint, null);
        builder.addDoubleProperty("encoderPosition", this::getHeight, null);
        builder.addBooleanProperty("isAtSetpoint", this::isAtSetpoint, null);

        if (Robot.isSimulation()) {
            elevatorSim.initSendable(builder);
        }
    }
}
