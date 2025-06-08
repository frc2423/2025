package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Intake.IntakeCommands;

public class ElevatorSubsystem extends SubsystemBase {
    private final double safeElevatorSpeedMultiplier = 0.75; // .15; // 0.5
    private double maxVel = 240 * safeElevatorSpeedMultiplier;
    private double maxAccel = 300 * safeElevatorSpeedMultiplier;

    ProfiledPIDController elevator_PID = new ProfiledPIDController(2, 0, 0,
            new TrapezoidProfile.Constraints(maxVel, maxAccel));// noice
    private double encoderPosition = 0;
    private double setpoint = 0;
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0.07, 0.015, 0, 0); // kg value with
                                                                                                  // springs is 0.0015
    private SparkFlex motor1 = new SparkFlex(24, MotorType.kBrushless);
    private SparkFlex motor2 = new SparkFlex(26, MotorType.kBrushless);
    private double highestPoint = 63.5;
    private double lowestPoint = 0.00;
    private final double MAX_VOLTAGE = 1.1;

    private ElevatorSimulation elevatorSim = new ElevatorSimulation(motor1);
    private ArmSubsystem arm;
    private IntakeCommands intake;

    double elevatorVoltage = 0;
    double calculatedPid = 0;

    public ElevatorSubsystem(ArmSubsystem arm, IntakeCommands intake) {
        this.arm = arm;
        this.intake = intake;
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
        encoderPosition = motor1.getEncoder().getPosition();
        calculatedPid = calculatePid(setpoint);
        elevatorVoltage = calculatedPid;

        if (elevatorVoltage > MAX_VOLTAGE) {
            elevatorVoltage = MAX_VOLTAGE;
        } else if (elevatorVoltage < -MAX_VOLTAGE) {
            elevatorVoltage = -MAX_VOLTAGE;
        }

        if (encoderPosition > highestPoint) {
            elevatorVoltage = Math.min(elevatorVoltage, 0);
        } else if (encoderPosition < lowestPoint) {
            elevatorVoltage = Math.max(elevatorVoltage, 0);
        }

        elevatorVoltage = Math.max(-.65, elevatorVoltage); // Used to be -0.2

        if (encoderPosition < 15) {
            elevatorVoltage = Math.max(-.4, elevatorVoltage); // Used to be -0.1
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
        double pid = elevator_PID.calculate(encoderPosition, position);
        var setpoint = elevator_PID.getSetpoint();

        double feedforward = m_feedforward.calculate(setpoint.velocity, 0);
        return (feedforward + pid) / RobotController.getBatteryVoltage(); // +pid
    }

    public Command goDown() { // for manual control, sick
        Command command = Commands.sequence(arm.goToSetpoint(Constants.ArmConstants.OUTSIDE_ELEVATOR), runOnce(() -> {
            setSetpoint(lowestPoint + 1);
        }), Commands.waitUntil(() -> {
            return isAtSetpoint();
        }), arm.goToSetpoint(Constants.ArmConstants.HANDOFF_POSE));
        command.setName("goDown");
        return command;
    }

    public Command goDownAndIntake() {
        Command command = Commands.sequence(goDown(), intake.in()/* , intake.backwards().withTimeout(.1) */);
        command.setName("goDownAndIntake");
        return command;
    }

    public Command goUp() {
        // for manual control, sick
        return goToSetpoint(highestPoint);
    }

    public Command goLittleUp(double constant) {
        // for manual control, sick
        return runOnce(() -> {
            setSetpoint(encoderPosition + constant);
        });
    }

    public Command goLittleDown(double constant) {
        // for manual control, sick
        return runOnce(() -> {
            setSetpoint(encoderPosition - constant);
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
            setSetpoint(encoderPosition);
        });
    }

    public Command scoreAlgae() {
        return Commands.sequence(goToSetpoint(Constants.SetpointConstants.REEF_L4),
                Commands.waitUntil(() -> {
                    return isAtSetpoint();
                }),
                arm.goToSetpoint(Constants.ArmConstants.ALGAE_SCORE));
        // intake.ejectAlgae());
    }

    public Command intakeAlgae(double setpoint) {
        return Commands.sequence(goToSetpoint(setpoint),
                Commands.waitUntil(() -> {
                    return isAtSetpoint();
                }),
                arm.goToSetpoint(Constants.ArmConstants.ALGAE_INTAKE),
                intake.intakeAlgae());
    }

    public Command intakeGroundAlgae() {
        return Commands.sequence(goToSetpoint(Constants.SetpointConstants.ZERO),
                Commands.waitUntil(() -> {
                    return isAtSetpoint();
                }),
                arm.goToSetpoint(Constants.ArmConstants.ALGAE_GROUND),
                intake.intakeAlgae());
    }

    public Command outtakeAlgae(double setpoint) {
        return Commands.sequence(goToSetpoint(setpoint),
                Commands.waitUntil(() -> {
                    return isAtSetpoint();
                }),
                arm.goToSetpoint(Constants.ArmConstants.ALGAE_SCORE),
                intake.intakeOut());
    }

    public double getElevatorVelocity() { // for manual control, sick
        return motor1.getEncoder().getVelocity();
    }

    public void resetElevatorPosition() {
        setpoint = 0;
    }

    public double getEncoderPosition() {
        return encoderPosition;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public boolean isAtSetpoint() {
        return (Math.abs(getEncoderPosition() - setpoint) < 2);
        // return elevator_PID.atGoal();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // This is used to add things to NetworkTables
        super.initSendable(builder);

        builder.addDoubleProperty("elevatorVoltage", () -> elevatorVoltage, null);
        builder.addDoubleProperty("calculatedPid", () -> calculatedPid, null);
        builder.addDoubleProperty("setpoint", () -> setpoint, null);
        builder.addDoubleProperty("encoderPosition", this::getEncoderPosition, null);
        builder.addBooleanProperty("isAtSetpoint", this::isAtSetpoint, null);
        builder.addDoubleProperty("motor1Current", motor1::getOutputCurrent, null);
        builder.addDoubleProperty("motor2Current", motor2::getOutputCurrent, null);

        if (Robot.isSimulation()) {
            elevatorSim.initSendable(builder);
        }
    }
}
