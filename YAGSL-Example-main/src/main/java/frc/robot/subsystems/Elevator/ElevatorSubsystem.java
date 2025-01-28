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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ElevatorSubsystem extends SubsystemBase {
    private double maxVel = .05;
    private double maxAccel = .1;
    ProfiledPIDController elevator_PID = new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(15, 22.5));// noice
    private double elevatorCurrentPose = 0;
    private double setpoint = 0;
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0.07, 0.18, 0, 0);
    private SparkFlex motor1 = new SparkFlex(24, MotorType.kBrushless);
    private SparkFlex motor2 = new SparkFlex(26, MotorType.kBrushless);
    private double highestPoint = 72;
    private double lowestPoint = 0.05;
    private final double MAX_VOLTAGE = .3;

    private ElevatorSim elevatorSim = new ElevatorSim();

    // the main mechanism object
    private Mechanism2d mech = new Mechanism2d(10, 100);
    // the mechanism root node
    private MechanismRoot2d root = mech.getRoot("bottom", 5, 0);

    // private final FlywheelSim elevatorSimMotor = new
    // FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004096955);

    // MechanismLigament2d objects represent each "section"/"stage" of the
    // mechanism, and are based
    // off the root node or another ligament object
    MechanismLigament2d elevator = root.append(new MechanismLigament2d("elevator", lowestPoint, 90));
    MechanismLigament2d bottom = elevator.append(
            new MechanismLigament2d("bottom", 5, 0, 6, new Color8Bit(Color.kBlanchedAlmond)));

    public ElevatorSubsystem() {

        motor1.getEncoder().setPosition(0);
        motor2.getEncoder().setPosition(0);

        // post the mechanism to the dashboard
        SmartDashboard.putData("Mech2d", mech);
        // elevatorSimMotor.setInput(0);
    }

    @Override
    public void periodic() {
        double calculatedPID = calculatePid(setpoint);

        if (calculatedPID > MAX_VOLTAGE) {
            calculatedPID = MAX_VOLTAGE;
        } else if (calculatedPID < -MAX_VOLTAGE) {
            calculatedPID = -MAX_VOLTAGE;
        }

        if (elevatorCurrentPose > highestPoint) {
            calculatedPID = Math.min(calculatedPID, 0);
        } else if (elevatorCurrentPose < lowestPoint) {
            calculatedPID = Math.max(calculatedPID, 0);
        }

        if (Robot.isSimulation()) {
            elevatorSim.periodic();
            elevator.setLength(50);
            bottom.setLength(elevatorSim.getHeight());
        }

        motor1.set(calculatedPID);
        motor2.set(-calculatedPID); // ONE OF THEM IS NEGITIVE
    }

    private double calculatePid(double position) {
        // updatePivotAngle();
        elevatorCurrentPose = motor1.getEncoder().getPosition();
        double pid = elevator_PID.calculate(elevatorCurrentPose, position);
        var setpoint = elevator_PID.getSetpoint();

        double feedforward = m_feedforward.calculate(setpoint.velocity, 0);
        return (feedforward + pid) / RobotController.getBatteryVoltage(); // +pid
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

    public Command goLittleDown(double constant) {
        // for manual control, sick
        return runOnce(() -> {
            setSetpoint(elevatorCurrentPose - constant);
        });
    }

    public Command goToSetpoint(double position) {
        return runOnce(() -> {
            setSetpoint(position);
        });
    }

    private void setSetpoint(double position) {
        if (position < highestPoint && position > lowestPoint) {
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
        return motor1.getAbsoluteEncoder().getPosition();
    }

    // public double getHeightSim() {

    // }

    @Override
    public void initSendable(SendableBuilder builder) {
        // This is used to add things to NetworkTables
        super.initSendable(builder);

        builder.addDoubleProperty("calculatePid", () -> calculatePid(setpoint), null);
        builder.addDoubleProperty("elevatorCurrentPose", () -> elevatorCurrentPose, null);
        builder.addDoubleProperty("setpoint", () -> setpoint, null);
    }
}
