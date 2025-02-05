package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.Encoder;

public class ElevatorSubsystem extends SubsystemBase {
    private double maxVel = .05;
    private double maxAccel = .1;
    ProfiledPIDController elevator_PID = new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(15, 22.5));// noice
    private double elevatorCurrentPose = 0;
    private double setpoint = 0;
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0.07, 0.18, 0, 0);
    private SparkFlex motorL = new SparkFlex(24, MotorType.kBrushless);
    private SparkFlex motorR = new SparkFlex(26, MotorType.kBrushless);
    private double highestPoint = 72;
    private double lowestPoint = 0.05;
    private final double MAX_VOLTAGE = .3;
    private final DCMotor m_elevatorGearbox = DCMotor.getNeoVortex(2);
    private final SparkFlexSim m_motorSimL = new SparkFlexSim(motorL, m_elevatorGearbox); // the gearbox is a littlbe
                                                                                          // bit suspect, it says it
                                                                                          // should be a motor
    // private final SparkFlexSim m_motorSimR = new SparkFlexSim(motorR,
    // m_elevatorGearbox);
    // private final SparkAbsoluteEncoder m_encoder = motorL.getAbsoluteEncoder();
    private final SparkAbsoluteEncoderSim m_encoderSim = new SparkAbsoluteEncoderSim(motorL);

    private final ElevatorSim m_elevatorSim = new ElevatorSim(
            m_elevatorGearbox,
            Constants.ElevatorSimConstants.kElevatorGearing,
            Constants.ElevatorSimConstants.kCarriageMass,
            Constants.ElevatorSimConstants.kElevatorSprocketDiameter,
            Constants.ElevatorSimConstants.kMinElevatorHeightMeters,
            Constants.ElevatorSimConstants.kMaxElevatorHeightMeters,
            true,
            0);

    // the main mechanism object
    // private Mechanism2d mech = new Mechanism2d(10, 100);
    // // the mechanism root node
    // private MechanismRoot2d root = mech.getRoot("bottom", 5, 0);

    // // private final FlywheelSim elevatorSimMotor = new
    // // FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004096955);

    // // MechanismLigament2d objects represent each "section"/"stage" of the
    // // mechanism, and are based
    // // off the root node or another ligament object
    // MechanismLigament2d elevator = root.append(new
    // MechanismLigament2d("elevator", lowestPoint, 90));
    // MechanismLigament2d bottom = elevator.append(
    // new MechanismLigament2d("bottom", 5, 0, 6, new
    // Color8Bit(Color.kBlanchedAlmond)));

    private final Mechanism2d m_mech2d = new Mechanism2d(8, 5);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 4, 0);
    private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(
            new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));

    public ElevatorSubsystem() {

        motorL.getEncoder().setPosition(5);
        motorR.getEncoder().setPosition(5);

        // post the mechanism to the dashboard
        // SmartDashboard.putData("Elevator", mech);
        // elevatorSimMotor.setInput(0);
        m_mech2d.setBackgroundColor(new Color8Bit(Color.kWhite));
        m_elevatorMech2d.setLineWeight(4.0);
        m_elevatorMech2d.setColor(new Color8Bit(Color.kBlue));
        SmartDashboard.putData("Elevator Sim", m_mech2d);
    }

    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_elevatorSim.setInput(m_motorSimL.getVelocity() * RobotController.getBatteryVoltage());
        System.out.println("velocity for motor is " + m_motorSimL.getVelocity());

        // Next, we update it. The standard loop time is 20ms.
        m_elevatorSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery
        // voltage
        m_encoderSim.setPosition(m_elevatorSim.getPositionMeters());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
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

        }

        motorL.set(calculatedPID);
        motorR.set(-calculatedPID); // ONE OF THEM IS NEGITIVE
    }

    private double calculatePid(double position) {
        // updatePivotAngle();
        elevatorCurrentPose = motorL.getEncoder().getPosition();
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
        return motorL.getEncoder().getVelocity();
    }

    public void resetElevatorPosition() {
        setpoint = 0;
    }

    public double getHeight() {
        return motorL.getAbsoluteEncoder().getPosition();
    }

    public boolean isAtSetpoint() {
        return elevator_PID.atSetpoint();
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
