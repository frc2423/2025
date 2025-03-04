package frc.robot.subsystems.Elevator;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ElevatorSimulation {

    private final double ELEVATOR_GEARING = 4;
    private final double CARRIAGE_MASS = 5;
    private final double ELEVATOR_DRUM_RADIUS = Units.inchesToMeters(1);
    private final double MIN_ELEVATOR_HEIGHT_METERS = 0;
    private final double MAX_ELEVATOR_HEIGHT_METERS = 2;

    DCMotor flexGearbox = DCMotor.getNeoVortex(2);
    private SparkFlexSim motorSim;
    ElevatorSim elevatorSim = new ElevatorSim(
            flexGearbox,
            ELEVATOR_GEARING,
            CARRIAGE_MASS,
            ELEVATOR_DRUM_RADIUS,
            MIN_ELEVATOR_HEIGHT_METERS,
            MAX_ELEVATOR_HEIGHT_METERS,
            false,
            0,
            0,
            0);

    // the main mechanism object
    private Mechanism2d mech = new Mechanism2d(100, 100);
    // the mechanism root node
    private MechanismRoot2d root = mech.getRoot("bottom", 5, 0);

    // MechanismLigament2d objects represent each "section"/"stage" of the
    // mechanism, and are based
    // off the root node or another ligament object
    MechanismLigament2d elevator = root.append(new MechanismLigament2d("elevator", 50, 90));
    MechanismLigament2d bottom = elevator.append(
            new MechanismLigament2d("bottom", 5, 0, 6, new Color8Bit(Color.kBlanchedAlmond)));

    public ElevatorSimulation(SparkFlex motor) {
        motorSim = new SparkFlexSim(motor, flexGearbox);

        // post the mechanism to the dashboard
        SmartDashboard.putData("Mech2d", mech);

    }

    public void simPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        elevatorSim.setInput(motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        elevatorSim.update(0.020);

        // Now, we update the Spark Flex
        final double MAX_ENCODER_POSITION = 65;
        double percentHeight = elevatorSim.getPositionMeters() / MAX_ELEVATOR_HEIGHT_METERS;
        motorSim.setPosition(MAX_ENCODER_POSITION * percentHeight);
        motorSim.iterate(
                elevatorSim.getVelocityMetersPerSecond(),
                RobotController.getBatteryVoltage(),
                0.02);
    }

    public void periodic() {
        final double MAX_LENGTH_PIXELS = 50;
        double percentHeight = elevatorSim.getPositionMeters() / MAX_ELEVATOR_HEIGHT_METERS;
        double length = MAX_LENGTH_PIXELS * percentHeight;
        bottom.setLength(length);
    }

    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("simHeightMeters", () -> elevatorSim.getPositionMeters(), null);
    }

}
