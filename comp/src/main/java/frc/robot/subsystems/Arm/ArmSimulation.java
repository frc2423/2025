package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
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

public class ArmSimulation {
    DCMotor maxGearbox = DCMotor.getNeoVortex(2);
    private SparkMaxSim motorSim;

    private final double ARM_GEARING = 4;
    private final double CARRIAGE_MASS = 5;
    private final double ARM_DRUM_RADIUS = Units.inchesToMeters(1);
    private final double MIN_ARM_ROTATION = 0;
    private final double MAX_ARM_ROTATION = 2;

    SingleJointedArmSim armSim = new SingleJointedArmSim(
            maxGearbox,
            ARM_GEARING,
            CARRIAGE_MASS,
            ARM_DRUM_RADIUS,
            MIN_ARM_ROTATION,
            MAX_ARM_ROTATION,
            false,
            0,
            0,
            0);

    private Mechanism2d mech = new Mechanism2d(100, 100);

    private MechanismRoot2d root = mech.getRoot("bottom", 5, 0);

    // MechanismLigament2d objects represent each "section"/"stage" of the
    // mechanism, and are based
    // off the root node or another ligament object
    MechanismLigament2d arm = root
            .append(new MechanismLigament2d("arm", 15, 75, 6, new Color8Bit(Color.kBlanchedAlmond)));

    public ArmSimulation(SparkMax motor) {
        motorSim = new SparkMaxSim(motor, maxGearbox);

        // post the mechanism to the dashboard
        SmartDashboard.putData("Mech2d", mech);
    }

    public void simPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        armSim.setInput(motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        armSim.update(0.020);

        // Now, we update the Spark Flex
        final double MAX_ENCODER_POSITION = 65;// read what encoder value on dashboard
        double percentRads = armSim.getAngleRads() / MAX_ARM_ROTATION;
        motorSim.setPosition(MAX_ENCODER_POSITION * percentRads);
        motorSim.iterate(
                armSim.getVelocityRadPerSec(),
                RobotController.getBatteryVoltage(),
                0.02);
    }

    public void periodic() {
        final double MAX_RAD_PIXELS = .5 * Math.PI;
        double percentRads = armSim.getAngleRads() / MAX_ARM_ROTATION;
        double rads = MAX_RAD_PIXELS * percentRads;
        arm.setAngle(Math.toDegrees(rads));
    }

    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("simAngle", () -> armSim.getAngleRads(), null);
    }
}
