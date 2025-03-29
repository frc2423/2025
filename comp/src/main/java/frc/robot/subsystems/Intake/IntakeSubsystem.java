package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.simulation.MockLaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class IntakeSubsystem extends SubsystemBase {
    private SparkFlex motor = new SparkFlex(23, MotorType.kBrushless);
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    // private boolean hasAlgae = false;
    private final LaserCanInterface intakeDist;
    private double speed = 0;
    MedianFilter distanceFilter = new MedianFilter(10);
    private double distMm;
    private boolean ejecting = false;

    public IntakeSubsystem() {
        setCurrentLimit(60, 60);
        intakeDist = Robot.isSimulation() ? new MockLaserCan() : new LaserCan(26);
    }

    @Override
    public void simulationPeriodic() {

    }

    @Override
    public void periodic() {
        if (hasAlgae() && !ejecting) {
            motor.set(speed / 8);
        } else {
            motor.set(speed);
        }

        distMm = distanceFilter.calculate(getRawSensorValue());
        // if (motor.getOutputCurrent() > 60) {
        // motor.set(speed / 8);
        // hasAlgae = true;
        // } else {
        // motor.set(speed);
        // hasAlgae = false;
        // }

        /*
         * to do:
         * 0 try distance
         * 0 make distance work
         * 0 fine tune algae descore w/ abs encoder
         * 0 make autoalign and descore
         * 0 try barge
         * 0 try processor
         */
    }

    private void setCurrentLimit(int limit, int free) {
        motorConfig.smartCurrentLimit(limit, free);
        motor.configureAsync(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        // try {
        // intakeDist.setRegionOfInterest(new RegionOfInterest(8, 8, 8, 8));
        // } catch (ConfigurationFailedException exception) {
        // exception.printStackTrace();
        // }
    }

    public void intake(double speed) {
        this.speed = speed;
    }

    public void outtake(double speed) {
        this.speed = speed;
    }

    public void outtakeAlgae(double speed) {
        this.speed = -speed;
        ejecting = true;
    }

    public void backwards(double speed) {
        this.speed = -speed;
    }

    public void stop() {
        speed = 0;
        ejecting = false;
    }

    public boolean haveCoral() {
        if (distMm() < 100) {
            return true;
        } else {
            return false;
        }
    }

    public double distMm() {
        return distMm;
    }

    public int getRawSensorValue() {
        var dist = intakeDist.getMeasurement();
        if (dist == null || intakeDist.getMeasurement().status != LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return 10000;
        } else {
            return dist.distance_mm;
        }
    }

    public boolean isOut() {
        return getRawSensorValue() > 60;
    }

    public boolean hasAlgae() {
        return distMm() > 95 && distMm() < 110;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("laserCan distance", () -> distMm(), null);
        builder.addDoubleProperty("laserCan Raw Value", () -> getRawSensorValue(), null);
        builder.addDoubleProperty("current current (haha)", () -> motor.getOutputCurrent(), null);
        builder.addDoubleProperty("speed", () -> motor.get(), null);
        builder.addBooleanProperty("hasAlgae", () -> hasAlgae(), null);
        builder.addBooleanProperty("hasCoral", () -> !isOut(), null);

        if (Robot.isSimulation()) {
            builder.addBooleanProperty("simulateHasCoral", this::haveCoral, value -> {
                ((MockLaserCan) intakeDist)
                        .setMeasurementPartialSim(LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT,
                                value ? 50 : 1000, 0);
            });
        }
    }
}