package frc.robot.subsystems.Intake;

import com.fasterxml.jackson.annotation.JsonTypeInfo.None;
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

    public double distMm() {
        return distMm;
    }

    public int getRawSensorValue() {
        var dist = intakeDist.getMeasurement();
        if (dist == null || dist.status != LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return 10000;
        } else {
            return dist.distance_mm;
        }
    }

    public boolean isStalled() {
        var currentSpeed = motor.getEncoder().getVelocity();
        var desiredSpeed = Math.abs(motor.get());

        if (currentSpeed <= 1 && desiredSpeed >= 0) {
            return true; // Stalled
        } else {
            return false; // Not stalled
        }
    }

    public boolean isOut() {
        return getRawSensorValue() > 60;
    }

    public boolean isOutMedianFilter() {
        return distMm() > 60;
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
        builder.addBooleanProperty("isStalled", () -> isStalled(), null);

        if (Robot.isSimulation()) {
            builder.addBooleanProperty("simulateHasCoral", () -> !isOut(), value -> {
                ((MockLaserCan) intakeDist)
                        .setMeasurementPartialSim(LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT,
                                value ? 20 : 1000, 0);
            });
        }
    }
}