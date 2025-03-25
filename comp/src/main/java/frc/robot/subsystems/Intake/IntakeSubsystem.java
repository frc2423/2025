package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import au.grapplerobotics.LaserCan;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class IntakeSubsystem extends SubsystemBase {
    private SparkFlex motor = new SparkFlex(23, MotorType.kBrushless);
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    // private boolean hasAlgae = false;
    private LaserCan intakeDist = new LaserCan(26);
    private double speed = 0;
    MedianFilter distanceFilter = new MedianFilter(10);
    private double distMm;
    private boolean ejecting = false;

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
         * O fine tune algae descore w/ abs encoder
         * O make autoalign and descore
         * O try barge
         * O try processor
         */
    }

    private void setCurrentLimit(int limit, int free) {
        motorConfig.smartCurrentLimit(limit, free);
        motor.configureAsync(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public IntakeSubsystem() {
        setCurrentLimit(80, 80);
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

    public double getRawSensorValue() {
        var dist = intakeDist.getMeasurement();
        if (dist == null || intakeDist.getMeasurement().status == intakeDist.LASERCAN_STATUS_OUT_OF_BOUNDS) {
            return 10000;
        } else {
            return dist.distance_mm;
        }
    }

    public boolean isOut() {
        return getRawSensorValue() > 60;
    }

    public boolean hasAlgae() {
        return distMm() > 95 && distMm() < 160;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("laserCan distance", () -> distMm(), null);
        builder.addDoubleProperty("current current (haha)", () -> motor.getOutputCurrent(), null);
        builder.addDoubleProperty("speed", () -> motor.get(), null);
        builder.addBooleanProperty("hasAlgae", () -> hasAlgae(), null);
        builder.addBooleanProperty("hasCoral", () -> !isOut(), null);
    }
}