package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import au.grapplerobotics.LaserCan;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class IntakeSubsystem extends SubsystemBase {
    private SparkFlex motor = new SparkFlex(23, MotorType.kBrushless);
    private LaserCan intakeDist = new LaserCan(26);

    public IntakeSubsystem() {

    }

    public void intake(double speed) {
        motor.set(speed);
    }

    public void outtake(double speed) {
        motor.set(speed);
    }

    public void backwards(double speed) {
        motor.set(-speed);
    }

    public void stop() {
        motor.set(0);
    }

    public double distMm() {
        var dist = intakeDist.getMeasurement();
        if (dist == null) {
            return 10000;
        } else {
            return dist.distance_mm;
        }

    }

    public boolean isOut() {
        return distMm() > 100;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("laserCan distance", () -> distMm(), null);
    }
}