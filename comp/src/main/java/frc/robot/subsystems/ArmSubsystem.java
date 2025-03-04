package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
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

public class ArmSubsystem extends SubsystemBase {

    private SparkMax armPivot = new SparkMax(25, MotorType.kBrushless);
    // private SparkFlex scoringWheelMotor = new SparkFlex(23,
    // MotorType.kBrushless);
    private double scoringWheelSpeed = 0;
    private double armCurrentPose = 0;
    private double maximum = 0; // some value
    private double minumum = -13.8; // some value
    private double setpoint = 0;// will change varibly
    private final ArmFeedforward m_feedforward = new ArmFeedforward(0, 0, 0, 0);
    private double MAX_VOLTAGE = 0.9;

    double calculatedPID = 0;

    ProfiledPIDController arm_PID = new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(100, 100));

    public ArmSubsystem() {
        armPivot.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        armCurrentPose = armPivot.getEncoder().getPosition();
        calculatedPID = calculatePid(setpoint);

        if (calculatedPID > MAX_VOLTAGE) {
            calculatedPID = MAX_VOLTAGE;
        } else if (calculatedPID < -MAX_VOLTAGE) {
            calculatedPID = -MAX_VOLTAGE;
        }

        if (armCurrentPose > maximum) {
            calculatedPID = Math.min(calculatedPID, 0);
        } else if (armCurrentPose < minumum) {
            calculatedPID = Math.max(calculatedPID, 0);
        }

        if (Robot.isSimulation()) {

        }

        armPivot.set(calculatedPID);
        // scoringWheelMotor.set(scoringWheelSpeed);

    }

    private double calculatePid(double position) {
        // updatePivotAngle();
        double pid = arm_PID.calculate(armCurrentPose, position);
        var setpoint = arm_PID.getSetpoint();

        double feedforward = m_feedforward.calculate(setpoint.velocity, 0);
        return (feedforward + pid) / RobotController.getBatteryVoltage(); // +pid
    }

    public Command goDown() { // for manual control, sick
        return goToSetpoint(minumum);
    }

    public Command goUp() { // for manual control, sick
        return goToSetpoint(maximum);
    }

    public Command goScore() {
        return goToSetpoint(Constants.ArmConstants.SCORING_POSITION);
    }

    public Command goToSetpoint(double position) {
        return runOnce(() -> {
            setSetpoint(position);
        });
    }

    public Command stopElevator() {
        Command command = runOnce(() -> {
            setSetpoint(armCurrentPose);
        });
        command.setName("stop elevator");
        return command;
    }

    public Command setScoringWheelSpeed(double speed) {
        Command command = runOnce(() -> {
            scoringWheelSpeed = speed;
        });
        command.setName("Set scoring wheel speed");
        return command;
    }

    public Command stop() {
        Command command = Commands.repeatingSequence(stopElevator(), setScoringWheelSpeed(0));
        command.setName("Stop Arm");
        return command;
    }

    private void setSetpoint(double position) {
        if (position <= maximum && position >= minumum) {
            setpoint = position;
        }
    }

    public double getCurrentArmPose() {
        return armCurrentPose;
    }

    public boolean isInSafeArea() {
        if (armCurrentPose < -1.9 && armCurrentPose > -5.3) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // This is used to add things to NetworkTables
        super.initSendable(builder);

        builder.addDoubleProperty("calculatePid", () -> calculatedPID, null);
        builder.addDoubleProperty("currentPose", () -> armCurrentPose, null);
        builder.addDoubleProperty("setpoint", () -> setpoint, null);
        builder.addDoubleProperty("scoringWheelSpeed", () -> scoringWheelSpeed, null);
        builder.addBooleanProperty("isInSafeArea", () -> isInSafeArea(), null);

    }
}
