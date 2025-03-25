package frc.robot.subsystems.Arm;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private double encoderPosition = 0;
    private double maximum = 0.917;
    private double minumum = 0.7;
    private double setpoint = maximum;// will change varibly
    private final ArmFeedforward m_feedforward = new ArmFeedforward(0, 0.455, 0, 0);
    private double MAX_VOLTAGE = 0.9;

    double calculatedPID = 0;

    private ArmSimulation armSim = new ArmSimulation(armPivot);

    ProfiledPIDController arm_PID = new ProfiledPIDController(70, 0, 0,
            new TrapezoidProfile.Constraints(4, 6));

    public ArmSubsystem() {
        // armPivot.getEncoder().setPosition(0);
        SmartDashboard.putData("arm_PID", arm_PID);
    }

    @Override
    public void periodic() {
        encoderPosition = armPivot.getAbsoluteEncoder().getPosition();
        calculatedPID = calculatePid(setpoint);

        if (calculatedPID > MAX_VOLTAGE) {
            calculatedPID = MAX_VOLTAGE;
        } else if (calculatedPID < -MAX_VOLTAGE) {
            calculatedPID = -MAX_VOLTAGE;
        }

        if (encoderPosition > maximum) {
            calculatedPID = Math.max(calculatedPID, 0);
        } else if (encoderPosition < minumum) {
            calculatedPID = Math.min(calculatedPID, 0);
        }

        armPivot.set(calculatedPID);
        // scoringWheelMotor.set(scoringWheelSpeed);
    }

    @Override
    public void simulationPeriodic() {
        armSim.simPeriodic();
    }

    private double calculatePid(double position) {
        // updatePivotAngle();
        double pid = arm_PID.calculate(encoderPosition, position);
        var setpoint = arm_PID.getSetpoint();

        double feedforward = m_feedforward.calculate(getArmAngle(), 0);
        return (feedforward + pid) / RobotController.getBatteryVoltage(); // +pid
    }

    public Command goDown() { // for manual control, sick
        return goToSetpoint(minumum);
    }

    public double getArmAngle() {
        return (encoderPosition - 0.64) * (2 * Math.PI);
    }

    public Command goUp() { // for manual control, sick
        return goToSetpoint(maximum);
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

    public Command goScoreL4() {
        return goToSetpoint(Constants.ArmConstants.L4_SCORING_POSITION);
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
            setSetpoint(encoderPosition);
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

    public boolean isAtSetpoint() {
        return (Math.abs(getEncoderPosition() - setpoint) < (2.0 / 78.0));
    }

    public double getEncoderPosition() {
        return encoderPosition;
    }

    public boolean isInSafeArea() {
        if (encoderPosition < 0.888900 && encoderPosition > 0.840048) {// 0.888900 and 0.840048
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
        builder.addDoubleProperty("absoluteEncoderPosition", () -> encoderPosition, null);
        builder.addDoubleProperty("setpoint", () -> setpoint, null);
        builder.addDoubleProperty("scoringWheelSpeed", () -> scoringWheelSpeed, null);
        builder.addBooleanProperty("isInSafeArea", () -> isInSafeArea(), null);
        builder.addDoubleProperty("encoder", () -> armPivot.getEncoder().getPosition(), null);
        builder.addDoubleProperty("getArmAngle", () -> getArmAngle(), null);

        if (Robot.isSimulation()) {
            armSim.initSendable(builder);
        }
    }
}
