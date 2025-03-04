package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class RobotTelemetry {
    private ElevatorSubsystem elevator;
    private ArmSubsystem arm;
    private Pose3d armPose;

    public RobotTelemetry(ElevatorSubsystem elevator, ArmSubsystem arm) {
        this.elevator = elevator;
        this.arm = arm;
    }

    public void update() {
        double height = (elevator.getEncoderPosition() / 65) * 2;
        double angle = (arm.getEncoderPosition() / 15) * Math.PI / 2;
        armPose = new Pose3d(0, 0, height, new Rotation3d(0, angle, 0));
        NTHelper.setPose3d("/advantageScopeModel/armPose", armPose);
    }
}
