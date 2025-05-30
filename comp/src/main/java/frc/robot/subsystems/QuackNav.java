package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.QuestNav;
import frc.robot.Robot;

public class QuackNav {
    public QuestNav questNav = new QuestNav();
    private boolean questInitialPose = false;

    private Pose2d questPoseAtReset = Pose2d.kZero;
    private Pose2d questRelativePoseAtReset = Pose2d.kZero;

    public static final Transform2d QUEST_TO_ROBOT = new Transform2d(Units.inchesToMeters(-2.591),
            Units.inchesToMeters(-8.013), Rotation2d.k180deg);

    public QuackNav() {
        SmartDashboard.putData("/QuackNavCommands/zeroAngle", zeroAngle());
        SmartDashboard.putData("/QuackNavCommands/zeroPose", zeroPose());
        SmartDashboard.putData("/QuackNavCommands/randomizePose", randomizePose());
        SmartDashboard.putData("/QuackNavCommands/clearInitialPose", Commands.runOnce(() -> {
            clearInitialPose();
        }));
    }

    private Command zeroAngle() {
        return Commands.runOnce(() -> {
            Pose2d pose = new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(Math.random() * .001));
            updateQuestPose(pose);
        });
    }

    public void zeroPose(Pose2d pose) {
        updateQuestPose(pose);
    }

    private Command zeroPose() {
        return Commands.runOnce(() -> {
            updateQuestPose(new Pose2d(Math.random() * .001, Math.random() * .001, Rotation2d.kZero));
        });
    }

    private Command randomizePose() {
        return Commands.runOnce(() -> {
            System.out.println("RANDOMIZE POSE");
            updateQuestPose(
                    new Pose2d(Math.random() * 10, Math.random() * 5, Rotation2d.fromDegrees(Math.random() * 360)));
        });
    }

    public void periodic() {
        questNav.cleanupResponses();
        questNav.processHeartbeat();
    }

    public boolean isQuestMode() {
        if (Robot.isSimulation()) {
            return questNav.getConnected() && hasInitialPose() && questNav.getTrackingStatus();
        }
        return questNav.getConnected() && hasInitialPose() && questNav.getTrackingStatus();
    }

    public double getTimestamp() {
        return questNav.getTimestamp();
    }

    public Pose2d getPose() {
        // Get the Quest pose
        Pose2d relativeToReset = questNav.getPose().relativeTo(questRelativePoseAtReset);
        Pose2d questPose = questPoseAtReset
                .plus(new Transform2d(relativeToReset.getTranslation(), relativeToReset.getRotation()));
        Pose2d robotPose = questPose.plus(QUEST_TO_ROBOT);

        return robotPose;
    }

    public boolean hasInitialPose() {
        return questInitialPose;
    }

    public void clearInitialPose() {
        questInitialPose = false;
    }

    public boolean isConnected() {
        return questNav.getConnected();
    }

    public double getBatteryPercent() {
        return questNav.getBatteryPercent();
    }

    public boolean getTrackingStatus() {
        return questNav.getTrackingStatus();
    }

    public void updateQuestPose(Pose2d pose) {
        updateQuestPose(pose, VecBuilder.fill(0, 0, 0));
    }

    public void updateQuestPose(Pose2d pose, Matrix<N3, N1> stdDev) {
        boolean isTrustworthy = false;
        if ((stdDev.get(0, 0) < 2.5 && stdDev.get(1, 0) < 2.5) && stdDev.get(2, 0) < 10)
            isTrustworthy = true;

        if (isTrustworthy) {
            questInitialPose = true;
            questPoseAtReset = pose.plus(QUEST_TO_ROBOT.inverse());
            questRelativePoseAtReset = questNav.getPose();
        }
    }
}
