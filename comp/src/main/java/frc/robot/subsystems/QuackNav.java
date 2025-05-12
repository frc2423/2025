package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.QuestNav;
import frc.robot.Robot;

public class QuackNav {
    public QuestNav questNav = new QuestNav();

    public QuackNav() {
        SmartDashboard.putData("/QuackNavCommands/zeroAngle", zeroAngle());
        SmartDashboard.putData("/QuackNavCommands/zeroPose", zeroPose());
        SmartDashboard.putData("/QuackNavCommands/randomizePose", randomizePose());
    }

    private Command zeroAngle() {
        return Commands.runOnce(() -> {
            Pose2d pose = new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(Math.random() * .001));
            updateQuestPose(pose);
        });
    }

    private Command zeroPose() {
        return Commands.runOnce(() -> {
            System.out.println("ZERO POSE");
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
            return questNav.getConnected() && questNav.hasInitialPose() && questNav.getTrackingStatus();
        }
        return questNav.getConnected() && questNav.hasInitialPose() && questNav.getTrackingStatus();
    }

    public double getTimestamp() {
        return questNav.getTimestamp();
    }

    public Pose2d getPose() {
        // Get the Quest pose
        Pose2d questPose = questNav.getPose();

        if (Robot.isSimulation()) {
            return questPose;
        }

        // Transform by the offset to get your final pose!
        Pose2d robotPose = questPose.transformBy(Constants.QuestNavConstants.QUEST_TO_ROBOT.inverse());

        return robotPose;
    }

    public boolean hasInitialPose() {
        return questNav.hasInitialPose();
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
            questNav.setPose(
                    Robot.isSimulation() ? pose : pose.transformBy(Constants.QuestNavConstants.QUEST_TO_ROBOT));
        }
    }
}
