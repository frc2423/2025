package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.QuestNav;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

public class QuackNav {
    public QuestNav questNav = new QuestNav();

    public QuackNav() {
    }

    public void periodic() {
        questNav.cleanupResponses();
        questNav.processHeartbeat();
    }

    public boolean isQuestMode() {
        return questNav.getConnected() && questNav.hasInitialPose() && questNav.getTrackingStatus();
    }

    public double getTimestamp() {
        return questNav.getTimestamp();
    }

    public Pose2d getPose() {
        // Get the Quest pose
        Pose2d questPose = questNav.getPose();

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

    public void updateQuestPose(Pose2d pose, Matrix<N3, N1> stdDev) {
        boolean isTrustworthy = false;
        if ((stdDev.get(0, 0) < 2.5 && stdDev.get(1, 0) < 2.5) && stdDev.get(2, 0) < 10)
            isTrustworthy = true;

        if (isTrustworthy) {
            questNav.setPose(pose.transformBy(Constants.QuestNavConstants.QUEST_TO_ROBOT));
        }
        // if (isInRange())
    }
}
