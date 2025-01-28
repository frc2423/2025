package frc.robot.subsystems.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.PoseTransformUtils;

public class SwerveCommands {

    private SwerveSubsystem swerve;
    private XboxController driverXbox = new XboxController(0);
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(7);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(7);

    public SwerveCommands(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    public Command autoAlign(Pose2d pose) {

        // Since we are using a holonomic drivetrain, the rotation component of this
        // pose
        // represents the goal holonomic rotation
        Pose2d targetPose = PoseTransformUtils.transformXRedPose(pose);

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                0.5, 1.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0 // Goal end velocity in meters/sec
        );// Rotation delay distance in meters. This is how far the robot should travel
          // before attempting to rotate.

        pathfindingCommand.setName("Align to Pose");

        return pathfindingCommand;
    }

    public Command lookAtAngle(double angle) {
        var command = Commands.run(() -> {
            actuallyLookAngleButMove(Rotation2d.fromDegrees(angle));
        }).until(() -> (driverXbox.getRightX() > .1) || (driverXbox.getRightY() > .1));
        command.addRequirements(swerve);
        return command;
    }

    public void actuallyLookAngleButMove(Rotation2d rotation2d) { // here
        // var swerveDrive = swerve.getSwerveDrive();
        final double maxRadsPerSecond = 5;
        // final double minRadsPerSecond = .25; // hella slow
        // double slowRange = 10;
        // double t = (swerveDrive.getYaw().getDegrees() + slowRange) / (slowRange * 2);

        double x = MathUtil.applyDeadband(
                driverXbox.getLeftX(),
                OperatorConstants.LEFT_X_DEADBAND);
        if (PoseTransformUtils.isRedAlliance()) {
            x *= -1;
        }
        double ySpeedTarget = m_xspeedLimiter.calculate(x);

        double y = MathUtil.applyDeadband(
                driverXbox.getLeftY(),
                OperatorConstants.LEFT_Y_DEADBAND);
        if (PoseTransformUtils.isRedAlliance()) {
            y *= -1;
        }
        double xSpeedTarget = m_yspeedLimiter.calculate(y);

        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(xSpeedTarget, ySpeedTarget,
                rotation2d);

        // double high = slowRange + rotation2d.getDegrees();
        // double low = rotation2d.getDegrees() - slowRange;

        // if (swerveDrive.getYaw().getDegrees() < high &&
        // swerveDrive.getYaw().getDegrees() > low) {
        // desiredSpeeds.omegaRadiansPerSecond = MathUtil.interpolate(-maxRadsPerSecond,
        // maxRadsPerSecond, t);
        // } else if (Math.abs(desiredSpeeds.omegaRadiansPerSecond) > maxRadsPerSecond)
        // {
        // desiredSpeeds.omegaRadiansPerSecond = Math.copySign(maxRadsPerSecond,
        // desiredSpeeds.omegaRadiansPerSecond);
        // } else if (Math.abs(desiredSpeeds.omegaRadiansPerSecond) < minRadsPerSecond)
        // {
        // desiredSpeeds.omegaRadiansPerSecond = Math.copySign(minRadsPerSecond,
        // desiredSpeeds.omegaRadiansPerSecond);
        // }

        if (Math.abs(desiredSpeeds.omegaRadiansPerSecond) > maxRadsPerSecond) {
            desiredSpeeds.omegaRadiansPerSecond = Math.copySign(maxRadsPerSecond, desiredSpeeds.omegaRadiansPerSecond);
        }

        // double dead = 1; // band

        // if (Math.abs(swerveDrive.getYaw().getDegrees() - rotation2d.getDegrees()) <
        // dead) {
        // desiredSpeeds.omegaRadiansPerSecond = 0;
        // }

        swerve.driveFieldOriented(desiredSpeeds);
    }
}
