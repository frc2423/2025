// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveCommands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

import java.io.File;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.IntakeCommands;
import frc.robot.subsystems.Claw.ClawSubsystem;
import frc.robot.subsystems.Claw.ClawCommands;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

        // Replace with CommandPS4Controller or CommandJoystick if needed
        // final CommandXboxController driverXbox = new CommandXboxController(0);
        XboxController driverXbox = new XboxController(0);
        XboxController operator = new XboxController(1);
        boolean isPanel = false;
        String deployDirectory = (Robot.isSimulation()) ? "sim-swerve/neo" : "swerve";
        // The robot's subsystems and commands are defined here...
        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), deployDirectory));

        IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

        IntakeCommands intakeCommands = new IntakeCommands(intakeSubsystem);
        ClawSubsystem clawSubsystem = new ClawSubsystem();

        ClawCommands clawCommands = new ClawCommands(clawSubsystem);
        SwerveCommands swerveCommands = new SwerveCommands(drivebase, elevator, intakeCommands);

        public static ElevatorSubsystem elevator = new ElevatorSubsystem();

        private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(0.5);
        private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(0.5);
        private static boolean runOnce = false;

        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled
         * by angular velocity.
         */
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driverXbox.getLeftY() * -1,
                        () -> driverXbox.getLeftX() * -1)
                        .withControllerRotationAxis(driverXbox::getRightX)
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        /**
         * Clone's the angular velocity input stream and converts it to a fieldRelative
         * input stream.
         */
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(driverXbox::getRightX,
                                        driverXbox::getRightY)
                        .headingWhile(true);

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the desired angle NOT angular rotation
        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the angular velocity of the robot
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

        Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

        SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> -driverXbox.getLeftY(),
                        () -> -driverXbox.getLeftX())
                        .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);
        // Derive the heading axis with math!
        SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
                        .withControllerHeadingAxis(() -> Math.sin(
                                        driverXbox.getRawAxis(
                                                        2) * Math.PI)
                                        * (Math.PI * 2),
                                        () -> Math.cos(
                                                        driverXbox.getRawAxis(
                                                                        2) * Math.PI)
                                                        *
                                                        (Math.PI * 2))
                        .headingWhile(true);

        Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

        Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
                configureBindings();
                Command driveFieldOrientedAngularVelocity = getTeleopDriveCommand();
                drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
                DriverStation.silenceJoystickConnectionWarning(true);
                NamedCommands.registerCommand("test", Commands.print("I EXIST"));
                SmartDashboard.putData("elevatorSubsystem", elevator);
                SmartDashboard.putData("intakeSubsystewm", intakeSubsystem);
                SmartDashboard.putData("swerveSubsystem", drivebase);

                // Logging callback for the active path, this is sent as a list of poses
                PathPlannerLogging.setLogActivePathCallback((poses) -> {
                        // Do whatever you want with the poses here
                        drivebase.getSwerveDrive().field.getObject("AutoAlignPath").setPoses(poses);
                });
        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary predicate, or via the
         * named factories in
         * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
         * for
         * {@link CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
         * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
         * Flight joysticks}.
         */

        private Command getTeleopDriveCommand() {
                Command driveFieldOrientedAngularVelocity = drivebase.driveCommand(
                                () -> {
                                        double y = MathUtil.applyDeadband(
                                                        driverXbox.getLeftY(),
                                                        OperatorConstants.LEFT_Y_DEADBAND);
                                        if (PoseTransformUtils.isRedAlliance()) {
                                                y *= -1;
                                        }
                                        return m_yspeedLimiter.calculate(y);
                                },
                                () -> {
                                        double x = MathUtil.applyDeadband(
                                                        driverXbox.getLeftX(),
                                                        OperatorConstants.LEFT_X_DEADBAND);
                                        if (PoseTransformUtils.isRedAlliance()) {
                                                x *= -1;
                                        }
                                        return m_xspeedLimiter.calculate(x);
                                },
                                () -> -driverXbox.getRightX());
                return driveFieldOrientedAngularVelocity; // :P
        }

        private void configureBindings() {

                new Trigger(() -> {
                        boolean value = DriverStation.isDisabled() && RobotContainer.runOnce;
                        RobotContainer.runOnce = true;
                        return value;
                }).whileTrue(elevator.stopElevator().repeatedly().ignoringDisable(true));

                new JoystickButton(driverXbox, XboxController.Button.kStart.value)
                                .onTrue((new InstantCommand(drivebase::zeroGyro)));

                // new JoystickButton(driverXbox, XboxController.Button.kA.value)
                // .onTrue(elevator.goDown());

                // new JoystickButton(driverXbox, XboxController.Button.kY.value)
                // .onTrue(elevator.goUp());
                new Trigger(() -> operator.getPOV() == 270)
                                .whileTrue(elevator.goToSetpoint((isPanel) ? Constants.SetpointConstants.REEF_L2
                                                : Constants.SetpointConstants.REEF_L2));
                new Trigger(() -> operator.getPOV() == 0)
                                .whileTrue(elevator.goToSetpoint((isPanel) ? Constants.SetpointConstants.REEF_L3
                                                : Constants.SetpointConstants.REEF_L3));
                new Trigger(() -> operator.getPOV() == 90)
                                .whileTrue(elevator.goToSetpoint((isPanel) ? Constants.SetpointConstants.REEF_L4
                                                : Constants.SetpointConstants.REEF_L4));
                new Trigger(() -> operator.getPOV() == 180)
                                .whileTrue(elevator.goToSetpoint((isPanel) ? Constants.SetpointConstants.ZERO
                                                : Constants.SetpointConstants.ZERO));

                // new Trigger(() -> operator.getPOV() == 0).whileTrue(elevator.goUp());
                new JoystickButton(driverXbox, XboxController.Button.kLeftBumper.value)
                                .whileTrue(swerveCommands.autoScoralClosest(Constants.SetpointConstants.REEF_L2,
                                                false));
                // .onTrue(elevator.goLittleDown(1));
                new JoystickButton(operator, XboxController.Button.kBack.value)
                                .onTrue(elevator.goToSetpoint(Constants.SetpointConstants.ALGAE_DESCORE_L3));
                new JoystickButton(driverXbox, XboxController.Button.kRightBumper.value)
                                .whileTrue(swerveCommands.autoScoralClosest(Constants.SetpointConstants.REEF_L2,
                                                true));
                // .onTrue(elevator.goLittleUp(1));

                new JoystickButton(driverXbox, XboxController.Button.kY.value)
                                .onTrue(intakeCommands.intakeIn());

                new JoystickButton(driverXbox, XboxController.Button.kBack.value)
                                .onTrue(swerveCommands.lookAtNearestTag());

                new JoystickButton(driverXbox, XboxController.Button.kX.value)
                                .onTrue(intakeCommands.intakeOut());

                // new JoystickButton(driverXbox, XboxController.Button.kRightBumper.value)
                // .whileTrue(intakeCommands.intakeStop());

                new JoystickButton(driverXbox, XboxController.Button.kA.value)
                                .whileTrue(clawCommands.clawRelease());
                // .whileTrue(swerveCommands.lookAtNearestTag());

                new JoystickButton(driverXbox, XboxController.Button.kB.value)
                                .whileTrue(clawCommands.clawStop());

                new Trigger(() -> driverXbox.getPOV() == 0)
                                .onTrue(swerveCommands.lookAtAngle(0));
                new Trigger(() -> driverXbox.getPOV() == 315)
                                .onTrue(swerveCommands.lookAtAngle(60));
                new Trigger(() -> driverXbox.getPOV() == 225)
                                .onTrue(swerveCommands.lookAtAngle(120));
                new Trigger(() -> driverXbox.getPOV() == 180)
                                .onTrue(swerveCommands.lookAtAngle(180));
                new Trigger(() -> driverXbox.getPOV() == 135)
                                .onTrue(swerveCommands.lookAtAngle(240));
                new Trigger(() -> driverXbox.getPOV() == 45)
                                .onTrue(swerveCommands.lookAtAngle(300));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return drivebase.getAutonomousCommand("New Auto");
        }

        public void setDriveMode() {
                configureBindings();
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }

}
