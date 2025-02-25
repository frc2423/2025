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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.IntakeCommands;
//import frc.robot.subsystems.ArmSubsystem;

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
        ArmSubsystem arm = new ArmSubsystem();
        ClimberSubsystem climberSubsystem = new ClimberSubsystem();
        FunnelSubsystem funnelSubsystem = new FunnelSubsystem();
        IntakeCommands intakeCommands = new IntakeCommands(intakeSubsystem, funnelSubsystem);
        ElevatorSubsystem elevator = new ElevatorSubsystem(arm);

        SwerveCommands swerveCommands = new SwerveCommands(drivebase, elevator, intakeCommands);

        private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
        private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
        private static boolean runOnce = false;

        SendableChooser<String> m_chooser = new SendableChooser<>();

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
                configureDriverBindings();
                configureOperatorBindings();
                Command driveFieldOrientedAngularVelocity = getTeleopDriveCommand();
                drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
                DriverStation.silenceJoystickConnectionWarning(true);
                NamedCommands.registerCommand("test", Commands.print("I EXIST"));
                SmartDashboard.putData("elevatorSubsystem", elevator);
                SmartDashboard.putData("intakeSubsystewm", intakeSubsystem);
                SmartDashboard.putData("autoChooser", m_chooser);
                SmartDashboard.putData("swerveSubsystem", drivebase);
                SmartDashboard.putData("ArmSubsystem", arm);

                m_chooser.setDefaultOption("Middle Side Auto L2", "Middle Side Auto L2");
                m_chooser.addOption("Middle Side Auto L3", "Middle Side Auto L3");
                m_chooser.addOption("Middle Side Auto L4", "Middle Side Auto L4");
                m_chooser.addOption("Left Side 2 Piece Auto Left Reef", "Left Side 2 Piece Auto Left Reef");

                NamedCommands.registerCommand("Elevator to Reef L2",
                                elevator.goToSetpoint(Constants.SetpointConstants.REEF_L2));

                NamedCommands.registerCommand("Elevator to Reef L3",
                                elevator.goToSetpoint(Constants.SetpointConstants.REEF_L3));

                NamedCommands.registerCommand("Elevator to Reef L4",
                                elevator.goToSetpoint(Constants.SetpointConstants.REEF_L4));

                NamedCommands.registerCommand("Outtake Reef", intakeCommands.intakeOut());

                NamedCommands.registerCommand("AutoScoral Right",
                                swerveCommands.autoScoralClosest(Constants.SetpointConstants.REEF_L2, true));

                NamedCommands.registerCommand("AutoScoral Left",
                                swerveCommands.autoScoralClosest(Constants.SetpointConstants.REEF_L2, false));

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

        private void configureDriverBindings() {

                new Trigger(() -> {
                        boolean value = DriverStation.isDisabled() && RobotContainer.runOnce;
                        RobotContainer.runOnce = true;
                        return value;
                }).whileTrue(elevator.stopElevator().repeatedly().ignoringDisable(true));

                new JoystickButton(driverXbox, XboxController.Button.kStart.value)
                                .onTrue((new InstantCommand(drivebase::zeroGyro)));

                new JoystickButton(driverXbox, XboxController.Button.kLeftBumper.value)
                                .whileTrue(swerveCommands.autoScoralClosest(Constants.SetpointConstants.REEF_L2,
                                                false));

                new JoystickButton(driverXbox, XboxController.Button.kRightBumper.value)
                                .whileTrue(swerveCommands.autoScoralClosest(Constants.SetpointConstants.REEF_L2,
                                                true));

                new JoystickButton(driverXbox, XboxController.Button.kY.value)
                                .onTrue(intakeCommands.in());

                new JoystickButton(driverXbox, XboxController.Button.kA.value)
                                .onTrue(intakeCommands.intakeOut());

                new JoystickButton(driverXbox, XboxController.Button.kBack.value)
                                .onTrue(swerveCommands.lookAtNearestTag());

                new JoystickButton(driverXbox, XboxController.Button.kX.value)
                                .onTrue(intakeCommands.intakeOut());

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

                // new JoystickButton(driverXbox, XboxController.Button.kA.value)
                // .onTrue(elevator.goDown());
                new JoystickButton(driverXbox, XboxController.Button.kA.value)
                                .onTrue(elevator.goDown());

                new JoystickButton(driverXbox, XboxController.Button.kY.value)
                                .onTrue(elevator.goUp());

        }

        private void configureOperatorBindings() {

                new Trigger(() -> {
                        boolean value = DriverStation.isDisabled() && RobotContainer.runOnce;
                        RobotContainer.runOnce = true;
                        return value;
                }).whileTrue(elevator.stopElevator().repeatedly().ignoringDisable(true));

                new JoystickButton(operator, XboxController.Button.kA.value)
                                .whileTrue(climberSubsystem.climb());

                new JoystickButton(operator, XboxController.Button.kB.value)
                                .whileTrue(climberSubsystem.deClimb());

                new JoystickButton(operator, XboxController.Button.kY.value)
                                .onTrue(intakeCommands.in());

                new JoystickButton(operator, XboxController.Button.kX.value)
                                .onTrue(intakeCommands.intakeOut());

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
                                .onTrue(elevator.goDown());

                // new Trigger(() -> operator.getPOV() == 0).whileTrue(elevator.goUp());

                // .onTrue(elevator.goLittleDown(1));
                new JoystickButton(operator, XboxController.Button.kBack.value)
                                .onTrue(elevator.goToSetpoint(Constants.SetpointConstants.ALGAE_DESCORE_L3));

                // .onTrue(elevator.goLittleUp(1));

                new JoystickButton(operator, XboxController.Button.kRightBumper.value).onTrue(elevator.goLittleUp(1));
                new JoystickButton(operator, XboxController.Button.kLeftBumper.value).onTrue(elevator.goLittleDown(1));
                // .whileTrue(intakeCommands.intakeStop());

                // .whileTrue(swerveCommands.lookAtNearestTag());

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return drivebase.getAutonomousCommand(m_chooser.getSelected());
        }

        public void setDriveMode() {
                configureDriverBindings();
                configureOperatorBindings();
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }

}
