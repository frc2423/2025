// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.AutoAlign;
import frc.robot.subsystems.swervedrive.AutoCommand;
import frc.robot.subsystems.swervedrive.SwerveCommands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

import java.io.File;
import java.util.Optional;

import swervelib.SwerveInputStream;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.LED.KwarqsLed;
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
        String deployDirectory = (Robot.isSimulation()) ? "sim-swerve/neo" : "swerve";
        // The robot's subsystems and commands are defined here...
        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), deployDirectory));

        IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        ArmSubsystem arm = new ArmSubsystem();
        ClimberSubsystem climberSubsystem = new ClimberSubsystem();
        FunnelSubsystem funnelSubsystem = new FunnelSubsystem();
        IntakeCommands intakeCommands = new IntakeCommands(intakeSubsystem, funnelSubsystem);
        ElevatorSubsystem elevator = new ElevatorSubsystem(arm, intakeCommands);
        RobotTelemetry robotTelemetry = new RobotTelemetry(elevator, arm);

        SwerveCommands swerveCommands = new SwerveCommands(drivebase, elevator, intakeCommands, arm, intakeSubsystem);

        KwarqsLed ledKwarqs = new KwarqsLed(swerveCommands.getVisionFromSwerve(), driverXbox);

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
                SmartDashboard.putData("ClimberSubsystem", climberSubsystem);
                SmartDashboard.putData("ledSubsystem", ledKwarqs);

                // m_chooser.setDefaultOption("Middle Side Auto L2", "Middle Side Auto L2");
                // m_chooser.addOption("Middle Side Auto L3", "Middle Side Auto L3");
                // m_chooser.addOption("Middle Side Auto L4", "Middle Side Auto L4");
                m_chooser.addOption("Left Side 2 Piece RED I & K", "Left Side 2 Piece RED I & K");
                m_chooser.addOption("Right Side 2 Piece RED E & D", "Right Side 2 Piece RED E & D");
                m_chooser.addOption("Back Right Single (Robot Oriented)", "Back Right Single (Robot Oriented)");
                m_chooser.addOption("Back Left Single (robot oriented)", "Back Left Single (robot oriented)");
                m_chooser.addOption("Left Side 2 3/4", "Left Side 3 Piece BLUE E & C & D");
                // m_chooser.addOption("Right Side 3 BLUE", "Right Side 3 Piece BLUE E & C &
                // D");
                // m_chooser.addOption("Left Side 3 BLUE", "Left Side 3 Piece BLUE J & K & L");
                m_chooser.addOption("Left Side 3 RED", "Left Side 3 Piece RED J & K & L");
                m_chooser.addOption("Right Side 3 RED", "Right Side 3 Piece RED E & C & D");

                // m_chooser.addOption("Right Side 2 Piece BLUE E & C", "Right Side 2 Piece BLUE
                // E & C"); //BAD DONT USE
                // m_chooser.addOption("Left 2 Piece I & L BLUE", "Left 2 Piece I & L BLUE");
                // //BAD DONT USE

                NamedCommands.registerCommand("stopMoving", swerveCommands.stopMoving());

                NamedCommands.registerCommand("Elevator to Reef L2",
                                elevator.goToSetpoint(Constants.SetpointConstants.REEF_L2));

                NamedCommands.registerCommand("Elevator to Reef L3",
                                elevator.goToSetpoint(Constants.SetpointConstants.REEF_L3));

                NamedCommands.registerCommand("Elevator to Reef L4",
                                elevator.goToSetpoint(Constants.SetpointConstants.REEF_L4));

                NamedCommands.registerCommand("Outtake Reef", intakeCommands.intakeOut());

                NamedCommands.registerCommand("AutoScoral 11 Right", swerveCommands
                                .autoScoral(Optional.of(11), Constants.SetpointConstants.REEF_L4, true));

                NamedCommands.registerCommand("AutoScoral 11 Left", swerveCommands
                                .autoScoral(Optional.of(11), Constants.SetpointConstants.REEF_L4, false));

                NamedCommands.registerCommand("AutoScoral Left", Commands.either(swerveCommands
                                .autoScoral(Optional.of(11), Constants.SetpointConstants.REEF_L4, false),
                                Commands.none(),
                                () -> PoseTransformUtils.isRedAlliance()));

                NamedCommands.registerCommand("AutoScoral 6 Right", swerveCommands.autoScoral(Optional.of(6),
                                Constants.SetpointConstants.REEF_L4, true));

                NamedCommands.registerCommand("AutoScoral 6 Left", swerveCommands.autoScoral(Optional.of(6),
                                Constants.SetpointConstants.REEF_L4, false));

                NamedCommands.registerCommand("AutoScoral 9 Left", swerveCommands.autoScoral(Optional.of(9),
                                Constants.SetpointConstants.REEF_L4, false));

                NamedCommands.registerCommand("AutoScoral 9 Right", swerveCommands.autoScoral(Optional.of(9),
                                Constants.SetpointConstants.REEF_L4, true));

                NamedCommands.registerCommand("AutoScoral 8 Right", swerveCommands.autoScoral(Optional.of(8),
                                Constants.SetpointConstants.REEF_L4, true));

                NamedCommands.registerCommand("AutoScoral 8 Left", swerveCommands.autoScoral(Optional.of(8),
                                Constants.SetpointConstants.REEF_L4, false));

                NamedCommands.registerCommand("AutoScoral 22 Left",
                                swerveCommands.autoScoral(Optional.of(22),
                                                Constants.SetpointConstants.REEF_L4, false));

                NamedCommands.registerCommand("AutoScoral 22 Right",
                                swerveCommands.autoScoral(Optional.of(22),
                                                Constants.SetpointConstants.REEF_L4, true));

                NamedCommands.registerCommand("AutoScoral 17 Right",
                                swerveCommands.autoScoral(Optional.of(17),
                                                Constants.SetpointConstants.REEF_L4, true));

                NamedCommands.registerCommand("AutoScoral 17 Left",
                                swerveCommands.autoScoral(Optional.of(17),
                                                Constants.SetpointConstants.REEF_L4, false));

                NamedCommands.registerCommand("AutoScoral 20 Left",
                                swerveCommands.autoScoral(Optional.of(20),
                                                Constants.SetpointConstants.REEF_L4, false));

                NamedCommands.registerCommand("AutoScoral 20 Right",
                                swerveCommands.autoScoral(Optional.of(20),
                                                Constants.SetpointConstants.REEF_L4, true));

                NamedCommands.registerCommand("AutoScoral 19 Right",
                                swerveCommands.autoScoral(Optional.of(19),
                                                Constants.SetpointConstants.REEF_L4, true));

                NamedCommands.registerCommand("AutoScoral 19 Left",
                                swerveCommands.autoScoral(Optional.of(19),
                                                Constants.SetpointConstants.REEF_L4, false));

                NamedCommands.registerCommand("AutoScoral 11 Right",
                                swerveCommands.autoScoral(Optional.of(11),
                                                Constants.SetpointConstants.REEF_L4, true));

                NamedCommands.registerCommand("AutoScoral 6 Right",
                                swerveCommands.autoScoral(Optional.of(6),
                                                Constants.SetpointConstants.REEF_L4, true));

                NamedCommands.registerCommand("AutoScoral 6 Left",
                                swerveCommands.autoScoral(Optional.of(6),
                                                Constants.SetpointConstants.REEF_L4, false));

                NamedCommands.registerCommand("Intake Start",
                                intakeCommands.intakeStart());

                NamedCommands.registerCommand("Funnel Start",
                                funnelSubsystem.funnelStart());

                NamedCommands.registerCommand("Run Intake Short", intakeCommands.intakeShort());

                NamedCommands.registerCommand("Run Intake", intakeCommands.intakeHumanPlayer());

                addAutoScoreCommand("AutoScoral left near", 11, 20, false);
                addAutoScoreCommand("AutoScoral 11/20 right", 11, 20, true);
                addAutoScoreCommand("AutoScoral left far", 6, 19, true);
                addAutoScoreCommand("AutoScoral 6/19 left", 6, 19, false);
                addAutoScoreCommand("AutoScoral right near", 9, 22, false);
                addAutoScoreCommand("AutoScoral 9/22 right", 9, 22, true);
                addAutoScoreCommand("AutoScoral right far", 8, 17, false);
                addAutoScoreCommand("AutoScoral 8/17 right", 8, 17, true);
                addAutoScoreCommand("AutoScoral back right (robot oriented)", 10, 21, true);
                addAutoScoreCommand("AutoScoral back left (robot oriented)", 10, 21, false);

                // Command autoScore11Left
                // NamedCommands.registerCommand("AutoScoral Right",
                // swerveCommands.autoScoralClosest(Constants.SetpointConstants.REEF_L2, true));

                // NamedCommands.registerCommand("AutoScoral Left",
                // swerveCommands.autoScoralClosest(Constants.SetpointConstants.REEF_L2,
                // false));
                NamedCommands.registerCommand("Intake Coral From Human Player", intakeCommands.intakeHumanPlayer());

                NamedCommands.registerCommand("Elevator Down", elevator.goDown());

                // Logging callback for the active path, this is sent as a list of poses
                PathPlannerLogging.setLogActivePathCallback((poses) -> {
                        // Do whatever you want with the poses here
                        drivebase.getSwerveDrive().field.getObject("AutoAlignPath").setPoses(poses);
                });
        }

        public void addAutoScoreCommand(String name, int redTag, int blueTag, boolean isRight) {

                Command redCommand = swerveCommands.autoScoral(Optional.of(redTag),
                                Constants.SetpointConstants.REEF_L4, isRight);
                Command blueCommand = swerveCommands.autoScoral(Optional.of(blueTag),
                                Constants.SetpointConstants.REEF_L4, isRight);

                NamedCommands.registerCommand(name,
                                Commands.either(redCommand, blueCommand, () -> PoseTransformUtils.isRedAlliance()));
        }

        public void updateTelemetry() {
                robotTelemetry.update();
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
                                        if (!PoseTransformUtils.isRedAlliance()) {
                                                y *= -1;
                                        }
                                        return m_yspeedLimiter.calculate(y);
                                },
                                () -> {
                                        double x = MathUtil.applyDeadband(
                                                        driverXbox.getLeftX(),
                                                        OperatorConstants.LEFT_X_DEADBAND);
                                        if (!PoseTransformUtils.isRedAlliance()) {
                                                x *= -1;
                                        }
                                        return m_xspeedLimiter.calculate(x);
                                },
                                () -> -driverXbox.getRightX());
                return driveFieldOrientedAngularVelocity; // :P
        }

        private void configureDriverBindings() { // RIP isPanel day 0, 2025 -> 3/25/2025

                new Trigger(() -> {
                        boolean value = DriverStation.isDisabled() && RobotContainer.runOnce;
                        RobotContainer.runOnce = true;
                        return value;
                }).whileTrue(elevator.stopElevator().repeatedly().ignoringDisable(true));

                new JoystickButton(driverXbox, XboxController.Button.kStart.value)
                                .onTrue((new InstantCommand(drivebase::zeroGyro)));

                new JoystickButton(driverXbox, XboxController.Button.kLeftBumper.value)
                                .whileTrue(Commands.parallel(
                                                swerveCommands.autoScoralClosest(false),
                                                ledKwarqs.isAutoScoring(true)))
                                .onFalse(Commands.parallel(intakeCommands.intakeStop(),
                                                ledKwarqs.isAutoScoring(false)));

                new JoystickButton(driverXbox, XboxController.Button.kRightBumper.value)
                                .whileTrue(Commands.parallel(
                                                swerveCommands.autoScoralClosest(true),
                                                ledKwarqs.isAutoScoring(true)))
                                .onFalse(Commands.parallel(intakeCommands.intakeStop(),
                                                ledKwarqs.isAutoScoring(false)));

                new JoystickButton(driverXbox, XboxController.Button.kY.value)
                                .onTrue(elevator.goDownAndIntake());

                new JoystickButton(driverXbox, XboxController.Button.kA.value)
                                .onTrue(intakeCommands.intakeOut());

                new JoystickButton(driverXbox, XboxController.Button.kB.value)
                                .onTrue(intakeCommands.intakeJustIn());

                new JoystickButton(driverXbox, XboxController.Button.kBack.value)
                                .onTrue(swerveCommands.orbitReefCenter());

                new JoystickButton(driverXbox, XboxController.Button.kX.value)
                                .onTrue(intakeCommands.intakeOut());

                new Trigger(() -> driverXbox.getPOV() == 270)
                                .whileTrue(elevator.goToSetpoint(Constants.SetpointConstants.REEF_L2));
                new Trigger(() -> driverXbox.getPOV() == 0)
                                .whileTrue(elevator.goToSetpoint(Constants.SetpointConstants.REEF_L3));
                new Trigger(() -> driverXbox.getPOV() == 90)
                                .whileTrue(elevator.goToSetpoint(Constants.SetpointConstants.REEF_L4));
                new Trigger(() -> driverXbox.getPOV() == 180)
                                .onTrue(elevator.goDown());

                new JoystickButton(driverXbox, XboxController.Button.kA.value)
                                .onTrue(elevator.goDown());

                // new JoystickButton(driverXbox, XboxController.Button.kY.value)
                // .onTrue(elevator.goUp());

        }

        private void configureOperatorBindings() { // RIP isPanel day 0, 2025 -> 3/25/2025

                new Trigger(() -> {
                        boolean value = DriverStation.isDisabled() && RobotContainer.runOnce;
                        RobotContainer.runOnce = true;
                        return value;
                }).whileTrue(elevator.stopElevator().repeatedly().ignoringDisable(true));

                new JoystickButton(operator, XboxController.Button.kA.value)
                                .whileTrue(climberSubsystem.climb()).onFalse(climberSubsystem.climbStop());// arm.goLittleUp(.05));
                                                                                                           // //
                                                                                                           // climberSubsystem.climb());

                new JoystickButton(operator, XboxController.Button.kB.value)
                                .whileTrue(climberSubsystem.deClimb()).onFalse(climberSubsystem.climbStop()); // arm.goLittleDown(.05));//

                new JoystickButton(operator, XboxController.Button.kY.value)
                                .onTrue(intakeCommands.in());

                new JoystickButton(operator, XboxController.Button.kX.value)
                                .onTrue(intakeCommands.intakeOut());

                new Trigger(() -> operator.getPOV() == 270)
                                .whileTrue(Commands.parallel(
                                                swerveCommands.autoAlignAndIntakeAlgae(
                                                                Constants.SetpointConstants.ALGAE_INTAKE_L2),
                                                ledKwarqs.isAutoScoring(true)))
                                .onFalse(Commands.sequence(ledKwarqs.isAutoScoring(false),
                                                arm.goToSetpoint(Constants.ArmConstants.ALGAE_HOLD),
                                                elevator.goToSetpoint(Constants.SetpointConstants.ZERO),
                                                intakeCommands.holdAlgae()));
                new Trigger(() -> operator.getPOV() == 0)
                                .whileTrue(Commands.parallel(
                                                swerveCommands.autoAlignAndIntakeAlgae(
                                                                Constants.SetpointConstants.REEF_L3),
                                                ledKwarqs.isAutoScoring(true)))
                                .onFalse(Commands.sequence(ledKwarqs.isAutoScoring(false),
                                                arm.goToSetpoint(Constants.ArmConstants.ALGAE_HOLD),
                                                elevator.goToSetpoint(Constants.SetpointConstants.ZERO),
                                                intakeCommands.holdAlgae()));// );
                new Trigger(() -> operator.getPOV() == 90)
                                .whileTrue(arm.goToSetpoint(Constants.ArmConstants.ALGAE_PROCESS));

                new Trigger(() -> operator.getPOV() == 180)
                                .onTrue(elevator.intakeGroundAlgae())
                                .onFalse(Commands.parallel(arm.goToSetpoint(Constants.ArmConstants.ALGAE_HOLD),
                                                intakeCommands.holdAlgae()));

                // new Trigger(() -> operator.getLeftTriggerAxis() > 0.1)
                // .whileTrue(Commands.parallel(
                // swerveCommands.autoAlignAndDescoreAlgae(
                // Constants.SetpointConstants.ALGAE_DESCORE_L2),
                // ledKwarqs.isAutoScoring(true)))
                // .onFalse(Commands.parallel(Commands.sequence(intakeCommands.intakeStop(),
                // arm.goToSetpoint(Constants.ArmConstants.OUTSIDE_ELEVATOR)),
                // ledKwarqs.isAutoScoring(false)));

                // new Trigger(() -> operator.getRightTriggerAxis() > 0.1)
                // .whileTrue(swerveCommands
                // .autoAlignAndDescoreAlgae(Constants.SetpointConstants.ALGAE_DESCORE_L3))
                // .onFalse(Commands.sequence(intakeCommands.intakeStop(),
                // arm.goToSetpoint(Constants.ArmConstants.OUTSIDE_ELEVATOR)));

                // .onTrue(elevator.goLittleDown(1));
                new JoystickButton(operator, XboxController.Button.kBack.value)
                                .whileTrue(Commands.parallel(intakeCommands.eject(), ledKwarqs.isEjectingPOOP(true)))
                                .onFalse(Commands.parallel(ledKwarqs.isEjectingPOOP(false),
                                                Commands.sequence(intakeCommands.intakeStop(),
                                                                funnelSubsystem.stop())));

                new JoystickButton(operator, XboxController.Button.kStart.value) // right
                                .whileTrue(intakeCommands.ejectAlgae())
                                .onFalse(Commands.sequence(arm.goToSetpoint(Constants.ArmConstants.HANDOFF_POSE),
                                                intakeCommands.intakeStop()));
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
        public AutoCommand getAutonomousCommand(String auto) {
                // An example command will be run in autonomous
                boolean isRed = false;
                if (auto.equals("Left Side 3 Piece RED J & K & L")) {
                        isRed = true;
                } else if (auto.equals("Right Side 3 Piece RED E & C & D")) {
                        isRed = true;
                }
                return drivebase.getAutonomousCommand(auto, isRed);
        }

        public AutoCommand getAutonomousCommand() {
                return getAutonomousCommand(m_chooser.getSelected());
        }

        public void setIsBlue(boolean isBlue) {
                drivebase.setIsBlue(isBlue);
        }

        public void configureBindings() {
                new Trigger(() -> RobotController.getUserButton()).onTrue(Commands.runOnce(() -> {
                        drivebase.toggleLedRing();
                }).ignoringDisable(true));

                configureDriverBindings();
                configureOperatorBindings();
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }

}
