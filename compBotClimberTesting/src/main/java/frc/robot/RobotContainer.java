// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  XboxController driverXbox = new XboxController(0);
  XboxController operator = new XboxController(1);
  ClimberSubsystem climberSub = new ClimberSubsystem();
  FunnelSubsystem funnelSubsystem = new FunnelSubsystem();
  ArmSubsystem armSubsystem = new ArmSubsystem();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    SmartDashboard.putData("arm", armSubsystem);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new JoystickButton(operator, XboxController.Button.kLeftBumper.value)
                    .whileTrue(climberSub.deClimb());
    new JoystickButton(operator, XboxController.Button.kRightBumper.value)
                    .whileTrue(climberSub.climb());
    new JoystickButton(driverXbox, XboxController.Button.kX.value)
                    .whileTrue(Commands.parallel(funnelSubsystem.spinInBoth(), armSubsystem.setScoringWheelSpeed(.8)));
    new JoystickButton(driverXbox, XboxController.Button.kY.value)
                    .whileTrue(armSubsystem.goToSetpoint(armSubsystem.middle));
    new JoystickButton(driverXbox, XboxController.Button.kA.value)
                    .whileTrue(armSubsystem.goToSetpoint(armSubsystem.lowestPose + 2));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   //return Autos.exampleAuto(m_exampleSubsystem);
  // }
}
