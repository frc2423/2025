package frc.robot.subsystems.LED;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NTHelper;
import frc.robot.subsystems.swervedrive.Vision;

public class KwarqsLed extends SubsystemBase {
    private final LedController ledController = new LedController(40); // 36 on each side
    private final Vision visionSubsystem;
    private final XboxController xboxController;

    private boolean isAutoScoring = false;
    private boolean isEjectingPOOP = false;

    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }
        return Alliance.Red.equals(DriverStation.getAlliance().get());
    }

    public KwarqsLed(Vision visionSubsystem, XboxController xboxController) {
        this.xboxController = xboxController;
        this.visionSubsystem = visionSubsystem;
        ledController.add("yellow", new Yellow());
        ledController.add("orange", new Orange());
        ledController.add("purple", new Purple());
        ledController.add("green", new Green());
        ledController.add("rainbow", new Rainbow());
        ledController.add("dark", new Dark());
        ledController.add("GreenCycle", new GreenCycle());
        ledController.add("RedCycle", new RedCycle());
        ledController.add("BlueCycle", new BlueCycle());
        ledController.add("POOP", new POOP());

        ledController.add("AutoDown", new AutoDown());
        ledController.set("dark");

        // setDefaultCommand(disable());
    }

    public Command disable() {
        var command = Commands.run(() -> {
            // System.out.println("SEEES DARK!!!");

            // ledController.set("dark");
        });
        // command.ignoringDisable(true);

        command.addRequirements(this);
        return command;
    }

    public Command setYellow() {
        var command = Commands.run(() -> {
            // System.out.println("SEEES YELLOW!!!");
            // ledController.set("yellow");
        });
        // command.ignoringDisable(?true);
        command.addRequirements(this);
        return command;
    }

    public Command setOrange() {
        var command = Commands.run(() -> {
            // System.out.println("SEEES YELLOW!!!");
            // ledController.set("yellow");
        });
        // command.ignoringDisable(?true);
        command.addRequirements(this);
        return command;
    }

    public Command setPurple() {
        var command = Commands.run(() -> {
            // ledController.set("purple");
        });
        command.addRequirements(this);
        return command;
    }

    public Command setGreen() {
        var command = Commands.run(() -> {
            // System.out.println("!!!!!!!!");
            // ledController.set("green");
        });
        command.addRequirements(this);
        return command;
    }

    public Command isAutoScoring(boolean flag) {
        var command = Commands.runOnce(() -> {
            isAutoScoring = flag;
        });
        command.addRequirements(this);
        return command;
    }

    public Command isEjectingPOOP(boolean flag) {
        var command = Commands.runOnce(() -> {
            isEjectingPOOP = flag;
        });
        // command.addRequirements(this);
        return command;
    }

    @Override
    public void periodic() {
        if (RobotState.isTeleop() && !RobotState.isDisabled()) {

            if (isRedAlliance()) {
                ledController.set("RedCycle");
            } else if (!isRedAlliance()) {
                ledController.set("BlueCycle");
            } else {
                ledController.set("dark");
            }

        } else if (RobotState.isAutonomous() && !RobotState.isDisabled()) {
            ledController.set("AutoDown");
        } else {
            // System.out.println("disabled");
            if (RobotState.isDisabled() && visionSubsystem.seesFrontAprilTag()) {
                ledController.set("GreenCycle");
            } else {
                ledController.set("dark");
            }
        }

        if (isAutoScoring) {
            ledController.set("rainbow");
        }

        if (isEjectingPOOP) {
            ledController.set("POOP");
        }

        ledController.run();

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("led", () -> ledController.getCurrentLed(), null);
    }

}