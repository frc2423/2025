package frc.robot.subsystems.Elevator;

import java.lang.module.ModuleDescriptor.Builder;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;

import frc.robot.Constants;
import frc.robot.NTHelper;
import frc.robot.PoseTransformUtils;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.NTHelper;

public class ElevatorLevelPicker {

    private ElevatorSubsystem elevatorSubsystem;

    private SwerveSubsystem swerve;
    private XboxController driverXbox = new XboxController(0);
    private final String[] DEFAULT_ELEVATOR_LEVEL = { "off" };

    private enum ElevatorLevel {
        T, L2, L3, L4
    }

    // only make front left and back left lists for now

    ElevatorLevel[] elevatorLevels = {
            ElevatorLevel.L4, ElevatorLevel.L4,
            ElevatorLevel.L3, ElevatorLevel.L3,
            ElevatorLevel.L2, ElevatorLevel.L2
    };

    boolean[] frontLeftReef = new boolean[6];//
    boolean[] backLeftReef = new boolean[6];

    boolean[] frontRightReef = new boolean[6];//
    boolean[] backRightReef = new boolean[6];

    boolean[] frontMiddleReef = new boolean[6];
    boolean[] backMiddleReef = new boolean[6];

    String[] reefPositionNames = { "frontLeft", "backLeft", "backMiddle", "backRight", "frontRight", "frontMiddle" };
    Map<String, boolean[]> nameToReefMap = Map.ofEntries(Map.entry("frontLeft", frontLeftReef),
            Map.entry("frontRight", frontRightReef), Map.entry("frontMiddle", frontMiddleReef),
            Map.entry("backLeft", backLeftReef), Map.entry("backRight", backRightReef),
            Map.entry("backMiddle", backMiddleReef));

    Map<String, Integer> nameToIdMapBlue = Map.ofEntries(Map.entry("frontLeft", 19),
            Map.entry("frontRight", 17), Map.entry("frontMiddle", 18),
            Map.entry("backLeft", 20), Map.entry("backRight", 22),
            Map.entry("backMiddle", 21));

    Map<String, Integer> nameToIdMapRed = Map.ofEntries(Map.entry("frontLeft", 6),
            Map.entry("frontRight", 8), Map.entry("frontMiddle", 7),
            Map.entry("backLeft", 11), Map.entry("backRight", 9),
            Map.entry("backMiddle", 10));

    public ElevatorLevelPicker(ElevatorSubsystem elevatorSubsystem, SwerveSubsystem swerve) {

        this.elevatorSubsystem = elevatorSubsystem;
        this.swerve = swerve;

        NTHelper.setStringArray("/elevatorLevel", DEFAULT_ELEVATOR_LEVEL);

        double[] defaultLevelValues = { 0, 0, 0, 0, 0, 0 };

        for (var name : reefPositionNames) {
            NTHelper.setBooleanArray("/elevatorLevelPicker/" + name, nameToReefMap.get(name));
            NTHelper.setDoubleArray("/elevatorLevelPicker/" + name + "Dashboard", defaultLevelValues);
            NTHelper.listen("/elevatorLevelPicker/" + name + "Dashboard", (event) -> {
                double[] newValues = NTHelper.getDoubleArray("/elevatorLevelPicker/" + name + "Dashboard",
                        defaultLevelValues);
                setDash(newValues, nameToReefMap.get(name), "/elevatorLevelPicker/" + name);
            });

        }

    }

    public String getClosestReefString() {
        int ID = swerve.vision.findClosestTagID(swerve.getPose());
        if (ID == 19 || ID == 6) {
            return "frontLeft";
        }
        if (ID == 17 || ID == 8) {
            return "frontRightReef";
        }
        if (ID == 20 || ID == 11) {
            return "backLeftReef";
        }
        if (ID == 22 || ID == 9) {
            return "backRightReef";
        }
        if (ID == 18 || ID == 7) {
            return "frontMiddleReef";
        } else {
            return "backMiddleReef";
        }
    }

    public boolean[] getClosestReef() {
        String name = getClosestReefName();
        return nameToReefMap.get(name);
    }

    public boolean reefIsOpen(String name) {
        boolean[] reef = nameToReefMap.get(name);

        for (boolean bool : reef) {
            if (bool == false) {
                return true;
            }
        }

        return false;
    }

    public boolean reefIsOpen(boolean[] reef) {
        for (boolean bool : reef) {
            if (bool == false) {
                return true;
            }
        }

        return false;
    }

    public Pose2d getNearestOpenReefPose() {
        String name = getNearestOpenReefName();

        return Vision.getAprilTagPose(getIdFromReefName(name));
    }

    public boolean[] getNearestOpenReef() {
        String name = getNearestOpenReefName();

        return nameToReefMap.get(name);
    }

    public String getNearestOpenReefName() {
        String[] options = getAdjacentReefNames();
        Pose2d pose1 = Vision.getAprilTagPose(getIdFromReefName(options[0]));
        Pose2d pose2 = Vision.getAprilTagPose(getIdFromReefName(options[1]));
        String closestPose = getClosestReefName();

        if (reefIsOpen(closestPose)) {
            return closestPose;
        }

        if (swerve.getPose().getTranslation().getDistance(pose1.getTranslation()) >= swerve.getPose().getTranslation()
                .getDistance(pose2.getTranslation())) {
            if (reefIsOpen(options[1]))
                return options[1];
            else
                return options[0];
        } else if (swerve.getPose().getTranslation().getDistance(pose1.getTranslation()) < swerve.getPose()
                .getTranslation()
                .getDistance(pose2.getTranslation())) {
            if (reefIsOpen(options[0]))
                return options[0];
            else
                return options[1];
        }

        return closestPose;
    }

    public int getIdFromReefName(String name) {
        if (PoseTransformUtils.isRedAlliance()) {
            return nameToIdMapRed.get(name);
        } else {
            return nameToIdMapBlue.get(name);
        }
    }

    public String[] getAdjacentReefNames() {
        String closestReef = getClosestReefName();
        int index = indexOfStringArray(closestReef, reefPositionNames);
        String[] options = { "", "" };

        if (index > 0 && index < 5) {
            options[0] = reefPositionNames[index - 1];
            options[1] = reefPositionNames[index + 1];
        } else if (index == 0) {
            options[0] = reefPositionNames[5];
            options[1] = reefPositionNames[1];
        } else if (index == 5) {
            options[0] = reefPositionNames[4];
            options[1] = reefPositionNames[0];
        }

        return options;
    }

    public int indexOfStringArray(String string, String[] array) {
        for (int i = 0; i < array.length; i++) {
            if (array[i].equals(string)) {
                return i;
            }
        }
        return -1;
    }

    public String getClosestReefName() {
        int ID = swerve.vision.findClosestTagID(swerve.getPose());
        if (ID == 19 || ID == 6) {
            return "frontLeft";
        }
        if (ID == 17 || ID == 8) {
            return "frontRight";
        }
        if (ID == 20 || ID == 11) {
            return "backLeft";
        }
        if (ID == 22 || ID == 9) {
            return "backRight";
        }
        if (ID == 18 || ID == 7) {
            return "frontMiddle";
        } else {
            return "backMiddle";
        }
    }

    public boolean isRightOpen(boolean[] array) {
        int index = 0;
        for (int i = 0; i < 6; i++) {
            if (!array[i]) {
                index = i;
                break;
            }
        }
        return index % 2 != 0;
    }

    public boolean isRightOpen() {
        return isRightOpen(getNearestOpenReef());
    }

    private ElevatorLevel selectElevatorLevel() {
        String dashboardElevatorLevel = NTHelper.getStringArray("/elevatorLevel", DEFAULT_ELEVATOR_LEVEL)[0];
        if (dashboardElevatorLevel.equals("L1")) {
            return ElevatorLevel.T;
        }
        if (dashboardElevatorLevel.equals("L2")) {
            return ElevatorLevel.L2;
        }
        if (dashboardElevatorLevel.equals("L3")) {
            return ElevatorLevel.L3;
        }
        if (dashboardElevatorLevel.equals("L4")) {
            return ElevatorLevel.L4;
        }

        if (driverXbox.getLeftTriggerAxis() < 0.5 && driverXbox.getRightTriggerAxis() < 0.5) {
            return ElevatorLevel.T;
        } else if (driverXbox.getLeftTriggerAxis() < 0.5 && driverXbox.getRightTriggerAxis() > 0.5) {
            return ElevatorLevel.L2;
        } else if (driverXbox.getLeftTriggerAxis() > 0.5 && driverXbox.getRightTriggerAxis() < 0.5) {
            return ElevatorLevel.L3;
        } else {
            return ElevatorLevel.L4;
        }
    }

    public ElevatorLevel getElevatorLevelAuto(boolean[] array) {
        for (int i = 0; i < 6; i++) {
            if (!array[i]) {
                return elevatorLevels[i];
            }
        }
        return elevatorLevels[0];
    }

    public ElevatorLevel getElevatorLevelAuto() {
        return getElevatorLevelAuto(getNearestOpenReef());
    }

    public Command setScoredLevel() {
        return Commands.runOnce(() -> {
            String reefName = getClosestReefName();
            var array = nameToReefMap.get(reefName);

            for (int i = 0; i < 6; i++) {
                if (!array[i]) {
                    array[i] = true;
                    break;
                }
            }

            var dash = getDash(array);

            NTHelper.setBooleanArray("/elevatorLevelPicker/" + reefName, array);
            NTHelper.setDoubleArray("/elevatorLevelPicker/" + reefName + "Dashboard", dash);
        });
    }

    public void setDash(double[] array, boolean[] reef, String key) {
        for (int i = 0; i < array.length; i++) {
            if (array[i] == 1) {
                reef[i] = true;
            } else {
                reef[i] = false;
            }
        }
        NTHelper.setBooleanArray(key, reef);
    }

    public double[] getDash(boolean[] array) {
        double[] dash = new double[6];

        for (int i = 0; i < array.length; i++) {
            if (array[i] == true)
                dash[i] = 1;
            else
                dash[i] = 0;
        }

        return dash;
    }

    public Command getElevatorLevelCommandAuto() {
        return new SelectCommand<>(
                // Maps selector values to commands
                Map.ofEntries(
                        Map.entry(ElevatorLevel.T, elevatorSubsystem.goToSetpoint(Constants.SetpointConstants.ZERO)),
                        Map.entry(ElevatorLevel.L2,
                                elevatorSubsystem.goToSetpoint(Constants.SetpointConstants.REEF_L2)),
                        Map.entry(ElevatorLevel.L3,
                                elevatorSubsystem.goToSetpoint(Constants.SetpointConstants.REEF_L3)),
                        Map.entry(ElevatorLevel.L4,
                                elevatorSubsystem.goToSetpoint(Constants.SetpointConstants.REEF_L4))),

                this::getElevatorLevelAuto);
    }

    public Command getElevatorLevelCommand() {
        return new SelectCommand<>(
                // Maps selector values to commands
                Map.ofEntries(
                        Map.entry(ElevatorLevel.T, elevatorSubsystem.goToSetpoint(Constants.SetpointConstants.ZERO)),
                        Map.entry(ElevatorLevel.L2,
                                elevatorSubsystem.goToSetpoint(Constants.SetpointConstants.REEF_L2)),
                        Map.entry(ElevatorLevel.L3,
                                elevatorSubsystem.goToSetpoint(Constants.SetpointConstants.REEF_L3)),
                        Map.entry(ElevatorLevel.L4,
                                elevatorSubsystem.goToSetpoint(Constants.SetpointConstants.REEF_L4))),

                this::selectElevatorLevel);
    }
}
