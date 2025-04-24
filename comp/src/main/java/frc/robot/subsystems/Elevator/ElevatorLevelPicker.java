package frc.robot.subsystems.Elevator;

import java.lang.module.ModuleDescriptor.Builder;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;

import frc.robot.Constants;
import frc.robot.NTHelper;

import frc.robot.NTHelper;

public class ElevatorLevelPicker {

    private ElevatorSubsystem elevatorSubsystem;
    private XboxController driverXbox = new XboxController(0);
    private final String[] DEFAULT_ELEVATOR_LEVEL = { "off" };

    public ElevatorLevelPicker(ElevatorSubsystem elevatorSubsystem) {

        this.elevatorSubsystem = elevatorSubsystem;

        NTHelper.setStringArray("/elevatorLevel", DEFAULT_ELEVATOR_LEVEL);
    }

    private enum ElevatorLevel {
        T, L2, L3, L4
    }

    // only make front left and back left lists for now

    ElevatorLevel[] elevatorLevels = {
            ElevatorLevel.L4, ElevatorLevel.L4,
            ElevatorLevel.L3, ElevatorLevel.L3,
            ElevatorLevel.L2, ElevatorLevel.L2
    };

    boolean[] frontLeftReef = new boolean[6];
    boolean[] backLeftReef = new boolean[6];

    public boolean isRightOpen(boolean[] array) {
        int index = 0;
        for (int i = 0; i < 6; i++) {
            if (!array[i]) {
                index = i;
            }
        }
        return index % 2 != 0;
    }

    public boolean isRightOpen() {
        return isRightOpen(frontLeftReef);
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
        return getElevatorLevelAuto(frontLeftReef);
    }

    public Command setScoredLevel(boolean[] array) {
        return Commands.runOnce(() -> {
            for (int i = 0; i < 6; i++) {
                if (!array[i]) {
                    array[i] = true;
                    break;
                }
            }
            NTHelper.setBooleanArray("/elevatorLevelPicker/values", array);
        });
    }

    public Command setScoredLevel() {
        return setScoredLevel(frontLeftReef);
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
