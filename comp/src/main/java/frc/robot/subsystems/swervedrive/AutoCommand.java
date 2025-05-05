package frc.robot.subsystems.swervedrive;

import com.pathplanner.lib.commands.PathPlannerAuto;

public class AutoCommand extends PathPlannerAuto{
    private boolean isRed;

    public AutoCommand(String pathName, boolean isRed) {
        super(pathName);
        this.isRed = isRed;
    }

    public boolean isRed(){
        return isRed;
    }
}
