// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class RobotContainer {

  public RobotContainer() {
    Pose3d pose = new Pose3d(0, 0, 0, new Rotation3d());
    NTHelper.setPose3d("/robotPose", pose);
    NTHelper.setDouble("/tuning/robotX", 0);
    NTHelper.setDouble("/tuning/robotY", 0);
    NTHelper.setDouble("/tuning/robotZ", 0);

    Pose3d cameraPose =   new Pose3d(1, 4, Units.inchesToMeters(6), new Rotation3d(0, Math.toRadians(-25), Math.toRadians(0)));
    NTHelper.setPose3d("/cameraPose", cameraPose);
  }

  public void periodic() {
    double robotX = NTHelper.getDouble("/tuning/robotX", 0);
    double robotY = NTHelper.getDouble("/tuning/robotY", 0);
    double robotZ = NTHelper.getDouble("/tuning/robotZ", 0);
    Pose3d pose = new Pose3d(robotX, robotY, robotZ, new Rotation3d());
    NTHelper.setPose3d("/robotPose", pose);
  }
}
