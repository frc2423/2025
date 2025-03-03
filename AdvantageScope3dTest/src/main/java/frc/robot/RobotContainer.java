// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class RobotContainer {
  double maxArmPitch = 45.0;
  double currentArmPitch = 0;
  boolean rising = true;
  
  public RobotContainer() {
    Pose3d pose = new Pose3d(0, 0, 0, new Rotation3d());
    Rotation3d pose45pitch = new Rotation3d(0, 45, 0);
    NTHelper.setPose3d("/robotPose", pose);
    NTHelper.setDouble("/tuning/robotX", 0);
    NTHelper.setDouble("/tuning/robotY", 0);
    NTHelper.setDouble("/tuning/robotZ", 0);

    Pose3d cameraPose =   new Pose3d(1, 4, Units.inchesToMeters(6), new Rotation3d(0, Math.toRadians(-25), Math.toRadians(0)));
    NTHelper.setPose3d("/cameraPose", cameraPose);
    NTHelper.setPose3d("/pivotPose", new Pose3d(0, 0, 0, new Rotation3d()));
  }

  public void periodic() {
    double robotX = NTHelper.getDouble("/tuning/robotX", 0);
    double robotY = NTHelper.getDouble("/tuning/robotY", 0);
    double robotZ = NTHelper.getDouble("/tuning/robotZ", 0);
    Pose3d pose = new Pose3d(robotX, robotY, robotZ, new Rotation3d());
    NTHelper.setPose3d("/robotPose", pose);
    if(currentArmPitch <= 0) {
      currentArmPitch += 1;
      rising = true;
    }
    else if (currentArmPitch >= maxArmPitch) {
      currentArmPitch -= 1;
      rising = false;
    }
    else if (0 < currentArmPitch && currentArmPitch < maxArmPitch) {
      if(!rising) {
        currentArmPitch -= 1;
      }
      else {
        currentArmPitch += 1;
      }
    }
    NTHelper.setPose3d("/pivotPose", new Pose3d(0, 0, 0, new Rotation3d(0, Math.toRadians(currentArmPitch), 0)));
  }
}
