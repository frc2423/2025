// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class Vision {
        public static final String kCameraName = "Arducam_OV9281_USB_Camera";
        public static final String knoteCameraName = "Arducam_OV9782_USB_Camera";

        // Assumed the origin point was on the floor -(^o^)-
        public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(-0.229, 0, 0.394),
                new Rotation3d(0, -0.419, Math.PI));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout;

        static {
            kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        }

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8 * 1.5);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1 * 1.5);
    }

    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED = Units.feetToMeters(14.5);
    // Maximum speed of the robot in meters per second, used to limit acceleration.

    // public static final class AutonConstants
    // {
    //
    // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
    // 0);
    // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
    // }

    public static final class DrivebaseConstants {
        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds
    }

    public static class OperatorConstants {

        // Joystick Deadband
        public static final double DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double LEFT_X_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = .75;
    }

    public static class SetpointConstants {
        public static final double REEF_L4 = 63.1569420;// 69.56; // noice
        public static final double REEF_L3 = 34.8;
        public static final double REEF_L2 = 18.2;

        public static final double ALGAE_INTAKE_L2 = 17;
        // public static final double ALGAE_DUNK_L2 = 15.5;
        // public static final double ALGAE_DUNK_L3 = 32;

        public static final double ALGAE_DESCORE_L3 = 26;
        public static final double ALGAE_DESCORE_L2 = 9;

        public static final double ALGAE_SCORE = 60;

        public static final double ZERO = 0.1;
    }

    public static class ArmConstants {
        // public static final double ALGAE_DESCORE = -7.5;// -7.5;// .2; // not real
        public static final double ALGAE_INTAKE = 0.809670;// -6.8; // all relative
        public static final double ALGAE_SCORE = 0.5; // 6; // relative can't use bc stupid
        public static final double ALGAE_GROUND = 0.718154; // -14.5; // NOT REAL relative
        public static final double ALGAE_HOLD = 0.885966;// 0.8156; // relative
        public static final double ALGAE_PROCESS = 0.85; // relative

        public static final double HANDOFF_POSE = 0.911322;
        public static final double OUTSIDE_ELEVATOR = 0.874339;

        public static final double SCORING_POSITION = 0.825457;
        public static final double L4_SCORING_POSITION = 0.80;

        public static final double ALGAE_DUNK = .79;

        public static final double ZERO = -1; // not real
    }

    public static class AprilTagPoses {
        public static final Rotation2d REEF_TAG_6_Rotation = new Rotation2d(Math.PI / 3);
        public static final Pose2d REEF_TAG_6 = new Pose2d(2.76, 13.786, REEF_TAG_6_Rotation);
        public static final Pose2d TEST_POSE2D = new Pose2d(1, 1, REEF_TAG_6_Rotation);
    }

    public static double inchToSetPoint(double inches) {
        return ((10 / 9.0) * inches) - (80 / 9.0);
    }
}
