// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 5;
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    public static final double ROLLER_EJECT_VALUE = 0.44;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }
  public static final class KinematicsConstants {
    public static final double ROBOT_WIDTH = 0.6;  // Example: 60 cm
    public static final double ROBOT_LENGTH = 0.6;

    // Define the locations of the wheels relative to the robot center
    public static final Translation2d FRONT_LEFT_POSITION = new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2);
    public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2);
    public static final Translation2d BACK_LEFT_POSITION = new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2);
    public static final Translation2d BACK_RIGHT_POSITION = new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2);

    // Create kinematics object
    public static final MecanumDriveKinematics KINEMATICS = new MecanumDriveKinematics(
        FRONT_LEFT_POSITION, FRONT_RIGHT_POSITION, BACK_LEFT_POSITION, BACK_RIGHT_POSITION
    );
  }
}
