// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public interface Constants {

  public interface Ports {
    // Controllers
    int kDriverController = 0;
    int kOperatorController = 1;

    // Motors
    int FRONTLEFT = 1;
    int BACKLEFT = 2;

    int FRONTRIGHT = 3;
    int BACKRIGHT = 4;
  }

  public interface Drivetrain {
    double kDriveSpeedCap = 0.7;
    double kTimedDriveVoltage = 2.5;
    double kTimedDriveTime = 3;
  }
}
