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
public final class Constants {
  public static final class JoystickConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class CAN_IDs {
    public static final int frontLeftID = 1;
    public static final int backLeftID = 2;
    public static final int frontRightID = 3;
    public static final int backRightID = 4;
  }

  public static final class RobotConstants {
    public static final double wheelDiameter = 8;
    public static final double limelightMountHeight = 35;

    // gear ratios
    public static final double driveGearRatio = 8.4;
    public static final double forearmGearRatio = 0.005208;
    
  }


  /* Joystick controls:
    driver:
      drive: leftStick Y axis
      strafe: leftStick X axis
      rotate: rightStick X axis
      balance: rightBumper
  */
}
