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
  
  public static class OperatorConstants {
    
    public static final int kDriverControllerPort = 0;
    public static final int LeftDriveTalonPort = 2;
    public static final int RightDriveTalonPort = 3;
    public static final int LeftDriveVictorPort = 4;
    public static final int RightDriveVictorPort = 5;
    public static final int YAxis = 1;
    public static final int XAxis = 0;
    public static final int Zero = 0;
    public static final int One = 1;
    public static final double angleThreshhold = 5.0;
    public static double balanceDistance = 1.4;
    public static final double tickstoMeters = 15.5*Math.PI/(100.0*4096.0);

  }
}
