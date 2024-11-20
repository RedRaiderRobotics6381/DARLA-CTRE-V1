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
  
  public static final class LauncherConstants
  {
    public static final int kLauncherTopLeft = 21;
    public static final int kLauncherTopRight = 22;
    public static final int feederLauncher = 20;

    public static final double bottomIntakeSpeed = 50.0;
    public static final double topIntakeSpeed = 50.0;

    public static final double bottomAmpSpeed = 75.0;
    public static final double topAmpSpeed = 75.0;

    public static final double topSpeakerSpeed = 1000.0;
    public static final double bottomSpeakerSpeed = 200.0;

    public static final double zeroSpeed = 0.0;

  }
  public static final class AprilTagConstants
  {
    public static int ampID = 0;
    public static int speakerID = 0;
    public static int stageIDA = 0;
    public static int stageIDB = 0;
    public static int stageIDC = 0;
    public static int sourceA = 0;
    public static int sourceB = 0;
  }

}