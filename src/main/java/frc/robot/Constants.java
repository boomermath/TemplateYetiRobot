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
public final class Constants
{
    public static final int LEFT_DRIVE_TALON_1 = 0;
    public static final int LEFT_DRIVE_TALON_2 = 1;
    public static final int RIGHT_DRIVE_TALON_1 = 2;
    public static final int RIGHT_DRIVE_TALON_2 = 3;
    public static final int ARM_TALON = 4;
    public static final int ARM_UPPER_LIMIT_PORT = 0;
    public static final int ARM_LOWER_LIMIT_PORT = 1;
    public static final int FORWARD_CHANNEL = 0;
    public static final int REVERSE_CHANNEL = 1;
    public static final int XBOX_CONTROLLER_PORT = 0;
    public static final double MOTOR_VOLTAGE_COMP = 10;
}
