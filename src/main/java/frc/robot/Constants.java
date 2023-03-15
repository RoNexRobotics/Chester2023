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
  public static class DriveConstants {
    // Spark Max CAN IDs
    public static final int kFrontLeftDriveMotorId = 1;
    public static final int kRearLeftDriveMotorId = 3;
    public static final int kFrontRightDriveMotorId = 5;
    public static final int kRearRightDriveMotorId = 7;

    public static final int kFrontLeftTurnMotorId = 2;
    public static final int kRearLeftTurnMotorId = 4;
    public static final int kFrontRightTurnMotorId = 6;
    public static final int kRearRightTurnMotorId = 8;

    public static final boolean kLeftDriveInverted = true;
    public static final boolean kRightDriveInverted = false;

    public static final double kPowerPercent = 0.8;
    public static final double kAngularPowerPercent = 0.8;
    public static final double kSwervePowerPercent = 0.6;

    // PID values
    public static final double kP = 0.4;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0;
  }

  public static final class VacuumConstants {
    public static final int kVacuumMotorId = 20;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriverControllerDeadband = 0.2;
    public static final int kArmControllerPort = 1;
  }
}