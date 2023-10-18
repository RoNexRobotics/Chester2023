// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.25);
    public static final double kWheelBase = Units.inchesToMeters(25.25);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2)
    );

    // TODO: Check these values
    public static final double kFrontLeftAngularOffset = Math.toRadians(0);
    public static final double kRearLeftAngularOffset = Math.toRadians(0);
    public static final double kFrontRightAngularOffset = Math.toRadians(0);
    public static final double kRearRightAngularOffset = Math.toRadians(0);

    // Spark Max CAN IDs
    public static final int kFrontLeftDriveMotorId = 1;
    public static final int kRearLeftDriveMotorId = 3;
    public static final int kFrontRightDriveMotorId = 5;
    public static final int kRearRightDriveMotorId = 7;

    public static final int kFrontLeftTurnMotorId = 2;
    public static final int kRearLeftTurnMotorId = 4;
    public static final int kFrontRightTurnMotorId = 6;
    public static final int kRearRightTurnMotorId = 8;

    // Motor configuration
    public static final boolean kFrontLeftDriveMotorInverted = false;
    public static final boolean kFrontRightDriveMotorInverted = false;
    public static final boolean kRearLeftDriveMotorInverted = false;
    public static final boolean kRearRightDriveMotorInverted = true;

    public static final boolean kFrontLeftTurnMotorInverted = false;
    public static final boolean kFrontRightTurnMotorInverted = false;
    public static final boolean kRearLeftTurnMotorInverted = false;
    public static final boolean kRearRightTurnMotorInverted = false;

    public static final boolean kLeftDriveInverted = true;
    public static final boolean kRightDriveInverted = false;

    // Speeds
    public static final double kPowerPercent = 1;
    public static final double kAngularPowerPercent = 1;
    public static final double kSwervePowerPercent = 1;

    // PID values
    // TODO: Tune PID values (Specifically the P and D terms)
    public static final double kP = 0.7; // Larger value reacts very quickly to large error, while smaller value is less reactive
    public static final double kI = 0; // Larger value will nudge onto target more proactively
    public static final double kD = 0.2; // Larger value dampens more and will try to slow down more aggressively
    public static final double kFF = 0;

    // Slew rate values
    public static final double kMagnitudeSlewRate = 0.9;
    public static final double kRotationalSlewRate = 0.9;
  }

  public static final class ModuleConstants {
    // Gear reductions
    public static final double kDriveMotorRatio = 7.36; // 7.36:1 // TODO: Check this value. Update: Seems decent enough
    public static final double kTurnMotorRatio = 1 / 1.9098593171027440292266051604702; // 1:1.9098593171027440292266051604702

    public static final double kDriveMotorFreeSpeed = NeoMotorConstants.kFreeSpeedRpm;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDriveWheelFreeSpeed = (kDriveMotorFreeSpeed * kWheelCircumferenceMeters) / kDriveMotorRatio;

    public static final boolean kDriveMotorInverted = false;
    public static final boolean kTurnMotorInverted = false;

    public static final double kDriveEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDriveMotorRatio; // Meters
    public static final double kDriveEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDriveMotorRatio) / 60.0; // Meters per second

    public static final double kTurnEncoderPositionFactor = (2 * Math.PI); // Radians
    public static final double kTurnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // Radians per second

    // PID controller values
    public static final double kDriveP = 0.5;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    public static final double kDriveFF = 1 / kDriveWheelFreeSpeed;
    public static final double kDriveMinOutput = -1;
    public static final double kDriveMaxOutput = 1;

    public static final double kTurnP = 0.8;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0.2;
    public static final double kTurnFF = 0;
    public static final double kTurnMinOutput = -1;
    public static final double kTurnMaxOutput = 1;

    public static final IdleMode kDriveMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurnMotorIdleMode = IdleMode.kBrake;

    public static final double kTurnEncoderPositionPIDMinInput = 0; // Radians
    public static final double kTurnEncoderPositionPIDMaxInput = kTurnEncoderPositionFactor; // Radians
  }

  public static final class ArmConstants {
    public static final int kRaiseMotorId = 23;
    public static final int kExtensionMotorId = 22;
    public static final int kPivotMotorId = 21;
    
    public static final int kRaiseEncoderChannelA = 2;
    public static final int kRaiseEncoderChannelB = 3;
    public static final int kExtensionEncoderChannelA = 0;
    public static final int kExtensionEncoderChannelB = 1;

    public static final int kExtensionLimitSwitchId = 9;
    public static final int kRaiseLimitSwitchId = 8;
    public static final int kPivotLowerLimitSwitchId = 7;
    public static final int kPivotUpperLimitSwitchId = 6;

    public static final boolean kRaiseEncoderInverted = false;
    public static final boolean kExtensionEncoderInverted = false;

    public static final EncodingType kRaiseEncoderEncodingType = EncodingType.k1X;
    public static final EncodingType kExtensionEncoderEncodingType = EncodingType.k1X;
    public static final EncodingType kPivotEncoderEncodingType = EncodingType.k1X;

    public static final double kRaiseMotorPowerPercent = 1;
    public static final double kExtensionMotorPowerPercent = 1;
    public static final double kPivotMotorPowerPercent = 0.5;

    // Encoder maximum limits (After configuration)
    public static final double kRaiseEncoderMaxValue = -12584;
    public static final double kExtensionEncoderMaxValue = 2682;
  }

  public static final class VacuumConstants {
    public static final int kVacuumMotorId = 20;

    public static final int kUpperSolenoidId = 1;
    public static final int kLowerSolenoidId = 0;
  }

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriverControllerDeadband = 0.2;
    public static final int kArmControllerPort = 1;
    public static final double kArmControllerDeadband = 0.2;
  }

  public static final class AutoConstants {
    public static final double kVisionP = 0.3;
    public static final double kVisionI = 0;
    public static final double kVisionD = 0;
  }

  public class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}