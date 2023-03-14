package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;

public class DriveSubsystem extends SubsystemBase {
  CANSparkMax m_frontLeftDriveMotor;
  CANSparkMax m_frontLeftTurnMotor;
  CANSparkMax m_frontRightDriveMotor;
  CANSparkMax m_frontRightTurnMotor;
  CANSparkMax m_rearLeftDriveMotor;
  CANSparkMax m_rearLeftTurnMotor;
  CANSparkMax m_rearRightDriveMotor;
  CANSparkMax m_rearRightTurnMotor;

  RelativeEncoder m_frontLeftTurnEncoder;
  RelativeEncoder m_frontRightTurnEncoder;
  RelativeEncoder m_rearLeftTurnEncoder;
  RelativeEncoder m_rearRightTurnEncoder;

  DifferentialDrive m_driveTrain;
  MotorControllerGroup m_leftDrive;
  MotorControllerGroup m_rightDrive;

  SparkMaxPIDController m_frontLeftTurnPIDController;
  SparkMaxPIDController m_frontRightTurnPIDController;
  SparkMaxPIDController m_rearLeftTurnPIDController;
  SparkMaxPIDController m_rearRightTurnPIDController;

  double m_desiredAngle = 0;

  public DriveSubsystem() {
    m_frontLeftDriveMotor = new CANSparkMax(DriveConstants.kFrontLeftDriveMotorId, MotorType.kBrushless);
    m_frontLeftTurnMotor = new CANSparkMax(DriveConstants.kFrontLeftTurnMotorId, MotorType.kBrushless);
    m_frontRightDriveMotor = new CANSparkMax(DriveConstants.kFrontRightDriveMotorId, MotorType.kBrushless);
    m_frontRightTurnMotor = new CANSparkMax(DriveConstants.kFrontRightTurnMotorId, MotorType.kBrushless);
    m_rearLeftDriveMotor = new CANSparkMax(DriveConstants.kRearLeftDriveMotorId, MotorType.kBrushless);
    m_rearLeftTurnMotor = new CANSparkMax(DriveConstants.kRearLeftTurnMotorId, MotorType.kBrushless);
    m_rearRightDriveMotor = new CANSparkMax(DriveConstants.kRearRightDriveMotorId, MotorType.kBrushless);
    m_rearRightTurnMotor = new CANSparkMax(DriveConstants.kRearRightTurnMotorId, MotorType.kBrushless);

    m_frontLeftTurnMotor.restoreFactoryDefaults();
    m_frontRightTurnMotor.restoreFactoryDefaults();
    m_rearLeftTurnMotor.restoreFactoryDefaults();
    m_rearRightTurnMotor.restoreFactoryDefaults();

    m_frontLeftTurnMotor.setIdleMode(IdleMode.kBrake);
    m_frontRightTurnMotor.setIdleMode(IdleMode.kBrake);
    m_rearLeftTurnMotor.setIdleMode(IdleMode.kBrake);
    m_rearRightTurnMotor.setIdleMode(IdleMode.kBrake);

    m_frontLeftTurnEncoder = m_frontLeftTurnMotor.getEncoder();
    m_frontRightTurnEncoder = m_frontRightTurnMotor.getEncoder();
    m_rearLeftTurnEncoder = m_rearLeftTurnMotor.getEncoder();
    m_rearRightTurnEncoder = m_rearRightTurnMotor.getEncoder();

    m_leftDrive = new MotorControllerGroup(m_frontLeftDriveMotor, m_rearLeftDriveMotor);
    m_rightDrive = new MotorControllerGroup(m_frontRightDriveMotor, m_rearRightDriveMotor);
    m_driveTrain = new DifferentialDrive(m_leftDrive, m_rightDrive);

    m_leftDrive.setInverted(DriveConstants.kLeftDriveInverted);
    m_rightDrive.setInverted(DriveConstants.kRightDriveInverted);

    m_driveTrain.setDeadband(OperatorConstants.kDriverControllerDeadband);
    m_driveTrain.setSafetyEnabled(false);

    m_frontLeftTurnPIDController = m_frontLeftTurnMotor.getPIDController();
    m_frontRightTurnPIDController = m_frontRightTurnMotor.getPIDController();
    m_rearLeftTurnPIDController = m_rearLeftTurnMotor.getPIDController();
    m_rearRightTurnPIDController = m_rearRightTurnMotor.getPIDController();

    m_frontLeftTurnPIDController.setP(DriveConstants.kP);
    m_frontLeftTurnPIDController.setI(DriveConstants.kI);
    m_frontLeftTurnPIDController.setD(DriveConstants.kD);
    m_frontLeftTurnPIDController.setFF(DriveConstants.kFF);

    m_frontRightTurnPIDController.setP(DriveConstants.kP);
    m_frontRightTurnPIDController.setI(DriveConstants.kI);
    m_frontRightTurnPIDController.setD(DriveConstants.kD);
    m_frontRightTurnPIDController.setFF(DriveConstants.kFF);

    m_rearLeftTurnPIDController.setP(DriveConstants.kP);
    m_rearLeftTurnPIDController.setI(DriveConstants.kI);
    m_rearLeftTurnPIDController.setD(DriveConstants.kD);
    m_rearLeftTurnPIDController.setFF(DriveConstants.kFF);

    m_rearRightTurnPIDController.setP(DriveConstants.kP);
    m_rearRightTurnPIDController.setI(DriveConstants.kI);
    m_rearRightTurnPIDController.setD(DriveConstants.kD);
    m_rearRightTurnPIDController.setFF(DriveConstants.kFF);

    m_frontLeftTurnPIDController.setFeedbackDevice(m_frontLeftTurnEncoder);
    m_frontRightTurnPIDController.setFeedbackDevice(m_frontRightTurnEncoder);
    m_rearLeftTurnPIDController.setFeedbackDevice(m_rearLeftTurnEncoder);
    m_rearRightTurnPIDController.setFeedbackDevice(m_rearRightTurnEncoder);
  }

  @Override
  public void periodic() {
    m_frontLeftTurnPIDController.setReference(m_desiredAngle, ControlType.kPosition);
    m_frontRightTurnPIDController.setReference(m_desiredAngle, ControlType.kPosition);
    m_rearLeftTurnPIDController.setReference(m_desiredAngle, ControlType.kPosition);
    m_rearRightTurnPIDController.setReference(m_desiredAngle, ControlType.kPosition);

    SmartDashboard.putNumber("Desired Angle", m_desiredAngle);

    SmartDashboard.putNumber("Front Left Encoder", m_frontLeftTurnEncoder.getPosition());
    SmartDashboard.putNumber("Front Right Encoder", m_frontRightTurnEncoder.getPosition());
    SmartDashboard.putNumber("Rear Left Encoder", m_rearLeftTurnEncoder.getPosition());
    SmartDashboard.putNumber("Rear Right Encoder", m_rearRightTurnEncoder.getPosition());
  }

  public void drive(double ySpeed, double rotSpeed, double pov) {
    if (pov == -1 || pov == 0) {
      m_desiredAngle = 0;

      m_driveTrain.arcadeDrive(
        ySpeed * DriveConstants.kPowerPercent,
        rotSpeed * DriveConstants.kAngularPowerPercent
      );
    } else if (pov == 270) {
      m_desiredAngle = -3;

      m_driveTrain.arcadeDrive(
        0.6,
        0
      );
    } else if (pov == 90) {
      m_desiredAngle = -3;

      m_driveTrain.arcadeDrive(
        -0.6,
        0
      );
    }
  }

  public void resetEncoders() {
    m_frontLeftTurnEncoder.setPosition(0);
    m_frontRightTurnEncoder.setPosition(0);
    m_rearLeftTurnEncoder.setPosition(0);
    m_rearRightTurnEncoder.setPosition(0);
  }
}