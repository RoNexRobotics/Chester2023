package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
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

  public DriveSubsystem() {
    m_frontLeftDriveMotor = new CANSparkMax(DriveConstants.kFrontLeftDriveMotorId, MotorType.kBrushless);
    m_frontLeftTurnMotor = new CANSparkMax(DriveConstants.kFrontLeftTurnMotorId, MotorType.kBrushless);
    m_frontRightDriveMotor = new CANSparkMax(DriveConstants.kFrontRightDriveMotorId, MotorType.kBrushless);
    m_frontRightTurnMotor = new CANSparkMax(DriveConstants.kFrontRightTurnMotorId, MotorType.kBrushless);
    m_rearLeftDriveMotor = new CANSparkMax(DriveConstants.kRearLeftDriveMotorId, MotorType.kBrushless);
    m_rearLeftTurnMotor = new CANSparkMax(DriveConstants.kRearLeftTurnMotorId, MotorType.kBrushless);
    m_rearRightDriveMotor = new CANSparkMax(DriveConstants.kRearRightDriveMotorId, MotorType.kBrushless);
    m_rearRightTurnMotor = new CANSparkMax(DriveConstants.kRearRightTurnMotorId, MotorType.kBrushless);

    m_frontLeftTurnEncoder = m_frontLeftTurnMotor.getEncoder();
    m_frontRightTurnEncoder = m_frontRightTurnMotor.getEncoder();
    m_rearLeftTurnEncoder = m_rearLeftTurnMotor.getEncoder();
    m_rearLeftTurnEncoder = m_rearRightTurnMotor.getEncoder();

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

    m_frontLeftTurnPIDController.setP(ModuleConstants.kP);
    m_frontLeftTurnPIDController.setI(ModuleConstants.kI);
    m_frontLeftTurnPIDController.setD(ModuleConstants.kD);
    m_frontLeftTurnPIDController.setFF(ModuleConstants.kFF);

    m_frontRightTurnPIDController.setP(ModuleConstants.kP);
    m_frontRightTurnPIDController.setI(ModuleConstants.kI);
    m_frontRightTurnPIDController.setD(ModuleConstants.kD);
    m_frontRightTurnPIDController.setFF(ModuleConstants.kFF);

    m_rearLeftTurnPIDController.setP(ModuleConstants.kP);
    m_rearLeftTurnPIDController.setI(ModuleConstants.kI);
    m_rearLeftTurnPIDController.setD(ModuleConstants.kD);
    m_rearLeftTurnPIDController.setFF(ModuleConstants.kFF);

    m_rearRightTurnPIDController.setP(ModuleConstants.kP);
    m_rearRightTurnPIDController.setI(ModuleConstants.kI);
    m_rearRightTurnPIDController.setD(ModuleConstants.kD);
    m_rearRightTurnPIDController.setFF(ModuleConstants.kFF);

    m_frontLeftTurnPIDController.setReference(0, ControlType.kPosition);
    m_frontRightTurnPIDController.setReference(0, ControlType.kPosition);
    m_rearLeftTurnPIDController.setReference(0, ControlType.kPosition);
    m_rearRightTurnPIDController.setReference(0, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Value", m_frontLeftTurnEncoder.getPosition());
  }

  public void drive(double ySpeed, double rotSpeed) {
    System.out.println(ySpeed);
    m_driveTrain.arcadeDrive(
      ySpeed * DriveConstants.kPowerPercent,
      rotSpeed * DriveConstants.kAngularPowerPercent
    );
  }
}