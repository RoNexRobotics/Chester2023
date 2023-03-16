package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;

public class DriveSubsystem extends SubsystemBase {
  private CANSparkMax m_frontLeftDriveMotor;
  private CANSparkMax m_frontLeftTurnMotor;
  private CANSparkMax m_frontRightDriveMotor;
  private CANSparkMax m_frontRightTurnMotor;
  private CANSparkMax m_rearLeftDriveMotor;
  private CANSparkMax m_rearLeftTurnMotor;
  private CANSparkMax m_rearRightDriveMotor;
  private CANSparkMax m_rearRightTurnMotor;

  private RelativeEncoder m_frontLeftTurnEncoder;
  private RelativeEncoder m_frontRightTurnEncoder;
  private RelativeEncoder m_rearLeftTurnEncoder;
  private RelativeEncoder m_rearRightTurnEncoder;

  private DifferentialDrive m_driveTrain;
  private MotorControllerGroup m_leftDrive;
  private MotorControllerGroup m_rightDrive;

  private SparkMaxPIDController m_frontLeftTurnPIDController;
  private SparkMaxPIDController m_frontRightTurnPIDController;
  private SparkMaxPIDController m_rearLeftTurnPIDController;
  private SparkMaxPIDController m_rearRightTurnPIDController;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);

  private double m_desiredAngle = 0;
  private boolean m_alignModules = true;

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
    if (m_alignModules) {
      m_frontLeftTurnPIDController.setReference(m_desiredAngle, ControlType.kPosition);
      m_frontRightTurnPIDController.setReference(m_desiredAngle, ControlType.kPosition);
      m_rearLeftTurnPIDController.setReference(m_desiredAngle, ControlType.kPosition);
      m_rearRightTurnPIDController.setReference(m_desiredAngle, ControlType.kPosition);
    }

    SmartDashboard.putNumber("Front Left Encoder", m_frontLeftTurnEncoder.getPosition());
    SmartDashboard.putNumber("Front Right Encoder", m_frontRightTurnEncoder.getPosition());
    SmartDashboard.putNumber("Rear Left Encoder", m_rearLeftTurnEncoder.getPosition());
    SmartDashboard.putNumber("Rear Right Encoder", m_rearRightTurnEncoder.getPosition());
  }

  public void drive(double ySpeed, double rotSpeed, double pov) {
    if (pov == -1 || pov == 0) { // Default or Up
      m_desiredAngle = 0;

      m_driveTrain.arcadeDrive(
        // ySpeed,
        // rotSpeed
        m_magLimiter.calculate(ySpeed),
        m_rotLimiter.calculate(rotSpeed)
      );
    } else if (pov == 270) { // Swerve left
      swerveLeft(DriveConstants.kSwervePowerPercent);
    } else if (pov == 90) { // Swerve right
      swerveRight(DriveConstants.kSwervePowerPercent);
    }
  }

  public void drive(double ySpeed, double rotSpeed) {
    m_desiredAngle = 0;
    
    m_driveTrain.arcadeDrive(
      // ySpeed,
      // rotSpeed
      m_magLimiter.calculate(ySpeed),
      m_rotLimiter.calculate(rotSpeed)
    );
  }

  public void swerveLeft(double speed) {
    m_desiredAngle = -3;
    
    m_driveTrain.arcadeDrive(speed, 0);
  }

  public void swerveRight(double speed) {
    m_desiredAngle = 3;
    
    m_driveTrain.arcadeDrive(speed, 0);
  }

  public void alignModulesEnabled(boolean enabled) {
    m_alignModules = enabled;
  }

  public void resetEncoders() {
    m_frontLeftTurnEncoder.setPosition(0);
    m_frontRightTurnEncoder.setPosition(0);
    m_rearLeftTurnEncoder.setPosition(0);
    m_rearRightTurnEncoder.setPosition(0);
  }
}