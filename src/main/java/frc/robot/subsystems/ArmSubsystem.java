package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private VictorSPX m_raiseMotor;
  private VictorSPX m_extensionMotor;
  private VictorSPX m_pivotMotor;
  private Encoder m_raiseEncoder;
  private Encoder m_extensionEncoder;
  private Encoder m_pivotEncoder;
  private DigitalInput m_extensionLimitSwitch;
  private DigitalInput m_raiseLimitSwitch;
  private DigitalInput m_pivotUpperLimitSwitch;
  private DigitalInput m_pivotLowerLimitSwitch;
  private double m_desiredExtensionPower = 0;

  public ArmSubsystem() {
    m_raiseMotor = new VictorSPX(ArmConstants.kRaiseMotorId);
    m_extensionMotor = new VictorSPX(ArmConstants.kExtensionMotorId);
    m_pivotMotor = new VictorSPX(ArmConstants.kPivotMotorId);

    m_raiseEncoder = new Encoder(
      ArmConstants.kRaiseEncoderChannelA,
      ArmConstants.kRaiseEncoderChannelB,
      ArmConstants.kRaiseEncoderInverted,
      ArmConstants.kRaiseEncoderEncodingType);
    m_extensionEncoder = new Encoder(
      ArmConstants.kExtensionEncoderChannelA,
      ArmConstants.kExtensionEncoderChannelB,
      ArmConstants.kExtensionEncoderInverted,
      ArmConstants.kExtensionEncoderEncodingType);
    m_pivotEncoder = new Encoder(
      ArmConstants.kPivotEncoderChannelA,
      ArmConstants.kPivotEncoderChannelB,
      ArmConstants.kPivotEncoderInverted,
      ArmConstants.kPivotEncoderEncodingType
    );

    m_extensionLimitSwitch = new DigitalInput(ArmConstants.kExtensionLimitSwitchId);
    m_raiseLimitSwitch = new DigitalInput(ArmConstants.kRaiseLimitSwitchId);
    m_pivotUpperLimitSwitch = new DigitalInput(ArmConstants.kPivotUpperLimitSwitchId);
    m_pivotLowerLimitSwitch = new DigitalInput(ArmConstants.kPivotLowerLimitSwitchId);
  }

  @Override
  public void periodic() {
    if (m_desiredExtensionPower < 0 && m_extensionLimitSwitch.get()) {
      m_extensionMotor.set(ControlMode.PercentOutput, 0);
    } else {
      m_extensionMotor.set(ControlMode.PercentOutput, m_desiredExtensionPower);
    }

    SmartDashboard.putNumber("Raise Encoder Value", m_raiseEncoder.getDistance());
    SmartDashboard.putNumber("Extension Encoder Value", m_extensionEncoder.getDistance());
    SmartDashboard.putNumber("Pivot Encoder Value", m_pivotEncoder.getDistance());
  }

  public void operateArm(double raiseValue, boolean extensionValue, double retractionValue, double pov) {
    double desiredRaiseMotorPower = raiseValue * ArmConstants.kRaiseMotorPowerPercent;

    // Raise motor
    if (desiredRaiseMotorPower > 0 && m_raiseLimitSwitch.get()) {
      m_raiseMotor.set(ControlMode.PercentOutput, 0);
    } else {
      m_raiseMotor.set(ControlMode.PercentOutput, desiredRaiseMotorPower);
    }

    // Extension motor
    if (extensionValue) {
      extendArm();
    } else {
      retractArm();
    }

    // Pivot motor
    if (pov == 0) { // Up
      if (m_pivotUpperLimitSwitch.get()) {
        m_pivotMotor.set(ControlMode.PercentOutput, 0);
      } else {
        m_pivotMotor.set(ControlMode.PercentOutput, -ArmConstants.kPivotMotorPowerPercent);
      }
    } else if (pov == 180) { // Down
      if (m_pivotLowerLimitSwitch.get()) {
        m_pivotMotor.set(ControlMode.PercentOutput, 0);
      } else {
        m_pivotMotor.set(ControlMode.PercentOutput, ArmConstants.kPivotMotorPowerPercent);
      }
    } else {
      m_pivotMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void extendArm() {
    m_desiredExtensionPower = ArmConstants.kExtensionMotorPowerPercent;
  }

  public void retractArm() {
    m_desiredExtensionPower = -ArmConstants.kExtensionMotorPowerPercent;
  }

  public void stopExtensionMotor() {
    m_desiredExtensionPower = 0;
  }
}
