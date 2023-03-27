package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  VictorSPX m_raiseMotor;
  VictorSPX m_extensionMotor;
  VictorSPX m_pivotMotor;
  Encoder m_raiseEncoder;
  Encoder m_extensionEncoder;
  Encoder m_pivotEncoder;

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
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Raise Encoder Value", m_raiseEncoder.getDistance());
    SmartDashboard.putNumber("Extention Encoder Value", m_extensionEncoder.getDistance());
    SmartDashboard.putNumber("Pivot Encoder Value", m_pivotEncoder.getDistance());
  }

  public void operateArm(double raiseValue, double pov) {
    m_raiseMotor.set(ControlMode.PercentOutput, raiseValue * ArmConstants.kRaiseMotorPowerPercent);

    if (pov == 0) { // Up
      m_pivotMotor.set(ControlMode.PercentOutput, ArmConstants.kPivotMotorPowerPercent);
    } else if (pov == 180) { // Down
      m_pivotMotor.set(ControlMode.PercentOutput, -ArmConstants.kPivotMotorPowerPercent);
    } else {
      m_pivotMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void extendArm() {
    m_extensionMotor.set(ControlMode.PercentOutput, ArmConstants.kExtensionMotorPowerPercent);
  }

  public void retractArm() {
    m_extensionMotor.set(ControlMode.PercentOutput, -ArmConstants.kExtensionMotorPowerPercent);
  }

  public void stopExtensionMotor() {
    m_extensionMotor.set(ControlMode.PercentOutput, 0);
  }
}
