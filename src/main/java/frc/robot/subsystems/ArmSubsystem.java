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
  Encoder m_raiseEncoder;
  Encoder m_extensionEncoder;

  public ArmSubsystem() {
    m_raiseMotor = new VictorSPX(ArmConstants.kRaiseMotorId);
    m_extensionMotor = new VictorSPX(ArmConstants.kExtensionMotorId);
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
  }

  public void operateArm(double pov) {
    if (pov == 0) {
      m_raiseMotor.set(ControlMode.PercentOutput, ArmConstants.kRaiseMotorPowerPercent);
    } else if (pov == 180) {
      m_raiseMotor.set(ControlMode.PercentOutput, -ArmConstants.kRaiseMotorPowerPercent);
    } else {
      m_raiseMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Raise Encoder Value", m_raiseEncoder.getDistance());
    SmartDashboard.putNumber("Extention Encoder Value", m_extensionEncoder.getDistance());
  }

  public void extendArm() {
    m_extensionMotor.set(ControlMode.PercentOutput, ArmConstants.kExtensionPowerPercent);
  }

  public void retractArm() {
    m_extensionMotor.set(ControlMode.PercentOutput, -ArmConstants.kExtensionPowerPercent);
  }

  public void stopExtensionMotor() {
    m_extensionMotor.set(ControlMode.PercentOutput, 0);
  }
}
