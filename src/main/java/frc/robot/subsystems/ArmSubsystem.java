package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  VictorSPX m_raiseMotor;
  VictorSPX m_extentionMotor;
  Encoder m_raiseEncoder;
  Encoder m_extentionEncoder;

  public ArmSubsystem() {
    m_raiseMotor = new VictorSPX(ArmConstants.kArmRaiseMotorId);
    m_extentionMotor = new VictorSPX(ArmConstants.kArmExtensionMotorId);
    m_raiseEncoder = new Encoder(2, 3);
    m_extentionEncoder = new Encoder(0, 1);
  }

  public void operateArm(double pov) {
    if (pov == 0) {
      m_raiseMotor.set(ControlMode.PercentOutput, 1);
      System.out.println("Raising");
    } else if (pov == 180) {
      m_raiseMotor.set(ControlMode.PercentOutput, -1);
      System.out.println("Lowering");
    } else {
      m_raiseMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Raise Encoder Value", m_raiseEncoder.getDistance());
    SmartDashboard.putNumber("Extention Encoder Value", m_extentionEncoder.getDistance());
  }

  public void extendArm() {
    m_extentionMotor.set(ControlMode.PercentOutput, 0.3);
    System.out.println("Extending");
  }

  public void retractArm() {
    m_extentionMotor.set(ControlMode.PercentOutput, -0.3);
    System.out.println("Retracting");
  }

  public void stopExtentionMotor() {
    m_extentionMotor.set(ControlMode.PercentOutput, 0);
    System.out.println("Stopped!");
  }
}
