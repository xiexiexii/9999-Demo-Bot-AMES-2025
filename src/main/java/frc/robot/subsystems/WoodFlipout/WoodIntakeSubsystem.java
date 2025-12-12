package frc.robot.subsystems.WoodFlipout;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.TalonFXConfigs;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorSpeedConstants;

public class WoodIntakeSubsystem extends SubsystemBase{
  
  // Motors - [VEX Falcon 500 + Talon FX] x1
  TalonFX m_intake;

  public WoodIntakeSubsystem() {
    
    m_intake = new TalonFX(MotorIDConstants.k_woodIntakeMotorID);

    m_intake.getConfigurator().apply(TalonFXConfigs.woodIntakeConfig, 0.05);
  }

  // Intake Stuff
  public void intake() {
    m_intake.set(MotorSpeedConstants.k_woodIntakeSpeed);
  }

  // Stop!
  public void stopIntake() {
    m_intake.set(0);
  }

  // Shoot Wood Into Stove
  public void reverseIntake() {
    m_intake.set(-MotorSpeedConstants.k_woodShooterSpeed);
  }

  public void periodic() {}
}
