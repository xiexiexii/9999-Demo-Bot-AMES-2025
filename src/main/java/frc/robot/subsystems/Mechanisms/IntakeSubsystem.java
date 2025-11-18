package frc.robot.subsystems.Mechanisms;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.VortexConfigs;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorSpeedConstants;

public class IntakeSubsystem extends SubsystemBase{
  
  // Motors - [NEO Vortex + SparkFLEX] x1
  SparkFlex m_intake;

  public IntakeSubsystem() {
    
    m_intake = new SparkFlex(MotorIDConstants.k_intakeMotorID, MotorType.kBrushless);

    // TODO: SET CAN ID (21)
    m_intake.configure(VortexConfigs.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Intake Stuff
  public void intake() {
    m_intake.set(MotorSpeedConstants.k_intakeSpeed);
  }

  public void stopShooter() {
    m_intake.set(0);
  }

  public void reverseIntake() {
    m_intake.set(-MotorSpeedConstants.k_intakeSpeed);
  }

  public void periodic() {}
}
