package frc.robot.subsystems.Superstructure;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.VortexConfigs;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorSpeedConstants;

public class ShooterSubsystem extends SubsystemBase{
  
  // Motors - [NEO Vortex + SparkFLEX] x1
  SparkFlex m_shooter;

  public ShooterSubsystem() {
    
    m_shooter = new SparkFlex(MotorIDConstants.k_shooterMotorID, MotorType.kBrushless);

    m_shooter.configure(VortexConfigs.shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Shoot Stuff
  public void shoot() {
    m_shooter.set(-MotorSpeedConstants.k_shooterSpeed);
  }

  // For snipe shot in auto
  public void shootSnipe() {
    m_shooter.set(-MotorSpeedConstants.k_shooterSpeedSnipe);
  }

  public void stopShooter() {
    m_shooter.set(0);
  }

  public void reverseShooterIndex() {
    m_shooter.set(MotorSpeedConstants.k_shooterIndexSpeed);
  }

  public void periodic() {}
}
