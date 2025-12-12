package frc.robot.subsystems.Superstructure;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.NeoConfigs;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorSpeedConstants;

public class TowerSubsystem extends SubsystemBase{
  
  // Motors - [NEO v1.1 + SparkMAX] x2
  SparkMax m_feeder;
  SparkMax m_indexer;

  public TowerSubsystem() {
    
    m_feeder = new SparkMax(MotorIDConstants.k_feederMotorID, MotorType.kBrushless);
    m_indexer = new SparkMax(MotorIDConstants.k_indexerMotorID, MotorType.kBrushless);

    m_feeder.configure(NeoConfigs.feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_indexer.configure(NeoConfigs.indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Feed Vegetables to Tower
  public void feed() {
    m_feeder.set(-MotorSpeedConstants.k_feederSpeed);
  }

  // Stop Feeding
  public void stopFeeder() {
    m_feeder.set(0);
  }

  // Reverse Feed
  public void reverseFeeder() {
    m_feeder.set(MotorSpeedConstants.k_feederSpeed);
  }

  // Index Vegetables in Tower
  public void index() {
    m_indexer.set(-MotorSpeedConstants.k_indexerSpeed);
  }

  // Stop Indexing
  public void stopIndexer() {
    m_indexer.set(0);
  }

  // Reverse Index
  public void reverseIndexer() {
    m_indexer.set(MotorSpeedConstants.k_indexerSpeed);
  }

  // Combined Index
  public void indexAll() {
    m_feeder.set(-MotorSpeedConstants.k_feederSpeed);
    m_indexer.set(-MotorSpeedConstants.k_indexerSpeed);
  }

  public void stopAll() {
    m_feeder.set(0);
    m_indexer.set(0);
  }

  public void periodic() {}
}
