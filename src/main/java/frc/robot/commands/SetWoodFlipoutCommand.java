// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WoodFlipoutConstants;
import frc.robot.subsystems.Mechanisms.WoodFlipoutSubsystem;

// Raises/Lowers Wood Flipout
public class SetWoodFlipoutCommand extends Command {

  // Uses Wood Flipout Subsystems
  WoodFlipoutSubsystem m_woodFlipoutSubsystem;
  String m_position;

  // Constructor
  public SetWoodFlipoutCommand(WoodFlipoutSubsystem woodFlipoutSubsystem, String position) {
        
    // Definitions and setting parameters are equal to members!
    m_woodFlipoutSubsystem = woodFlipoutSubsystem;
    addRequirements(woodFlipoutSubsystem);

    // Position
    m_position = position;
  }

  // Reset timer when the command starts executing
  public void initialize() {
  }
  
  // Actual command
  public void execute() {

    // Lower to Intake Position
    if (m_position.equals("INTAKE")) {
      m_woodFlipoutSubsystem.setPosition(WoodFlipoutConstants.k_woodFlipoutIntakeAngle);
    }

    // Lower to Scoring Position
    if(m_position.equals("SCORE")) {
      m_woodFlipoutSubsystem.setPosition(WoodFlipoutConstants.k_woodFlipoutScoreAngle);
    }

    // Lower to Limelight Position
    if(m_position.equals("LIMELIGHT")) {
      m_woodFlipoutSubsystem.setPosition(WoodFlipoutConstants.k_woodFlipoutLimelightAngle);
    }

    // Raise to Stowed Position
    if(m_position.equals("HOME")) {
      m_woodFlipoutSubsystem.setPosition(WoodFlipoutConstants.k_woodFlipoutHomeAngle);
    }

    // Force Intake to Hard Stop Position
    if(m_position.equals("RESET")) {
      m_woodFlipoutSubsystem.setPosition(WoodFlipoutConstants.k_woodFlipoutResetAngle);
    }
  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {
  }

  // Checks if the command is done
  public boolean isFinished() {

    // Am I done?  Am I done? Am I finally done?
    return true;
  }
}
