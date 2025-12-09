// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WoodFlipoutConstants;
import frc.robot.subsystems.Mechanisms.WoodFlipoutSubsystem;

// Zero Flipout (for the same reason as elevator)
public class ZeroWoodFlipoutCommand extends Command {

  // Uses Wood Flipout Subsystem
  WoodFlipoutSubsystem m_woodFlipoutSubsystem;
  boolean m_finished;

  // Constructor
  public ZeroWoodFlipoutCommand(WoodFlipoutSubsystem woodFlipoutSubsystem) {
        
    // Definitions and setting parameters are equal to members!
    m_woodFlipoutSubsystem = woodFlipoutSubsystem;
    addRequirements(woodFlipoutSubsystem);
  }

  // Reset timer when the command starts executing
  public void initialize() {
    m_finished = false;
  }
  
  // Actual command
  public void execute() {
    if(m_woodFlipoutSubsystem.getCurrentPosition() < 0.5 && m_woodFlipoutSubsystem.getCurrentVelocity() == 0) { // TODO: CHECK NUMBERS
      m_finished = true;
    }
  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {

    // Neutral Motors and Reset Encoder Values
    m_woodFlipoutSubsystem.setNeutral();
    m_woodFlipoutSubsystem.resetSensorPosition(WoodFlipoutConstants.k_woodFlipoutHomeAngle);
  }

  // Checks if the command is done
  public boolean isFinished() {

    // Am I done?  Am I done? Am I finally done?
    return m_finished;
  }
}
