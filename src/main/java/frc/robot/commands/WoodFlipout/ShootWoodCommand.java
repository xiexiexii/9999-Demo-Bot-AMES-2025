package frc.robot.commands.WoodFlipout;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TimeConstants;
import frc.robot.subsystems.WoodFlipout.WoodIntakeSubsystem;

public class ShootWoodCommand extends Command{
   
    // Instantiate stuff
    WoodIntakeSubsystem m_woodIntakeSubsystem;

    Timer m_timer = new Timer();
    double m_seconds;
    double m_holdWoodSeconds = TimeConstants.k_holdWoodTime;

    boolean m_isFinished = false;

    public ShootWoodCommand(WoodIntakeSubsystem woodIntakeSubsystem, double seconds) {

      // Definitions and setting parameters equal to members
      m_woodIntakeSubsystem = woodIntakeSubsystem;
      m_seconds = seconds;

      addRequirements(woodIntakeSubsystem);
    }


    public void initialize() {
      m_timer.start();
      m_timer.reset();
    }
        

    public void execute() {
      if (m_timer.hasElapsed(m_holdWoodSeconds)) m_woodIntakeSubsystem.reverseIntake();

      if (m_timer.hasElapsed(m_seconds)) {

      }
    }

    public void end(boolean interrupted){
      m_woodIntakeSubsystem.stopIntake();
    }

    public boolean isFinished(){
      return m_timer.hasElapsed(m_seconds);
    }

}