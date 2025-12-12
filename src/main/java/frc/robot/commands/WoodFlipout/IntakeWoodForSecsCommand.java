package frc.robot.commands.WoodFlipout;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WoodFlipout.WoodIntakeSubsystem;

public class IntakeWoodForSecsCommand extends Command{
   
    // Instantiate stuff
    WoodIntakeSubsystem m_woodIntakeSubsystem;

    Timer m_timer = new Timer();
    double m_seconds;

    public IntakeWoodForSecsCommand(WoodIntakeSubsystem woodIntakeSubsystem, double seconds) {

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
      m_woodIntakeSubsystem.intake();
    }

    public void end(boolean interrupted){
      m_woodIntakeSubsystem.stopIntake();
    }

    public boolean isFinished(){
      return m_timer.hasElapsed(m_seconds);
    }

}