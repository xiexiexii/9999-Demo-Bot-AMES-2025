package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelight.LimelightHelpers;
import frc.robot.subsystems.Swerve.DriveSubsystem;


// Positions Robot at the Nearest Valid Reef
public class IntakeAutoPositionCommand extends Command {
    
  // Instantiate Stuff
  DriveSubsystem m_swerveSubsystem;

  // Timer for cancellation with failed robot adjustment
  Timer timer = new Timer();

  // PID Controller stuff (woah so many so scary) 
  PIDController m_aimController = new PIDController(VisionConstants.kP_aim, VisionConstants.kI_aim, VisionConstants.kD_aim);
  PIDController m_rangeController = new PIDController(VisionConstants.kP_range, VisionConstants.kI_range, VisionConstants.kD_range);

  // Lil boolean for checking for "Target In View" 
  private boolean tiv;

  // Constructor
  public IntakeAutoPositionCommand(DriveSubsystem driveSubsystem) {
        
    // Definitions and setting parameters are equal to members!
    m_swerveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // What we do to set up the command 
  public void initialize() {

    // Checks for TIV
    tiv = (LimelightHelpers.getTV(VisionConstants.k_intakeLimelightName)
      && LimelightHelpers.getDetectorClass(VisionConstants.k_intakeLimelightName).equals("vegetable")
    );

    // Timer Reset
    timer.start();
    timer.reset();
  }
    
  // The actual control!
  public void execute() {

    VisionConstants.k_positioning = true;

    // Checks for a continued valid pose
    if (tiv) {
      tiv = LimelightHelpers.getTV(VisionConstants.k_intakeLimelightName)
        && LimelightHelpers.getDetectorClass(VisionConstants.k_intakeLimelightName).equals("vegetable");
      m_swerveSubsystem.drive(limelight_range_PID(), 0, limelight_aim_PID(), false);
    }
  }

  // Add stuff we do after to reset here (a.k.a tag filters)
  public void end(boolean interrupted) {
    VisionConstants.k_positioning = false;
  }

  // Are we done yet? Finishes when threshold is reached or if no tag in view or if timer is reached 
  public boolean isFinished() {
    return (
      !tiv || timer.get() > 2
    );
  }

  // PID-assisted ranging control with Limelight's TY value from target-relative data
  private double limelight_range_PID() {

    // Limelight Z Axis Range in meters
    m_rangeController.enableContinuousInput(-5, 50);
    
    // Calculates response based on difference in distance from tag to robot
    double targetingForwardSpeed = m_rangeController.calculate(LimelightHelpers.getTY(VisionConstants.k_intakeLimelightName));

    // Value scale up to robot max speed and invert (double cannot exceed 1.0)
    targetingForwardSpeed *= -1 * DriveConstants.kMaxSpeedMetersPerSecond;

    // Hooray
    return targetingForwardSpeed;
  }

  // PID-assisted ranging control with Limelight's TX value from target-relative data
  private double limelight_aim_PID() {

    // Limelight Yaw Angle in Degrees
    m_aimController.enableContinuousInput(-41, 41);
    
    // Calculates response based on difference in angle from tag to robot
    double targetingAngularVelocity = m_aimController.calculate(LimelightHelpers.getTX(VisionConstants.k_intakeLimelightName));

    // Multiply by -1 because robot is CCW Positive. Multiply by a reduction 
    // multiplier to reduce speed. Scale TX up with robot speed.
    targetingAngularVelocity *= DriveConstants.kMaxAngularSpeed;

    // Hooray
    return targetingAngularVelocity;
  }
}
