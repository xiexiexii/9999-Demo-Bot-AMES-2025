package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LiveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelight.LimelightHelpers;
import frc.robot.subsystems.Swerve.DriveSubsystem;

// Positions Robot to Shooting Position for Stove
public class ShootAutoPositionCommand extends Command {
    
  // Instantiate Stuff
  DriveSubsystem m_swerveSubsystem;

  // Timer for cancellation with failed robot adjustment
  Timer timer = new Timer();

  // PID Controller stuff (woah so many so scary) 
  PIDController m_aimController = new PIDController(VisionConstants.kP_aim, VisionConstants.kI_aim, VisionConstants.kD_aim);
  PIDController m_rangeController = new PIDController(VisionConstants.kP_range, VisionConstants.kI_range, VisionConstants.kD_range);

  /*
   * Tag Guide (Perspective is from collective DS):
   * 1: Cabin Outside Red Left
   * 2: Cabin Outside Blue Right
   * 3: Wood Shed Red Left
   * 4: Wood Shed Blue Right
   * 5: Veg Garden Red Right
   * 6: Veg Garden Blue Left
   * 7: Stove Internal Red Left
   * 8: Stove Internal Blue Right
   * 9: Stove Upper Red Left
   * 10: Stove Upper Blue Right
   * 11: Driver Station Middle
   */

  // All the Valid IDs available for positioning
  int[] validIDs = {9, 10};

  // Boolean for checking for "Tag In View" 
  private boolean tiv;

  // Constructor
  public ShootAutoPositionCommand(DriveSubsystem driveSubsystem) {
        
    // Definitions and setting parameters are equal to members!
    m_swerveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // What we do to set up the command 
  public void initialize() {

    // Reset Live Constants
    LiveConstants._positioned = false;

    // Adds condition that filters out undesired IDs
    LimelightHelpers.SetFiducialIDFiltersOverride(VisionConstants.k_shooterLimelightName, validIDs);

    // Checks for TIV
    tiv = (LimelightHelpers.getTV(VisionConstants.k_shooterLimelightName));

    // Timer Reset
    timer.start();
    timer.reset();
  }
    
  // The actual control!
  public void execute() {

    LiveConstants._positioning = true;

    // Checks for a continued valid pose
    if (tiv) {
      tiv = LimelightHelpers.getTV(VisionConstants.k_shooterLimelightName);
      m_swerveSubsystem.drive(limelight_range_PID(), 0, limelight_aim_PID(), false);
    }
  }

  // Add stuff we do after to reset here (a.k.a tag filters)
  public void end(boolean interrupted) {
    LiveConstants._positioning = false;

    // Set Positioned Flag if positioned
    if (
      // Range (Distance to Tag)
      Math.abs(LimelightHelpers.getTY(VisionConstants.k_shooterLimelightName)) < VisionConstants.k_rangeThreshold &&
      // Aim (Angle)
      Math.abs(LimelightHelpers.getTX(VisionConstants.k_shooterLimelightName)) < VisionConstants.k_aimThreshold) {
        
        LiveConstants._positioned = true;
    }
  }

  // Are we done yet? Finishes when threshold is reached or if no tag in view or if timer is reached 
  public boolean isFinished() {
    return (
      // Range (Distance to Tag)
      Math.abs(LimelightHelpers.getTY(VisionConstants.k_shooterLimelightName)) < VisionConstants.k_rangeThreshold &&
      // Aim (Angle)
      Math.abs(LimelightHelpers.getTX(VisionConstants.k_shooterLimelightName)) < VisionConstants.k_aimThreshold

      || !tiv || timer.get() > 2
    );
  }

  // PID-assisted ranging control with Limelight's TY value from target-relative data
  private double limelight_range_PID() {

    // Limelight Z Axis Range in Degrees
    m_rangeController.enableContinuousInput(-30, 30); // TODO: ADJUST ME
    
    // Calculates response based on difference in distance from tag to robot
    double targetingForwardSpeed = m_rangeController.calculate(LimelightHelpers.getTY(VisionConstants.k_shooterLimelightName));

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
    double targetingAngularVelocity = m_aimController.calculate(LimelightHelpers.getTX(VisionConstants.k_shooterLimelightName));

    // Multiply by 1 because robot is CCW Positive. Multiply by a reduction 
    // multiplier to reduce speed. Scale TX up with robot speed.
    targetingAngularVelocity *= DriveConstants.kMaxAngularSpeed;

    // Hooray
    return targetingAngularVelocity;
  }
}
