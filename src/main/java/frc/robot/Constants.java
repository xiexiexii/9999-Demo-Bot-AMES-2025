// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;

public final class Constants {

  // Drive Subsystem stuff
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kMaxAngularSpeed = 1.5 * Math.PI; // radians per second

    // Chassis configuration
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5);
      
    // Locations of each swerve module
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // DRIVE CAN IDs
    public static final int kFrontLeftDrivingCanId = 6;
    public static final int kRearLeftDrivingCanId = 2;
    public static final int kFrontRightDrivingCanId = 8;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 9;
    public static final int kFrontRightTurningCanId = 7;
    public static final int kRearRightTurningCanId = 3;

    // Gyro inverts
    public static final boolean kGyroReversed = true;
  }

  // Module Constants for MAXSwerve Modules
  public static final class ModuleConstants {

    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
      / kDrivingMotorReduction;
  }

  // The one random constant that tells you how fast a NEO VORTEX goes
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }

  // Controller Ports, Deadband, Buttons and Triggers
  public static final class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;

    public final static int k_start = Button.kStart.value; // Start Button
    public final static int k_back = Button.kBack.value; // Back Button

    public static final int k_A = Button.kA.value; // A
    public static final int k_B = Button.kB.value; // B
    public static final int k_X = Button.kX.value; // X
    public static final int k_Y = Button.kY.value; // Y

    public static final int k_dpadup = 0; // D-Pad Up
    public static final int k_dpadRight = 90; // D-Pad Right
    public static final int k_dpadDown = 180; // D-Pad Down
    public static final int k_dpadLeft = 270; // D-Pad Left

    public final static int k_rightbump = Button.kRightBumper.value; // Right Bump
    public final static int k_leftbump = Button.kLeftBumper.value; // Left Bump

    public final static int k_righttrig = Axis.kRightTrigger.value; // Right Trig
    public final static int k_lefttrig = Axis.kLeftTrigger.value; // Left Trig
  }

  // Motor CAN IDs
  public static final class MotorIDConstants {
    
    public static final int k_intakeMotorID = 11;

    public static final int k_feederMotorID = 21;
    public static final int k_indexerMotorID = 22;

    public static final int k_shooterMotorID = 31;

    public static final int k_woodFlipoutMotorID = 41;
    public static final int k_woodIntakeMotorID = 42;
  }

  // Sensor CAN IDs
  public static final class SensorIDConstants {
    
    public static final int k_intakeCANRangeID = 51;
  }

  // Motor Speed Constants
  public static final class MotorSpeedConstants {

    // Intake
    public static final double k_intakeSpeed = 0.78;

    // Wood Intake
    public static final double k_woodIntakeSpeed = 0.40; // 0.235
    public static final double k_woodIntakeAutoSpeed = 0.27;

    public static final double k_woodShooterSpeed = 0.40;

    // Feeder
    public static final double k_feederSpeed = 0.65;

    // Indexer
    public static final double k_indexerSpeed = 0.5;

    // Shooter 
    public static final double k_shooterSpeed = 0.70;
    public static final double k_shooterSpeedSnipe = 0.95;
    public static final double k_shooterIndexSpeed = 0.05;
  }

  // PID stuff
  public static final class WoodFlipoutConstants {

    // Wood Flipout
    public static final double k_woodFlipoutP = 0.35; 
    public static final double k_woodFlipoutI = 0.1;
    public static final double k_woodFlipoutD = 0.0;
    public static final double k_woodFlipoutS = 0.4;
    public static final double k_woodFlipoutV = 0.001;
    public static final double k_woodFlipoutA = 0.0;
    public static final double k_woodFlipoutG = 0.5;

    public static final Angle k_woodFlipoutResetAngle = edu.wpi.first.units.Units.Rotations.of(-2);
    public static final Angle k_woodFlipoutHomeAngle = edu.wpi.first.units.Units.Rotations.of(0);
    public static final Angle k_woodFlipoutLimelightAngle = edu.wpi.first.units.Units.Rotations.of(5);
    public static final Angle k_woodFlipoutScoreAngle = edu.wpi.first.units.Units.Rotations.of(11); 
    public static final Angle k_woodFlipoutIntakeAngle = edu.wpi.first.units.Units.Rotations.of(21.3);
  }

  // Vision Constants
  public static final class VisionConstants {

    // Camera Name
    public static String k_intakeLimelightName = "limelight-intake";
    public static String k_shooterLimelightName = "limelight-three";

    // PID Stuff
    public static final double kP_aim = 0.005; 
    public static final double kI_aim = 0.000;
    public static final double kD_aim = 0.000;

    public static final double kP_range = 0.01; // 0.3
    public static final double kI_range = 0.000;
    public static final double kD_range = 0.000;

    public static final double k_aimThreshold = 0.5;
    public static final double k_rangeThreshold = 0.5;
  }

  // Pneumatics stuff
  public static final class PneumaticConstants {
    public static final int k_solenoidIDFlipTakeLeftForwardChannel = 0;
    public static final int k_solenoidIDFlipTakeLeftReverseChannel = 1;
    public static final int k_solenoidIDFlipTakeRightForwardChannel = 2;
    public static final int k_solenoidIDFlipTakeRightReverseChannel = 3;
  }

  // Time Constants
  public static final class TimeConstants {
    public static final double k_spinUpTime = 1.0;
    public static final double k_shootTime = 3.0;

    public static final double k_holdWoodTime = 0.2;
  }

  public static final class LiveConstants {

    // Intake Stuff
    public static int _intakeCounter = 0;
    public static boolean _lastIsDetected = false;

    // Limelight Stuff
    public static boolean _positioned = false;
    public static boolean _positioning = false;
  }
}