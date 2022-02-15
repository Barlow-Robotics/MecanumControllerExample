// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  public static final double RadiansToDegrees = 180.0 / Math.PI ;
  public static final double DegreesToRadians = Math.PI / 180.0 ;
  public static final double Inches_Per_Foot = 12.0 ;
  public static final double InchesToMeters = 0.0254 ;



  public static final class DriveConstants {
    public static final int ID_frontLeftMotor = 4;
    public static final int ID_backLeftMotor = 5;
    public static final int ID_frontRightMotor = 1;
    public static final int ID_backRightMotor = 2;

    //public static final double countsPerRevolution = 8192.0;
    public static final double circumferenceOfWheel = 6.0 * Math.PI;

    // wpk need to measure and update.
    public static final int Counts_Per_Revolution = 21300 ;
    public static final double Wheel_Diameter = 6.0 ;
    public static final double Inches_Per_Revolution = Math.PI * Wheel_Diameter ;
    public static final double Meters_Per_Revolution = Inches_Per_Revolution * InchesToMeters ;
    public static final double Meters_Per_Count = Meters_Per_Revolution / Counts_Per_Revolution ; 

    // wpk need to measure and update.
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 28 * InchesToMeters ;

    // wpk need to measure and update.
    // Distance between centers of front and back wheels on robot
    public static final double kWheelBase = 20.5 * InchesToMeters;

    public static final MecanumDriveKinematics kDriveKinematics 
        = new MecanumDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), 
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
            );

    // public static final int kEncoderCPR = 1024;
    // public static final double kWheelDiameterMeters = 0.15;
    // public static final double kEncoderDistancePerPulse =
    //         // Assumes the encoders are directly mounted on the wheel shafts
    //         (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The RobotPy Characterization Toolsuite provides a convenient tool for
    // obtaining these
    // values for your robot.
    public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(1, 0.8, 0.15);

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPFrontLeftVel = 0.75;
    public static final double kPRearLeftVel = 0.75;
    public static final double kPFrontRightVel = 0.75;
    public static final double kPRearRightVel = 0.75;

    // Drivetrain
    public static final double GearRatio = 48/1;

    public static final double drivetrainMinPower = 0.05;
    public static final double drivetrainMaxPower = 1.0;
    public static final double manualVoltageRampingConstant = 0.21;
    public static final double closedVoltageRampingConstant = 0.21;

    public static final int PID_id = 0;
    public static final double PID_Period = 1.0 / 20.0;
    public static final double DrivetrainKf = 1.8; // 0.1797
    public static final double DrivetrainkP = 0.02;

    public static final double UnitsPerRotation = 1024;
    public static final double RPMsToUnitsPerHundredMilliseconds = 1.0 / 600.0;
    public static final double DesiredRPMsForDrive = 560.0;
    public static final double MaxDriveVelocity = 6000.0;
    public static final double VelocityInputConversionFactor = DesiredRPMsForDrive * UnitsPerRotation
            * RPMsToUnitsPerHundredMilliseconds;

    public static final int encoderTimeout = 30;
    public static final int mainFeedbackLoop = 0;

    public static final double autoBackingDistance = 3.5; // 3.5 rotations of the wheel ~ 65"
    public static final double pathFollowingThreshold = 20;
    public static final int autonomousDriveTime = 2500;
    public static final double autonomousDriveSpeed = 0.7;
    public static final double autonomousTurnRate = 0.7;

    public static final double speedConstantForBallChase = 0.3;
    public static final double maxAngleChangeForAlignFinish = 0.5;
    public static final double maxAngleDifferenceBetweenNavXAndVision = 0.01;
    public static final double alignTimeoutTime = 1000;
    public static final double alignMemorySize = 3;

    public static final double kFrontLeft_x = 3; //feet (change)
    public static final double kFrontLeft_y = 3; //feet (change)
    public static final double kFrontRight_x = 3; //feet (change)
    public static final double kFrontRight_y = 3; //feet (change)
    public static final double kBackLeft_x = 3; //feet (change)
    public static final double kBackLeft_y = 3; //feet (change)
    public static final double kBackRight_x = 3; //feet (change)
    public static final double kBackRight_y = 3; //feet (change) 
}


  // public static final class DriveConstants {
  //   public static final int kFrontLeftMotorPort = 0;
  //   public static final int kRearLeftMotorPort = 1;
  //   public static final int kFrontRightMotorPort = 2;
  //   public static final int kRearRightMotorPort = 3;

  //   public static final int[] kFrontLeftEncoderPorts = new int[] {0, 1};
  //   public static final int[] kRearLeftEncoderPorts = new int[] {2, 3};
  //   public static final int[] kFrontRightEncoderPorts = new int[] {4, 5};
  //   public static final int[] kRearRightEncoderPorts = new int[] {6, 7};

  //   public static final boolean kFrontLeftEncoderReversed = false;
  //   public static final boolean kRearLeftEncoderReversed = true;
  //   public static final boolean kFrontRightEncoderReversed = false;
  //   public static final boolean kRearRightEncoderReversed = true;

  //   public static final double kTrackWidth = 0.5;
  //   // Distance between centers of right and left wheels on robot
  //   public static final double kWheelBase = 0.7;
  //   // Distance between centers of front and back wheels on robot

  //   public static final MecanumDriveKinematics kDriveKinematics =
  //       new MecanumDriveKinematics(
  //           new Translation2d(kWheelBase / 2, kTrackWidth / 2),
  //           new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
  //           new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
  //           new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

  //   public static final int kEncoderCPR = 1024;
  //   public static final double kWheelDiameterMeters = 0.15;
  //   public static final double kEncoderDistancePerPulse =
  //       // Assumes the encoders are directly mounted on the wheel shafts
  //       (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

  //   // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
  //   // These characterization values MUST be determined either experimentally or theoretically
  //   // for *your* robot's drive.
  //   // The SysId tool provides a convenient method for obtaining these values for your robot.
  //   public static final SimpleMotorFeedforward kFeedforward =
  //       new SimpleMotorFeedforward(1, 0.8, 0.15);

  //   // Example value only - as above, this must be tuned for your drive!
  //   public static final double kPFrontLeftVel = 0.5;
  //   public static final double kPRearLeftVel = 0.5;
  //   public static final double kPFrontRightVel = 0.5;
  //   public static final double kPRearRightVel = 0.5;
  // }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
