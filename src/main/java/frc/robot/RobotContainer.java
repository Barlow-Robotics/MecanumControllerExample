// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;

import com.pathplanner.lib.*;
//import com.pathplanner.lib.commands.PPMecanumControllerCommand;
import frc.robot.PPMecanumControllerCommand;
import com.pathplanner.lib.PathPlannerTrajectory.*;

// import java.io.IOException;
// import java.nio.file.Path;
// import java.util.List;
// import frc.robot.Robot;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    Joystick m_driverController = new Joystick(1); // change

    // Creates our Motion Profile Controller and Trajectories class
    // Trajectories trajectories = new Trajectories();

    PathPlannerTrajectory trajectory ;
    List<PathPlannerTrajectory> trajectories ;




    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();


        //trajectories = new List<PathPlannerTrajectory>() ;
        // trajectories.add(PathPlanner.loadPath("0_TarmacB1_to_BBallD", Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        // trajectories.add(PathPlanner.loadPath("1_TarmacB1_to_BBallD_BBallC", Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        // trajectories.add(PathPlanner.loadPath("2_TarmacB2_to_BBallB", Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        // trajectories.add(PathPlanner.loadPath("3_TarmacB2_to_BBallB_BBallC", Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        // trajectories.add(PathPlanner.loadPath("4_TarmacB2_to_BBallC", Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        // trajectories.add(PathPlanner.loadPath("5_TarmacB2_to_BBallC_BBallB", Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        // trajectories.add(PathPlanner.loadPath("6_TarmacB2_to_BBallC_BBallD", Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        // trajectories.add(PathPlanner.loadPath("7_TarmacR1_to_RBallD", Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        // trajectories.add(PathPlanner.loadPath("8_TarmacR1_to_RBallD_RBallE", Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        // trajectories.add(PathPlanner.loadPath("9_TarmacR1_to_RBallE", Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        // trajectories.add(PathPlanner.loadPath("10_TarmacR1_to_RBallE_RBallF", Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        // trajectories.add(PathPlanner.loadPath("11_TarmacR2_to_RBallF", Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        // trajectories.add(PathPlanner.loadPath("12_TarmacR2_to_RBallF_RBallE", Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        // trajectories.add(PathPlanner.loadPath("13_Test_Constant_x", Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        // trajectories.add(PathPlanner.loadPath("14_Test_Constant_y", Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        // trajectories.add(PathPlanner.loadPath("15_Test_Diagonal", Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        // trajectories.add(PathPlanner.loadPath("16_Test_Loop", Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        // trajectories.add(PathPlanner.loadPath("17_Test_Sideways", Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        // trajectories.add(PathPlanner.loadPath("18_Test_U_Shape_Dif_Angle", Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        // trajectories.add(PathPlanner.loadPath("19_Test_U_Shape_Same_Angle", Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));

        trajectory = PathPlanner.loadPath("2_TarmacB2_to_BBallB", 1.0, 0.5);


        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        m_robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(() -> {  
                    m_robotDrive.drive(
                            m_driverController.getRawAxis(4),
                            m_driverController.getRawAxis(5),
                            -m_driverController.getRawAxis(0) * 0.5, 
                            false);
                }, m_robotDrive));

            // new RunCommand(() -> {
            //     double input = MathUtil.applyDeadband(m_driverController.getRawAxis(5), 0.05) ;
            //     double speed = -input * 5 ;
            //     m_robotDrive.setWheelSpeeds( new MecanumDriveWheelSpeeds(speed, speed, speed, speed));
            // }, m_robotDrive)
            // );
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // Drive at half speed when the right bumper is held
        new JoystickButton(m_driverController, Button.kRightBumper.value)
                .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
                .whenReleased(() -> m_robotDrive.setMaxOutput(1));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * 
     * @param Filesystem
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // PathPlannerTrajectory trajectory = trajectories.get(0) ;

        PPMecanumControllerCommand ppCommand = new PPMecanumControllerCommand(
                trajectory,
                m_robotDrive::getPose,
                DriveConstants.kDriveKinematics,
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                new ProfiledPIDController( AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),
                AutoConstants.kMaxSpeedMetersPerSecond,
                m_robotDrive::setWheelSpeeds,
                m_robotDrive
                );


        // // Create config for trajectory
        // TrajectoryConfig config = new TrajectoryConfig(
        //         AutoConstants.kMaxSpeedMetersPerSecond,
        //         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //                 // Add kinematics to ensure max speed is actually obeyed
        //                 .setKinematics(DriveConstants.kDriveKinematics);

        // // An example trajectory to follow. All units in meters.
        // // if (Robot.trajectory == null) {
        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        //         // Start at the origin facing the +X direction
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         // List.of(new Translation2d(1, 1), new Translation2d(-2,1)),
        //         List.of(
        //                 new Translation2d(0, 1)),
        //         new Pose2d(0, 2, new Rotation2d(Math.PI / 2.0)),
        //         config);
        // // }


        // Reset odometry to the starting pose of the trajectory.
        Pose2d temp = trajectory.getInitialPose();
        PathPlannerState s = (PathPlannerState) trajectory.getStates().get(0) ;


      //  Pose2d temp2 = new Pose2d( temp.getTranslation(), new Rotation2d()) ;

      //Pose2d temp2 = new Pose2d( temp.getTranslation(), new Rotation2d(Math.PI/2.0)) ;
        Pose2d temp2 = new Pose2d( temp.getTranslation(), s.holonomicRotation) ;
      //m_robotDrive.resetOdometry(trajectory.getInitialPose());
        m_robotDrive.resetOdometry(temp2);

        // Run path following command, then stop at the end.
        return ppCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}