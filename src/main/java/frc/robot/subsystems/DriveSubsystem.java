// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import edu.wpi.first.networktables.*;


public class DriveSubsystem extends SubsystemBase {

    WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(DriveConstants.ID_frontLeftMotor);
    WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(DriveConstants.ID_backLeftMotor);
    WPI_TalonSRX m_frontRight = new WPI_TalonSRX(DriveConstants.ID_frontRightMotor);
    WPI_TalonSRX m_rearRight = new WPI_TalonSRX(DriveConstants.ID_backRightMotor);

    private final MecanumDrive m_drive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);

    // The gyro sensor
    private final Gyro m_gyro = new ADXRS450_Gyro();

    // Odometry class for tracking robot pose
    MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d());

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        // Sets the distance per pulse for the encoders

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.

        // m_frontRight.setInverted(true);
        // m_rearRight.setInverted(true);
        // m_frontLeft.setInverted(false);
        // m_rearLeft.setInverted(false);

        m_frontRight.setInverted(false);
        m_rearRight.setInverted(false);
        m_frontLeft.setInverted(true);
        m_rearLeft.setInverted(true);


    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                m_gyro.getRotation2d(),
                new MecanumDriveWheelSpeeds(
                    getSpeed(m_frontLeft) ,
                    getSpeed(m_rearLeft) ,
                    getSpeed(m_frontRight) ,
                    getSpeed(m_rearRight) 
                    )
         ) ;
         report() ;
    }

    private double getSpeed(WPI_TalonSRX motor) {
        // multiplied by 10 because velocity reported as counts per 1/10th second
        double s = motor.getSelectedSensorVelocity() * 10.0 * Constants.DriveConstants.Meters_Per_Count;
        return (s);
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    
    /**
     * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
     * and the linear
     * speeds have no effect on the angular speed.
     *
     * @param xSpeed        Speed of the robot in the x direction
     *                      (forward/backwards).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        if (fieldRelative) {
            m_drive.driveCartesian(ySpeed, xSpeed, rot, -m_gyro.getAngle());
        } else {
            m_drive.driveCartesian(ySpeed, xSpeed, rot);
        }
    }

    /** Sets the front left drive MotorController to a voltage. */
    public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
        m_frontLeft.setVoltage(volts.frontLeftVoltage);
        m_rearLeft.setVoltage(volts.rearLeftVoltage);
        m_frontRight.setVoltage(volts.frontRightVoltage);
        m_rearRight.setVoltage(volts.rearRightVoltage);
        NetworkTableInstance.getDefault().getEntry("drive/front_left_volts").setDouble(volts.frontLeftVoltage);
        NetworkTableInstance.getDefault().getEntry("drive/front_right_volts").setDouble(volts.frontRightVoltage);
        NetworkTableInstance.getDefault().getEntry("drive/rear_left_volts").setDouble(volts.rearLeftVoltage);
        NetworkTableInstance.getDefault().getEntry("drive/rear_right_volts").setDouble(volts.rearRightVoltage);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.setSelectedSensorPosition(0);
        m_rearLeft.setSelectedSensorPosition(0);
        m_frontRight.setSelectedSensorPosition(0);
        m_rearRight.setSelectedSensorPosition(0);
    }


    /**
     * Gets the current wheel speeds.
     *
     * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
     */
    public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
            getSpeed(m_frontLeft) ,
            getSpeed(m_rearLeft) ,
            getSpeed(m_frontRight) ,
            getSpeed(m_rearRight) ) ;

                // m_frontLeftEncoder.getRate(),
                // m_rearLeftEncoder.getRate(),
                // m_frontRightEncoder.getRate(),
                // m_rearRightEncoder.getRate());
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -m_gyro.getRate();
    }


    private void report() {
        // Report various parameters out to network tables for monitoring purposes
        NetworkTableInstance.getDefault().getEntry("drive/back_left_position").setDouble(m_rearLeft.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("drive/back_left_velocity").setDouble(m_rearLeft.getSelectedSensorVelocity());

        NetworkTableInstance.getDefault().getEntry("drive/back_right_position").setDouble(m_rearRight.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("drive/back_right_velocity").setDouble(m_rearRight.getSelectedSensorVelocity());

        NetworkTableInstance.getDefault().getEntry("drive/front_left_position").setDouble(m_frontLeft.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("drive/front_left_velocity").setDouble(m_frontLeft.getSelectedSensorVelocity());

        NetworkTableInstance.getDefault().getEntry("drive/front_right_position").setDouble(m_frontRight.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("drive/front_right_velocity").setDouble(m_frontRight.getSelectedSensorVelocity());
    }


}
