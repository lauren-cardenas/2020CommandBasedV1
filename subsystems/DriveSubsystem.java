/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimeLightConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class DriveSubsystem extends SubsystemBase {
  //Motors
  private final SpeedControllerGroup m_leftMotors =
    new SpeedControllerGroup(new WPI_VictorSPX(DriveConstants.kLeftMotor1Port), new WPI_VictorSPX(DriveConstants.kLeftMotor2Port));
  private final SpeedControllerGroup m_rightMotors =
    new SpeedControllerGroup(new WPI_VictorSPX(DriveConstants.kRightMotor1Port), new WPI_VictorSPX(DriveConstants.kRightMotor2Port));
  
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
/*
  private final Encoder m_leftEncoder =
    new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1], DriveConstants.kLeftEncoderReversed);
  private final Encoder m_rightEncoder =
    new Encoder(DriveConstants.kRightEncoderPorts[2], DriveConstants.kRightEncoderPorts[3], DriveConstants.kRightEncoderReversed);


  //Creates DriveSubsystem
  public DriveSubsystem() {
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
  }
*/
  public void tankDrive(double lft, double rht){
    m_drive.tankDrive(lft, rht);
  }
/*
  public void resetEncoders(){
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getAverageEncoderDistance(){
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  public Encoder getLeftEncoder(){
    return m_leftEncoder;
  }

  public Encoder getRightEncoder(){
    return m_rightEncoder;
  }
*/
  public void setMaxOutput(double maxOutput){
    m_drive.setMaxOutput(maxOutput);
  }

  public void limelightLeft(){
    // tune for smooth robot movement junk
    final double kP = LimeLightConstants.kP;
    final double min_command = LimeLightConstants.kMinCommand;
    double steering_adjust = 0.0;
    final double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    final double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    double left_command = 0.0;
    double right_command = 0.0;

    final double heading_error = -tx;
      if(tv == 0.0)
      {
        steering_adjust = -0.5;
      }
      else 
      {
        if(tx<1.0){
          steering_adjust = kP * heading_error + min_command;
        }
        else if(tx>1.0){
          steering_adjust = kP * heading_error - min_command;
        }
      }
      left_command+=steering_adjust;
      right_command-=steering_adjust;
      m_drive.tankDrive(left_command, right_command);
  }

  public void limelightRight(){
    // tune for smooth robot movement junk
    final double kP = LimeLightConstants.kP;
    final double min_command = LimeLightConstants.kMinCommand;
    double steering_adjust = 0.0;
    final double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    final double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    double left_command = 0.0;
    double right_command = 0.0;

    final double heading_error = -tx;
      if(tv == 0.0)
      {
        steering_adjust = 0.5;
      }
      else 
      {
        if(tx<1.0){
          steering_adjust = kP * heading_error + min_command;
        }
        else if(tx>1.0){
          steering_adjust = kP * heading_error - min_command;
        }
      }
      left_command+=steering_adjust;
      right_command-=steering_adjust;
      m_drive.tankDrive(left_command, right_command);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
