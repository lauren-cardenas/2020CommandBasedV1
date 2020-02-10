/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ArmSubsystem m_robotArm = new ArmSubsystem();

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
/*
    double triggerVal = (m_driverController.getTriggerAxis(Hand.kRight)
    - m_driverController.getTriggerAxis(Hand.kLeft));

    double stick = (m_driverController.getX(Hand.kLeft)) * OIConstants.TURNING_RATE;

    double left_command = (triggerVal + stick) * OIConstants.DRIVING_SPEED;
    double right_command = (triggerVal - stick) * OIConstants.DRIVING_SPEED;
*/
    m_robotDrive.setDefaultCommand(
      new RunCommand(() -> m_robotDrive
        .tankDrive(
          (m_driverController.getTriggerAxis(Hand.kRight)
          - m_driverController.getTriggerAxis(Hand.kLeft)
          + (m_driverController.getX(Hand.kLeft) * OIConstants.TURNING_RATE))
          * OIConstants.DRIVING_SPEED,
          (m_driverController.getTriggerAxis(Hand.kRight)
          - m_driverController.getTriggerAxis(Hand.kLeft)
          - (m_driverController.getX(Hand.kLeft) * OIConstants.TURNING_RATE))
          * OIConstants.DRIVING_SPEED
        ), m_robotDrive));
  }

  
  private void configureButtonBindings() {
  
  //Driver Buttons
    new JoystickButton(m_driverController, Button.kY.value)
      .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
      .whenReleased(() -> m_robotDrive.setMaxOutput(1));
    
    new JoystickButton(m_driverController, Button.kX.value)
      .whileHeld(() -> m_robotDrive.tankDrive(1, 1), m_robotDrive);

    new JoystickButton(m_driverController, Button.kA.value)
      .whileHeld(() -> m_robotDrive.limelightLeft(), m_robotDrive);

    new JoystickButton(m_driverController, Button.kB.value)
      .whileHeld(() -> m_robotDrive.limelightRight(), m_robotDrive);

  //Operator Buttons
    new JoystickButton(m_operatorController, Button.kA.value)
      .whileHeld(() -> m_robotArm.moveDown(), m_robotArm);

    new JoystickButton(m_operatorController, Button.kY.value)
      .whileHeld(() -> m_robotArm.moveUp(), m_robotArm);

    new JoystickButton(m_operatorController, Button.kB.value)
      .whileHeld(() -> m_robotArm.intake(OIConstants.ROLLER_SPEED), m_robotArm)
      .whenReleased(() -> m_robotArm.intake(0));
    
    new JoystickButton(m_operatorController, Button.kX.value)
      .whileHeld(() -> m_robotArm.outtake(-OIConstants.ROLLER_SPEED), m_robotArm)
      .whenReleased(() -> m_robotArm.outtake(0));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }
}
