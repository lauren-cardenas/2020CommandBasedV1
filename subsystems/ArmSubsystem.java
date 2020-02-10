/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class ArmSubsystem extends SubsystemBase {
  
  private final WPI_VictorSPX m_armMotor = new WPI_VictorSPX(ArmConstants.kMotorPort);

  private final WPI_VictorSPX m_rollerMotor = new WPI_VictorSPX(ArmConstants.kRollerPort);

  private final DigitalInput d_down = new DigitalInput(ArmConstants.kDown);
  private final DigitalInput d_up = new DigitalInput(ArmConstants.kUp);

  /**
   * Creates a new ArmSubsystem.
   */
  public ArmSubsystem() {

  }

  public void moveDown(){
    if(!d_down.get()){
      m_armMotor.set(OIConstants.ARM_SPEED);
    } else{
      m_armMotor.set(0);
    }
  }

  public void moveUp(){
    if(!d_up.get()){
      m_armMotor.set(OIConstants.ARM_SPEED);
    } else{
      m_armMotor.set(0);
    }
  }

  public void intake(double inSpeed){
    m_rollerMotor.set(inSpeed);
  }

  public void outtake(double outSpeed){
    m_rollerMotor.set(outSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
