// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.controls.WristControls;
import frc.robot.subsystems.WristSubsystemPID;
import frc.robot.subsystems.WristSubsystemMotorPower;


public class RobotContainer {

  /**
   * In this class we generally only declare subsystems, controls(like WristControls), and a few other things. 
   */

  //Create instance variables of the WristSubsystem and WristControls classes
  //see WristControls.java for an explanation of instance variables. 
  private final WristSubsystemPID m_wristPID; 
  private final WristSubsystemMotorPower m_wristSimple; 

  private final WristControls m_wristControls; 

  //the constructor. See WristControls.java for an explanation on constructors. 
  public RobotContainer() {
    //turn the m_wrist subsystem into an object by assigning it an instance of the WristSubsystem class 
    m_wristPID = new WristSubsystemPID(); 
    m_wristSimple = new WristSubsystemMotorPower(); 

    /**
     * Turn the m_wristControls instance variable into an object by passing it an instance of the WristControls class 
     *
     * Additionally, pass in m_wrist, the WristSubsystem object(created above) 
     * 
     * The WristControls() class requires a WristSubsystem as a parameter. 
     * 
     */
    m_wristControls = new WristControls(m_wristPID,m_wristSimple);
    
    //configure the button-command bindings
    m_wristControls.configureControls();
  }


}
