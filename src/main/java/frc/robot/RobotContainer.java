// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.controls.WristControls;
import frc.robot.subsystems.WristSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  //Create nstance variables of the WristSubsystem and WristControls classes
  private final WristSubsystem m_wrist; 
  private final WristControls m_wristControls; 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //turn the m_wrist subsystem into an object by assigning it an instance of the WristSubsystem class 
    m_wrist = new WristSubsystem(); 

    /**
     * Turn the m_wristControls instance variable into an object by passing it an instance of the WristControls class 
     *
     * Additionally, pass in m_wrist, the WristSubsystem object(created above) 
     * 
     * The WristControls() class requires a WristSubsystem object as a parameter. 
     * 
     */
    m_wristControls = new WristControls(m_wrist);
    
    //configure the button-command bindings
    m_wristControls.configureControls();
  }


}
