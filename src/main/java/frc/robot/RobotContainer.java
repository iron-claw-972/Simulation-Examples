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
  // The robot's subsystems and commands are defined here...
  private final WristSubsystem m_wrist;
  private final WristControls m_wristControls; 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_wrist = new WristSubsystem(); 
    m_wristControls = new WristControls(m_wrist);
    m_wristControls.configureControls();
  }


}
