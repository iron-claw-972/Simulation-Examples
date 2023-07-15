// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class MoveWrist extends CommandBase {
  
  //These are instance variables. See WristControls.java for an explanation on this type of variable. 
  private final WristSubsystem m_wrist;
  private final double m_setpoint; 

  /**
   * this is the constructor for the MoveWrist command class. 
   * See WristControls.java for a detailed explanation of a constructor. 
   * 
   * @param Wrist the wrist subsystem
   * @param Setpoint the desired setpoint
   */
  public MoveWrist(WristSubsystem wrist, double setpoint) {
    m_wrist = wrist;
    m_setpoint = setpoint;

    /**
     * From WPILib docs: Each command should declare any subsystems it controls as requirements. 
     * This backs the command scheduler’s resource management mechanism, ensuring that no more than one command requires a given subsystem at the same time.
     * https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html
     */

    addRequirements(wrist);
  }

  @Override
  public void execute() {
    m_wrist.moveMotorsWithPID(m_setpoint);
  }

  /**
   * We don't need the below methods because we never want this command to end. We want the command to call
   * the moveMotorsWithPID() method constantly so that the motors get set a power from the PID controller constantly,
   * which updates the armSim constantly, which updates the moving arm display constantly. 
   */

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {}

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }
}
