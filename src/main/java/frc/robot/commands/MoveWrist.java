// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class MoveWrist extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final WristSubsystem m_wrist;
  private final double m_setpoint; 

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveWrist(WristSubsystem wrist, double setpoint) {
    m_wrist = wrist;
    m_setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.moveMotorsWithPID(m_setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
