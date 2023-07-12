package frc.robot.controls;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.MoveWrist;
import frc.robot.subsystems.WristSubsystem;

public class WristControls {
  WristSubsystem m_wrist; 
  Trigger m_button1;
  Joystick m_joy; 

  public WristControls(WristSubsystem wrist){
    m_wrist = wrist; 
    m_joy = new Joystick(0); 
    m_button1 = new JoystickButton(m_joy, 1);

  }

  public void configureControls(){

    //my hyptothesis as to why the instant command wasn't working: 
    //it will initialize, execute once, and end on the same iteration of the scheduler. (WPILIB docs)
    //
    m_button1.onTrue(new MoveWrist(m_wrist, 50)); 
    //m_button1.onFalse(new InstantCommand(()-> m_wrist.moveMotorsWithPID(70))); 
    //m_button1.onTrue(new InstantCommand(()->System.out.println("Not Pressed"))); 
    

  }
}
