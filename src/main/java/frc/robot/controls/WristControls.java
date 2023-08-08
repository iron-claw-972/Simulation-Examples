package frc.robot.controls;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.WristSubsystemPID;
import frc.robot.subsystems.WristSubsystemSimple;

public class WristControls {

  /**
   * Below are instance variables. Think of them as the specific variables that we are going to need for THIS class
   * 
   * m_wrist is a variable of the type WristSubsystem 
   * m_button1, m_button1, and m_button3 are variables of the type Trigger
   * m_joy is a variable of the type Joystick
  */

  WristSubsystemPID m_wristPID; 
  WristSubsystemSimple m_wristSimple; 

  Trigger m_button1;
  Trigger m_button2;
  Trigger m_button3;
  Joystick m_joy; 

  /**
   * This is the constructor for the WristControls class. It requires a WristSubsystem class as a parameter. 
   * 
   * <p>
   * 
   * When we create an object of this class in RobotContainer.java we will pass it a WristSubsystem object to it as well. 
   * 
   * <p> 
   * 
   * By doing m_wrist = wristSubsystem, m_wrist holds a WristSubsystem class now. We can now access all the methods inside WristSubsystem.java
   * 
   * <p>
   * 
   * For the other instance variables, we can just turn them directly into objects by assigning them to an instance of the class
   * by doing m_instanceVariable = new Class(); in the constructor, ex: m_joy = new Joystick(0); 
   * 
   * <p> 
   * 
   * @param wristSubsystem
   */
  public WristControls(WristSubsystemPID wristSubsystemPID, WristSubsystemSimple wristSubsystemSimple){
    
    m_wristPID = wristSubsystemPID; 
    m_wristSimple = wristSubsystemSimple; 
    m_joy = new Joystick(0); 
    m_button1 = new JoystickButton(m_joy, 1);
    m_button2 = new JoystickButton(m_joy, 2);
    m_button3 = new JoystickButton(m_joy,3);
  }


  /**
   * The configureControls() method is used for control binding.
   */
  public void configureControls(){

    /**
     * When a button is pressed an InstantCommand() is created that, on button press,
     * sets the desired setpoint in the WristSubsystem using the setSetpoint() method declared there 
     */

    // m_button1.onTrue(new InstantCommand(()-> m_wrist.setSetpoint(-45)));
    // m_button2.onTrue(new InstantCommand(()-> m_wrist.setSetpoint(0)));
    // m_button3.onTrue(new InstantCommand(()-> m_wrist.setSetpoint(45)));  
    m_button1.onTrue(new InstantCommand(()-> m_wristSimple.setMotorPower(0.25)));
    m_button2.onTrue(new InstantCommand(()-> m_wristSimple.setMotorPower(-0.25)));
    

  }
}
