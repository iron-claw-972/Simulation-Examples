package frc.robot.controls;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.WristConstantsPID;
import frc.robot.subsystems.WristSubsystemMotorPower;
import frc.robot.subsystems.WristSubsystemPID;

public class WristControls {

  /**
   * Below are instance variables. Think of them as the specific variables that we are going to need for THIS class
   * 
   * m_wrist is a variable of the type WristSubsystemPID
   * m_button1, m_button1, and m_button3 are variables of the type Trigger
   * m_joy is a variable of the type Joystick
  */

  WristSubsystemPID m_wristPID; 
  WristSubsystemMotorPower m_wristMotorPower; 

  Trigger m_button1PID;
  Trigger m_button2PID;
  Trigger m_button3PID;

  Trigger m_button1MotorPower;
  Trigger m_button2MotorPower;
  
  Joystick m_joyPID; 
  Joystick m_joyMotorPower; 

  /**
   * This is the constructor for the WristControls class. We give the constructor a parameter -- an object(wristSubsystemPID) of the type WristSubsystemPID(a class). 
   * 
   * <p>
   * 
   * To satisfy this parameter, when we create an object of this controls class in RobotContainer.java we will pass in a WristSubsystemPID object into it. 
   * 
   * <p> 
   * 
   * By doing m_wrist = wristSubsystemPID(line 50), m_wrist now holds a WristSubsystemPID class now. We can now access all the methods inside WristSubsystemPID.java
   * 
   * <p>
   * 
   *  For the other instance variables, we can just pass in objects of our desired classes to them by doing m_instanceVariable = new Class(); .We do this in the constructor below, ex: m_joy = new Joystick(0); 
   *
   * <p> 
   * 
   * @param wristSubsystem
   */
  public WristControls(WristSubsystemPID wristSubsystemPID, WristSubsystemMotorPower wristSubsystemMotorPower){
    
    m_wristPID = wristSubsystemPID; 
    m_wristMotorPower = wristSubsystemMotorPower; 

    //create a joystick object and assign it to m_joy. this joystick and its buttons can be accessed in simulation and real life. 
    m_joyPID = new Joystick(0); 
    m_joyMotorPower = new Joystick(1); 

    //PID buttons
    m_button1PID = new JoystickButton(m_joyPID, 1);
    m_button2PID = new JoystickButton(m_joyPID, 2);
    m_button3PID = new JoystickButton(m_joyPID,3);

    //motor power buttons
    m_button1MotorPower = new JoystickButton(m_joyMotorPower, 1);
    m_button2MotorPower = new JoystickButton(m_joyMotorPower, 2);



  }


  /**
   * The configureControls() method is used for control binding.
   */
  public void configureControls(){

    /**
     * When a button is pressed an InstantCommand() is created that, on button press,
     * sets the desired setpoint to the wrist by using the setSetpoint() method declared in WristSubsystemPID.java
     */

    m_button1PID.onTrue(new InstantCommand(()-> m_wristPID.setSetpoint(-45)));
    m_button2PID.onTrue(new InstantCommand(()-> m_wristPID.setSetpoint(0)));
    m_button3PID.onTrue(new InstantCommand(()-> m_wristPID.setSetpoint(45)));  

    //Motor Power buttons
    m_button1MotorPower.onTrue(new InstantCommand(()-> m_wristMotorPower.setMotorPower(0.25)));
    m_button2MotorPower.onTrue(new InstantCommand(()-> m_wristMotorPower.setMotorPower(-0.25)));



  }
}
