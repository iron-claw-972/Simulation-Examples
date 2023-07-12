package frc.robot.controls;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.MoveWrist;
import frc.robot.subsystems.WristSubsystem;

public class WristControls {

  /**
   * These are instance variables.
   * Think of them as the specific variables that we are going to need for THIS class
   * 
   * m_wrist is a variable of the type WristSubsystem 
   * m_button1, m_button1, and m_button3 are variables of the type Trigger
   * m_joy is a variable of the type Joystick
  */

  WristSubsystem m_wrist; 
  Trigger m_button1;
  Trigger m_button2;
  Trigger m_button3;
  Joystick m_joy; 

  /**
   * This is the constructor for the WristControls class. As a parameter, it requires a WristSubsystem. 
   * 
   * <p>
   * 
   * When an object of the WristControls() class is created(see RobotContainer.java) we will also pass in a WristSubsystem object to it(see RobotContainer.java)
   * and assign it to the m_wrist instance variable. m_wrist now holds a WristSubsystem object. Now we can access the
   * various methods we made in the WristSubsystem.java file.
   * 
   * <p>
   * 
   * For the other instance variables, we can just turn them directly into objects by assigning them to an instance of the class
   * by doing m_instanceVariable = new Class();
   * 
   * @param wristSubsystem
   */
  public WristControls(WristSubsystem wristSubsystem){
    
    m_wrist = wristSubsystem; 
    m_joy = new Joystick(0); 
    m_button1 = new JoystickButton(m_joy, 1);
    m_button2 = new JoystickButton(m_joy, 2);
    m_button3 = new JoystickButton(m_joy,3);
  }


  /**
   * The configureControls() method binds the MoveWrist() command to the pressing of different buttons
   */
  public void configureControls(){

    /**
     * When a button is pressed run the MoveWrist command. 
     * Pass the command the WristSubsystem(stored in m_wrist instance variable) and a desired setpoint to move the wrist too
     */
    m_button1.onTrue(new MoveWrist(m_wrist, -50)); 
    m_button2.onTrue(new MoveWrist(m_wrist, 75)); 
    m_button3.onTrue(new MoveWrist(m_wrist, 200));
    
    /**
     * When a button is pressed, create an InstantCommand() that, on button press, set's the desired setpoint in the WristSubsystem     * 
     */

    // m_button1.onTrue(new InstantCommand(()-> m_wrist.setSetpoint(-50)));
    // m_button2.onTrue(new InstantCommand(()-> m_wrist.setSetpoint(75)));
    // m_button3.onTrue(new InstantCommand(()-> m_wrist.setSetpoint(200)));

    

  }
}
