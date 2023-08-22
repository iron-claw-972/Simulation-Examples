// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//This Simulation example is built off of WPILib's Arm Simulation Example and Iron Claw's 2023 FRC code available on Github. 

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.WristConstantsMotorPower;
import frc.robot.constants.WristConstantsMotorPower;

public class WristSubsystemMotorPower extends SubsystemBase {

  //These are instance variables. See WristControls.java for an explanation on this type of variable. 
  private final WPI_TalonFX m_wristMotor;

  private Mechanism2d m_wristDisplay; 
  private MechanismRoot2d m_pivot; 
  private MechanismLigament2d m_stationary; 
  private MechanismLigament2d m_moving; 
  private SingleJointedArmSim m_wristSim; 

  private double m_motorPower; 

  /**
   * This is the constructor for the WristSubsystemMotorPower class. 
   * See WristControls.java for a detailed explanation of a constructor. 
   */

  public WristSubsystemMotorPower() {

    //create the motor for the wrist 
    m_wristMotor = new WPI_TalonFX(1);

    /**
     * Allocate resources for simulation only if the robot is in a simulation. 
     * This is VERY IMPORTANT TO DO because when we update our simulated inbuilt motor encoder(falcon motors have an inbuilt encoder) in simulationPeriodic() it updates the actual encoder in the motor in the real world. 
     * 
     * By wrapping the simulation stuff in this if statement, simulation stuff is only created if we are running the simulation. If sim stuff is only created when sim mode is running, then there is little chance of actual motors, sensors, etc. being affected and (say) moving. 
     *
     */
    if(RobotBase.isSimulation()){

      //create wrist physics simulation. This will simulate the physics of our wrist. 
      //we are calculating the physiscs of our wrist based on a single jointed arm. 
      m_wristSim = new SingleJointedArmSim(
        WristConstantsMotorPower.m_armGearbox, 
        WristConstantsMotorPower.kGearing, 
        WristConstantsMotorPower.kMomentOfInertia, 
        WristConstantsMotorPower.kArmLength, 
        Units.degreesToRadians(WristConstantsMotorPower.kMinAngleDegrees), 
        Units.degreesToRadians(WristConstantsMotorPower.kMaxAngleDegrees),
        true
      );

      //create the "board" that our wrist will be displayed on 
      m_wristDisplay = new Mechanism2d(90, 90);

      //create the pivot that our wrist will bend from 
      m_pivot = m_wristDisplay.getRoot("ArmPivot", 45, 45);
  
      //create the stationary arm of the wrist
      m_stationary = m_pivot.append(new MechanismLigament2d("Stationary", 60, -180));
        
      /**
       * Create the moving arm of the wrist. 
       * 
       * Set the initial angle to whatever angle the simulation returns. Here the sim would return 0 radians. 
       * So initially we would see the moving arm be flat. The angle changes in simulationPeriodic() however, so the moving arm droops. 
       */
      m_moving = m_pivot.append(
          new MechanismLigament2d(
              "Moving",
              30,
              Units.radiansToDegrees(m_wristSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));
  
      //put the wrist on SmartDashboard by putting m_wristDisplay on SmartDashboard 
      SmartDashboard.putData("Wrist with motor power", m_wristDisplay); 
    }

    //initally set the motor power to 0. In real life this prevents the wrist from shooting up when the robot is powered on. 
    //it is a good idea to do these function calls after you have declared everything in the constructor to avoid getting errors(objects you are trying to use haven't been declared yet and thus return null)
    setMotorPower(0);
  }


  //This the periodic method. It constantly runs in teleop, autonomous, and test mode.   
  @Override
  public void periodic() {
    // This method will be called once every time the scheduler runs
    moveMotor();
  }


  //This is the simulationPeriodic() method. It constantly runs in disabled mode, teleop, and autonomous modes when we launch the simulator.

  @Override
  public void simulationPeriodic() {
    /**
     * Set the desired power as set by the setMotorPower() method(this is called in the constructor and in our controls.java file)
     */
    moveMotor();
    /**
     * First, we set our voltage to the physiscs sim. 
     *  
     * We do this by multipling the motor's power value(-1 to 1) by the calculated battery voltage
     * 
     * The BatterySim estimates loaded battery voltages(voltage of battery under load due to motors and other stuff running) based on the physics sim's calculation of it's current draw. 
     * 
     * Adding up all the currents of all of our subsystems would give us a more accurate loaded battery voltage number and a more accurate sim. But it's fine for this. It may or may not be neccesary to add based on the current draw of the other subsystems.  
     *    
     * If we were to call m_wristSim.getAngleRads() the physiscs sim would do some math using this voltage and gravity and other stuff(probably) and calculate its angle in radians. 
     *
     * Initally, in the constructor we call setMotorPower(0). Then we call moveMotor(), setting our desired power of 0. So, m_wristMotor.get() returns 0. Thus, we multiply our battery voltage by 0. We set a voltage of 0 to the physics simulation.
     * 
     * However the arm sim's angle still changes every loop iteration because it does calculations based on gravity. Thus doing m_armSim.getAngleRads() yields an angle.
     *
     * However when we call setMotorPower(0.5) in our controls.java file, which sets the speed to 50%, m_wristMotor.get() returns 0.5. Thus we set a different voltage to the physics sim, changing it's angle. 
     * 
     */    
    m_wristSim.setInput(m_wristMotor.get() * BatterySim.calculateDefaultBatteryLoadedVoltage(m_wristSim.getCurrentDrawAmps()));
    
  
    // Next, we update the armSim physics simulation. The standard loop time is 20ms.
    m_wristSim.update(0.020);
      

    // Finally update the moving arm's angle based on the armSim angle. This updates the display. 
    m_moving.setAngle(Units.radiansToDegrees(m_wristSim.getAngleRads()));
     

    //finally, loop back to the top of the loop. 
  }

  /**
   * This method sets a desired motor power to the m_motorPower variable declared in this file. 
   * We call this method in the constructor for this subsystem and in an InstantCommand in WristControls.java in order to set a desired motor power. 
   * @param setpoint
   */

  public void setMotorPower(double power){
    m_motorPower = MathUtil.clamp(power,-1,1); 
  }

  //set m_motorPower to the motor so that it moves.  
  public void moveMotor(){
    m_wristMotor.set(m_motorPower); 
  }

}
