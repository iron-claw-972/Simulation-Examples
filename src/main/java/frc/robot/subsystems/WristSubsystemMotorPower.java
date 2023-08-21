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
  private SingleJointedArmSim m_armSim; 

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
     * This is VERY IMPORTANT TO DO because when we update our simulated inbuilt motor encoder(falcon motors have an inbuilt encoder) in simulationPeriodic() it updates the actual encoder in the motor.
     * 
     * By wrapping the simulation stuff in this if statement, simulation stuff is only created if we are running the simulation. If sim stuff is only created when sim mode is running, then there is little chance of actual motors, sensors, etc. being affected and (say) moving. 
     *
     */
    if(RobotBase.isSimulation()){

      //create arm physics simulation 
      m_armSim = new SingleJointedArmSim(
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
       * In the angle parameter we don't set a fixed angle. Otherwise the moving arm won't update it's position when we display it. 
       * 
       * Instead we tell the angle to be what the current angle of the arm physics simulation is. When this changes in
       * simulationPeriodic() the moving arm will move on the display. 
       */
      m_moving = m_pivot.append(
          new MechanismLigament2d(
              "Moving",
              30,
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));
  
      //put the wrist on SmartDashboard by putting m_wristDisplay on SmartDashboard 
      SmartDashboard.putData("Wrist with motor power", m_wristDisplay); 
    }

    //initally set the motor power to 0. In real life this prevents the wrist from shooting up when the robot is powered on. 
    //it is a good idea to do these function calls after you have declared everything in the constructor to avoid getting errors(objects you are trying to use haven't been declared yet and thus return null)
    setMotorPower(0);
  }


  //This the periodic method. It constantly runs in teleop and autonomous(not sure about test mode).   
  @Override
  public void periodic() {
    // This method will be called once every time the scheduler runs
    moveMotor();
  }


  //This is the simulationPeriodic() method. It constantly runs in disabled mode, teleop, and autonomous modes when we launch the simulator.

  @Override
  public void simulationPeriodic() {
    /**
     * First, we set our voltage to the armSim physiscs simulation. 
     *  
     * We do this by multipling the motor's power value(-1 to 1) by the battery voltage, which gets estimated later.
     * 
     * The motor's power value changes when our instant commands in WristControls changes it(they call the setMotorPower() method). 
     * This causes us to set a different voltage input to the arm sim, which updates it's angle,which updates the moving arm of the wrist, 
     * which updates the display. 
     * 
     */   
    
    m_armSim.setInput(m_wristMotor.get() * RobotController.getBatteryVoltage());
  
    // Next, we update the armSim physics simulation. The standard loop time is 20ms.
    m_armSim.update(0.020);
      
    /**
     * The BatterySim estimates loaded battery voltages(voltage of battery under load due to motors and other stuff running) based on the armSim's calculation of it's current draw. 
     * 
     * Adding up all the currents of all of our subsystems would give us a more accurate loaded battery voltage number and a more accurate sim. But it's fine for this. It may or may not be neccesary to add currents based on the current draw of the other subsystems.  
     * 
     * We set this voltage as the VIn Voltage of the RoboRio(voltage that gets sent to the roboRio, this input voltage changes in real life as well)
     * Then we can do RobotController.getBatteryVoltage() to get the VIn voltage of the roboRio and multiply it by our motor's power to set the armSim's input, as we did at the top of the simulationPeriodic() method. 
     */

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Finally update the moving arm's angle based on the armSim angle. This updates the display. 
    m_moving.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    
    //call the moveMotor method to set the desired power to the motor. 
    moveMotor(); 
  }

  /**
   * This method sets a desired motor power to the m_motorPower variable declared in this file. 
   * We call this method in an InstantCommand in WristControls.java in order to set a desired motor power. 
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
