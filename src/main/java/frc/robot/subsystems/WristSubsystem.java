// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//This Simulation example is built off of WPILib's Arm Simulation Example and Iron Claw's 2023 FRC code available on Github. 
//https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armsimulation
//https://github.com/iron-claw-972/FRC2023


package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.WristConstants;

public class WristSubsystem extends SubsystemBase {

  //These are instance variables. See WristControls.java for an explanation on this type of variable. 
  private final WPI_TalonFX m_wristMotor;
  private Mechanism2d m_wristDisplay; 
  private MechanismRoot2d m_pivot; 
  private MechanismLigament2d m_stationary; 
  private MechanismLigament2d m_moving; 
  private SingleJointedArmSim m_armSim; 

  private final PIDController m_controller;
  private double m_pidValue; 
  private double m_setpoint; 

  private final Encoder m_encoder;
  private EncoderSim m_encoderSim; 

  /**
   * This is the constructor for the WristSubsystem class. 
   * See WristControls.java for a detailed explanation of a constructor. 
   */

  public WristSubsystem() {

    //create the motor for the wrist 
    m_wristMotor = new WPI_TalonFX(1);

    //create the encoder
    m_encoder = new Encoder(0,1);

    //set the distance per pulse -- how much of an angle(in radians) each tick of the encoder is 
    m_encoder.setDistancePerPulse(2*Math.PI/4096);

    //create the PID controller 
    m_controller = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);

    /**
     * Allocate resources for simulation only if the robot is in a simulation. 
     * This is VERY IMPORTANT TO DO because when we update our simulated encoder in simulationPeriodic() it updates the actual encoder.
     * But since this only creates simulation stuff if we are running the simulation, simulationPeriodic() has nothing to update when we are running it on a real robot. 
     * 
     * You don't want your encoder values changing in teleop because your simulation is running. That would break everything 
     */
    if(RobotBase.isSimulation()){
      //create simulated encoder 
      m_encoderSim = new EncoderSim(m_encoder); 
      
      //create arm physics simulation 
      m_armSim = new SingleJointedArmSim(
        WristConstants.m_armGearbox, 
        WristConstants.kGearing, 
        WristConstants.kMomentOfInertia, 
        WristConstants.kArmLength, 
        WristConstants.kMinAngleRads, 
        WristConstants.kMaxAngleRads,
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
       * In the angle parameter we don't set a fixed angle. Otherwise this moving part won't move. 
       * Instead we tell the angle to be what the current angle of the arm simulation is. When this changes in
       * simulationPeriodic() the moving part of the wrist will thus move on the display. 
       */
      m_moving = m_pivot.append(
          new MechanismLigament2d(
              "Moving",
              30,
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));
  
      //put the wrist on SmartDashboard by putting the wrist_display on smartDashboard 
      SmartDashboard.putData("Wrist", m_wristDisplay); 

      //put the PID on SmartDashboard for easy PID tuning. 
      SmartDashboard.putData(m_controller); 
    }

  }


  //This the periodic method. It constantly runs in both teleop and autonomous. 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  //This is the simulationPeriodic() method. It constantly runs in simuation. 
  @Override
  public void simulationPeriodic() {
    /**
     * First, we set our voltage to the armSim
     *  
     * We do this by multipling the motor's power value(-1 to 1) by the battery voltage. This voltage
     * is estimated later. 
     * 
     * The armSim then does some math to calculate it's position in radians, which we can get using
     * the getAngleRads() method. 
     */

    m_armSim.setInput(m_wristMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update the armSim. The standard loop time is 20ms.
    m_armSim.update(0.020);

    /**
     * Finally, we set our simulated encoder's readings according to the angle(radians) of the armSim. 
     * IMPORTANT: This also updates the real encoder as used in the PID method.
     * 
     * Because of this, a new error is calculated by the PID method, and a new PID output is sent to the motor.
     * This affects the motor's power value(-1 to 1) and we again set the armSim's input voltage as we did at the top of 
     * the simulationPeriodic() method, which affects the armSim's angle. 
     */

    m_encoderSim.setDistance(m_armSim.getAngleRads());
    
    /**
     * The BatterySim estimates loaded battery voltages(voltage of battery under motor load) based on the armSim.
     * We set this voltage as the VIn Voltage of the RobRio(voltage into the roboRio, this input voltage changes in real life as well)
     * Then we can do RobotController.getBatteryVoltage() and multiply it by our motor's power to set the armSim's input, as we did at the top of the simulationPeriodic() method. 
     */

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Finally update the moving arm's angle based on the armSim angle. This updates the display. 
    m_moving.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));

    //uncomment below line if using the instant command bindings(second set in WristControls.java)
    //reason for this given in slideshow, slide 8: https://docs.google.com/presentation/d/175tFEuVqD1jP3H9Jsz3JcBUEr5Q0GBBXTg9RuWCQ6A4/edit#slide=id.g10b0dfc564d_0_5
    
    //moveMotorsWithPID(getSetpoint()); 
  }

  /**
   * This method employs a PID controller to calculate a value to send to a motor
   * to get the arm to reach a setpoint(an angle)
   * @param setpoint
   */
  public void moveMotorsWithPID(double setpoint){

    m_pidValue = m_controller.calculate(m_encoder.getDistance(), Units.degreesToRadians(setpoint));
    m_pidValue = MathUtil.clamp(m_pidValue,-1,1); 
    m_wristMotor.set(m_pidValue);
    
  }

  /**
   * This method sets the m_setpoint variable declared in this subsystem to a setpoint set by 
   * an instant command in WristControls.java
   * @param setpoint
   */
  public void setSetpoint(double setpoint){
    m_setpoint = setpoint;
  }

  /**
   * This method simply returns m_setpoint
   * @return m_setpoint
   */
  public double getSetpoint(){
    return m_setpoint;
  }

}
