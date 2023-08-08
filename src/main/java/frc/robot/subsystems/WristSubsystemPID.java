// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//This Simulation example is built off of WPILib's Arm Simulation Example and Iron Claw's 2023 FRC code available on Github. 
//https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armsimulation
//https://github.com/iron-claw-972/FRC2023


package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.constants.WristConstants;

public class WristSubsystemPID extends SubsystemBase {

  //These are instance variables. See WristControls.java for an explanation on this type of variable. 
  private final WPI_TalonFX m_wristMotor;

  private TalonFXSimCollection m_motorSim; 

  private Mechanism2d m_wristDisplay; 
  private MechanismRoot2d m_pivot; 
  private MechanismLigament2d m_stationary; 
  private MechanismLigament2d m_moving; 
  private SingleJointedArmSim m_armSim; 

  private final PIDController m_controller;
  private double m_motorPower; 

  /**
   * This is the constructor for the WristSubsystem class. 
   * See WristControls.java for a detailed explanation of a constructor. 
   */

  public WristSubsystemPID() {

    //create the motor for the wrist 
    m_wristMotor = new WPI_TalonFX(1);

    //create the PID controller 
    m_controller = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);

    //inititally have the PID setpoint be set to the wrist's resting position so that the wrist doesn't fly up when powered on on a real robot. 
    setSetpoint(Units.radiansToDegrees(WristConstants.kMinAngleRadsHardStop));

    
    /**
     * Allocate resources for simulation only if the robot is in a simulation. 
     * This is VERY IMPORTANT TO DO because when we update our simulated inbuilt motor encoder(falcon motors have an inbuilt encoder) in simulationPeriodic() it updates the actual encoder in the motor.
     * 
     * By wrapping the sim stuff in this if statement, simulation stuff is only created if we are running the simulation. If sim stuff is only created when sim mode is running, then there is little chance of actual motors, sensors, etc. being affected.
     *
     */
    if(RobotBase.isSimulation()){

      //create a simulated falcon(and inbuilt encoder, the encoder is inbuilt in the motor)
      m_motorSim = m_wristMotor.getSimCollection();

      //create arm physics simulation 
      m_armSim = new SingleJointedArmSim(
        WristConstants.m_armGearbox, 
        WristConstants.kGearing, 
        WristConstants.kMomentOfInertia, 
        WristConstants.kArmLength, 
        WristConstants.kMinAngleRadsHardStop, 
        WristConstants.kMaxAngleRadsHardStop,
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
       * Instead we tell the angle to be what the current angle of the arm is. When this changes in
       * simulationPeriodic() the moving arm will move on the display. 
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
    // This method will be called once every time the scheduler runs
    moveMotorsWithPID();
    
  }

  //This is the simulationPeriodic() method. It constantly runs in simuation. 
  @Override
  public void simulationPeriodic() {
    /**
     * First, we set our voltage to the armSim
     *  
     * We do this by multipling the motor's power value(-1 to 1) by the battery voltage, which gets estimated later.
     * 
     * If we were to call m_armSim.getAngleRads() the armSim would do some math using this voltage and calculate its angle in radians. 
     */    
    m_armSim.setInput(m_wristMotor.get() * RobotController.getBatteryVoltage());
    
    // Next, we update the armSim. The standard loop time is 20ms.
    m_armSim.update(0.020);

    /**
     * Finally, we set our simulated encoder's readings according to the angle(radians) of the armSim. 
     * IMPORTANT: This also updates the real encoder as used in the PID method.
     * 
     * The flow of the simulation is detailed below. 
     * 
     * 1) Set our simulated encoder's readings. 
     * 
     * 2) Actual encoder position, which is used in the PID method is updated automatically. 
     * 
     * 3) a new error is calculated by the PID method, and a new PID output is sent to the motor.
     *
     * 4) The motor's power value(-1 to 1) gets updated because of that 
     * 
     * 5) We once again set the armSim's input voltage as we did at the top of 
     * the simulationPeriodic() method, 
     * 
     * 6) the armSim's angle is affected, loop back to step 1.   
     */
    m_motorSim.setIntegratedSensorRawPosition(armSimRadsToTicks(m_armSim.getAngleRads())); 
      
    /**
     * The BatterySim estimates loaded battery voltages(voltage of battery under load due to motors and other stuff running) based on the armSim's calculation of it's current draw. 
     * 
     * Adding up all the currents of all of our subsystems would give us a more accurate loaded battery voltage number and a more accurate sim. But it's fine for this. It may or may not be neccesary to add based on the current draw of the other subsystems.  
     * 
     * We set this voltage as the VIn Voltage of the RobRio(voltage that gets sent to the roboRio, this input voltage changes in real life as well)
     * Then we can do RobotController.getBatteryVoltage() and multiply it by our motor's power to set the armSim's input, as we did at the top of the simulationPeriodic() method. 
     */

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Finally update the moving arm's angle based on the armSim angle. This updates the display. 
    m_moving.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    
    moveMotorsWithPID(); 
  }

  /**
   * This method employs a PID controller to calculate a value to send to a motor
   * to get the arm to reach a setpoint(an angle)
   * @param setpoint
   */
  public void moveMotorsWithPID(){

    m_motorPower = m_controller.calculate(m_wristMotor.getSelectedSensorPosition()*WristConstants.kEncoderTicksToRadsConversion);
    m_motorPower = m_motorPower + WristConstants.kGravityCompensation; 

    //clamp the final motor power to prevent it from going to fast. This is useful in real life to stop your subsystem from breaking. 
    MathUtil.clamp(m_motorPower, WristConstants.kMinPower, WristConstants.kMaxPower); 

    /**
     * Add a gravity compensation value to help the wrist fight back against gravity. The PID value gets so small near the actual setpoint 
     * that the motor is unable to overcome gravity
     */
    m_wristMotor.set(m_motorPower);
    
  }

  /**
   * This method sets the desired setpoint to the PID controller 
   * @param setpoint in degrees
   */
  public void setSetpoint(double setpoint){
    //reset i-term of PID controller. You should almost NEVER need to use I. 
    m_controller.reset();
    
    //clamp the setpoint to prevent giving the wrist a setpoint value that exceeds it's max or min rotation range to prevent it from breaking itself
    MathUtil.clamp(Units.degreesToRadians(setpoint), WristConstants.kMinAngleRadsSoftStop,WristConstants.kMaxAngleRadsSoftStop);
    
    //finally set the setpoint to the controller
    m_controller.setSetpoint(Units.degreesToRadians(setpoint)); 
  }

  /**
   * This function converts the armSim physics simulation radian measurement to ticks(in integer form) for the simulated falcon encoder
   * The falcon encoder accepts only integers. 
   * 
   * @param rads the armSim's radian measurement
   * @return raw encoder position in integer form 
   */
  public int armSimRadsToTicks(double rads){
    int rawPos = (int)(rads/(2*Math.PI)*2048);
    return rawPos;  
  }
}
