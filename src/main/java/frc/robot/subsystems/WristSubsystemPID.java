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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
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
import frc.robot.constants.WristConstantsPID;

public class WristSubsystemPID extends SubsystemBase {

  //These are instance variables. See WristControls.java for an explanation on this type of variable. 
  private final WPI_TalonFX m_wristMotor;
  private final PIDController m_controller;
  private double m_motorPower; 

  private TalonFXSimCollection m_motorSim; 
  private Mechanism2d m_wristDisplay; 
  private MechanismRoot2d m_pivot; 
  private MechanismLigament2d m_stationary; 
  private MechanismLigament2d m_moving; 
  private SingleJointedArmSim m_wristPhysicsSim; 



  /**
   * This is the constructor for the WristSubsystem class. 
   * See WristControls.java for a detailed explanation of a constructor. 
   */

  public WristSubsystemPID() {

    //create the motor for the wrist 
    m_wristMotor = new WPI_TalonFX(1);

    //create the PID controller 
    m_controller = new PIDController(WristConstantsPID.kP, WristConstantsPID.kI, WristConstantsPID.kD);

    
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

      //create wrist physics simulation. This will simulate the physics of our wrist. 
      //we are calculating the physiscs of our wrist based on a single jointed arm. 
      m_wristPhysicsSim = new SingleJointedArmSim(
        WristConstantsPID.m_armGearbox, 
        WristConstantsPID.kGearing, 
        WristConstantsPID.kMomentOfInertia, 
        WristConstantsPID.kArmLength, 
        WristConstantsPID.kMinAngleRadsHardStop, 
        WristConstantsPID.kMaxAngleRadsHardStop,
        false
      );
      
      m_wristPhysicsSim.setState(VecBuilder.fill(WristConstantsPID.kMinAngleRadsHardStop,0)); 
      //initialize encoder value to 0. This models us turning on our robot
      m_wristMotor.setSelectedSensorPosition(0);

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
              Units.radiansToDegrees(m_wristPhysicsSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

      //put the wrist display on the Sim GUI by putting m_wrist_display on smartDashboard. See slideshow on how to acess and use the simulator
      SmartDashboard.putData("Wrist with PID", m_wristDisplay); 
    }



    //put the PID on SmartDashboard for easy PID tuning. 
    SmartDashboard.putData(m_controller); 


    //inititally have the PID setpoint be set to the wrist's resting position so that the wrist doesn't fly up when powered on on a real robot. 
    setSetpoint(Units.radiansToDegrees(WristConstantsPID.kMinAngleRadsSoftStop));

    

  }


  //This the periodic method. It constantly runs in both teleop and autonomous(not sure about test mode though). 
  @Override
  public void periodic() {
    // This method will be called once every time the scheduler runs
    moveMotorsWithPID();
    
  }

  //This is the simulationPeriodic() method. It constantly runs in simuation. 
  @Override
  public void simulationPeriodic() {    
    moveMotorsWithPID(); 

    /**
     * First, we set our voltage to the physics sim
     *  
     * We do this by multipling the motor's power value(-1 to 1) by the calculated battery voltage
     * 
     * The BatterySim estimates loaded battery voltages(voltage of battery under load due to motors and other stuff running) based on the physics sim's calculation of it's current draw. 
     * 
     * Adding up all the currents of all of our subsystems would give us a more accurate loaded battery voltage number and a more accurate sim. But it's fine for this. It may or may not be neccesary to add based on the current draw of the other subsystems.  
     *    
     * If we were to call m_wristSim.getAngleRads() the physiscs sim would do some math using this voltage and gravity and other stuff(probably) and calculate its angle in radians. 
     *
     * Initally, in the constructor we give the wrist a setpoint to go to by calling setSetpoint(). Then, by calling moveMotorsWithPID(), m_wristMotor.get() returns a number that isn't 0. Thus, if we multiply it by the calculated battery voltage, we set a voltage to the physics sim. This changes the angle of the physics simulation. 
     *      
     * This voltage changes depending on the setpoint that we set to the wrist(see controls.java where we set these setpoints using instant commands). Thus, the simulated angle changes.
     * 
     * Finally, by setting the angle of the sim to the moving part of the wrist, we see it changes angle in the display. 
     */    
    m_wristPhysicsSim.setInput(m_wristMotor.get() * BatterySim.calculateDefaultBatteryLoadedVoltage(m_wristPhysicsSim.getCurrentDrawAmps()));
    
    //Next, we update the physics sim. The standard loop time is 20ms.
    m_wristPhysicsSim.update(0.020);

    //here we set the position of the encoder for PID calculations. We add the radian offset to the radian angle value that the physics sim returns in order to
    //convert from whatever angle the physics sim returns to what the actual angle the wrist should be in real life. 
    //For example, when the sim is at -PI/2 radians, the wrist looks like this:  ---|
    //                                                                                 |
    //The value that the encoder gets set is: -512 ticks. But at that angle, we want the encoder to be 0. Thus by adding the setpoint offset we get 0.
    //Read more in the comments above kSetpointOffsetRads in WristConstants.java.
    m_motorSim.setIntegratedSensorRawPosition(physicsSimRadsToTicks(m_wristPhysicsSim.getAngleRads()+WristConstantsPID.kSetpointOffsetRads)); 

    //Finally update the moving arm's angle based on the sim's angle. This updates the display. 
    m_moving.setAngle(Units.radiansToDegrees(m_wristPhysicsSim.getAngleRads()));
    
  }

  /**
   * This method sets the desired setpoint to the PID controller 
   * @param setpoint in degrees
   */
  public void setSetpoint(double setpoint){
    //reset i-term of PID controller. You should almost NEVER need to use I. 
    m_controller.reset();

    //add the radian setpoint offset to the setpoint after converting it to radians.
    //we add the offset here as well. Read the comments above kSetpointOffsetRads in WristConstants.java for an explanation why. 

    setpoint = Units.degreesToRadians(setpoint)+WristConstantsPID.kSetpointOffsetRads;
    
    //clamp the setpoint to prevent giving the wrist a setpoint value that exceeds it's max or min rotation range to prevent it from breaking itself
    //we add the offset here as well. Read the comments above kSetpointOffsetRads in WristConstants.java for an explanation why. 
    setpoint = MathUtil.clamp(setpoint, WristConstantsPID.kMinAngleRadsSoftStop+WristConstantsPID.kSetpointOffsetRads,WristConstantsPID.kMaxAngleRadsHardStop+WristConstantsPID.kSetpointOffsetRads);

    //finally set the setpoint to the controller
    m_controller.setSetpoint(setpoint); 
  }

  /**
   * This method employs a PID controller to calculate a value to send to a motor to get the arm to reach a setpoint(a radian angle)
   */
  public void moveMotorsWithPID(){

    m_motorPower = m_controller.calculate(m_wristMotor.getSelectedSensorPosition()*WristConstantsPID.kEncoderTicksToRadsConversion);

    //clamp the final motor power to prevent it from going to fast. This is useful in real life to stop your subsystem from breaking. 
    MathUtil.clamp(m_motorPower, WristConstantsPID.kMinPower, WristConstantsPID.kMaxPower); 

    m_wristMotor.set(m_motorPower);
    
  }


  /**
   * This function converts the physics simulation radian measurement to ticks(in integer form) for the simulated falcon encoder
   * The falcon encoder accepts only integers. 
   * 
   * @param rads the sim's radian measurement
   * @return raw encoder position in integer form 
   */
  public int physicsSimRadsToTicks(double rads){
    int rawPos = (int)((rads/(2*Math.PI))*2048);
    return rawPos;  
  }
}
