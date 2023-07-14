// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//This Simulation example is built off of WPILib's Arm Simulation Example available on Github. 
//https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armsimulation

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
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

  private final WPI_TalonFX m_wristMotor;
  //private final TalonFXSimCollection m_wristMotorSim;  

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

  public WristSubsystem() {

    //create the motor for the wrist 
    m_wristMotor = new WPI_TalonFX(0);
    //m_wristMotorSim = m_wristMotor.getSimCollection();
    //m_wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    //create the encoder
    m_encoder = new Encoder(0,1);

    //set the distance per pulse -- how much of an angle(in radians) each tick of the encoder is 
    m_encoder.setDistancePerPulse(2*Math.PI/4096);

    //create the PID controller 
    m_controller = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);

    /**
     * Allocate resources for simulation only if the robot is in a simulation. 
     * This is VERY IMPORTANT TO DO because when we update our simulated encoder in simulationPeriodic() it updates the actual encoder
     * So if we don't create the simulation stuff simulationPeriodic has nothing to update
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

      //create the "board" that our arm will be displayed on 
      m_wristDisplay = new Mechanism2d(90, 90);

      //create the pivot that our wrist will bend from 
      m_pivot = m_wristDisplay.getRoot("ArmPivot", 45, 45);
  
      //create the stationary arm of the wrist
      m_stationary = m_pivot.append(new MechanismLigament2d("Stationary", 60, -180));
  
      //create the moving part of the wrist. We pass to the angle parameter m_armSim.getAngleRads() so that it
      
      /**
       * Create the moving arm of the wrist. 
       * In the angle parameter we don't set a fixed angle. Otherwise this moving part won't move. 
       * Instead we tell the angle to be what the current angle of the arm simulation is. When this changes in
       * simulationPeriodic() the moving part of the wrist will thus move. 
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
      SmartDashboard.putData(m_controller); 
    }

  }


  //This the periodic method. It constantly runs in teleop. 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  //this is the simulationPeriodic() method. It constantly runs in simuation. 
  @Override
  public void simulationPeriodic() {
    /**
     * First, we set our voltage to the arm.
     *  
     * We do this by multipling the motor's power value(-1 to 1) by the battery voltage. The voltage
     * is estimated in the lines below where we set our simulated encoder's readings. 
     * 
     * The Arm sim then does some math to calculate it's position in radians, which we can get using
     * the m_armSim.getAngleRads() method. 
     */

    m_armSim.setInput(m_wristMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    /**
     * Finally, we set our simulated encoder's readings according to the angle(radians) of the simulated arm. 
     * IMPORTANT: This also updates the real encoder as used in the PID method.
     * 
     * Because of this, a new error is calculated by the PID method, and a new PID output is sent to the motor.
     * This affects the motor's power value(-1 to 1) and we again set the arm's input voltage(see line 141). 
     *  
     */

    m_encoderSim.setDistance(m_armSim.getAngleRads());
    
    /**
     * The BatterySim estimates loaded battery voltages(voltage of battery under motor load) based on the arm sim.
     * We set this voltage as the VIn Voltage of the RobRio(voltage into the roboRio, this input voltage changes in real life as well)
     * Then we can do RobotController.getBatteryVoltage() as in line 141 to get this VIn Voltage
     */

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Update the moving arm angle based on the simulated arm angle
    m_moving.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));

    //uncomment below line if using the instant command bindings(second set in WristControls.java)
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
