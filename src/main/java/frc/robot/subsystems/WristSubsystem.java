// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  //create the motor

  private final PWMSparkMax m_wristMotor;
  //private final TalonFXSimCollection m_wristMotorSim;  

  //create the wrist display canvas
  private Mechanism2d m_wristDisplay; 
  private MechanismRoot2d m_pivot; 
  private MechanismLigament2d m_stationaryAppendage; 
  private MechanismLigament2d m_movingAppendage; 
  private SingleJointedArmSim m_armSim; 

  private final PIDController m_controller;
  private double m_pidValue; 
  private double m_setpoint; 

  private final Encoder m_encoder;
  private final EncoderSim m_encoderSim; 

  public WristSubsystem() {
    m_wristMotor = new PWMSparkMax(0);
    //m_wristMotorSim = m_wristMotor.getSimCollection();
    //m_wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    m_encoder = new Encoder(0,1);
        
    m_encoderSim = new EncoderSim(m_encoder); 
    
    m_encoder.setDistancePerPulse(2*Math.PI/4096);

    m_controller = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);

    if(RobotBase.isSimulation()){
      
      m_armSim = new SingleJointedArmSim(
        WristConstants.m_armGearbox, 
        WristConstants.kGearing, 
        WristConstants.kMomentOfInertia, 
        WristConstants.kArmLength, 
        WristConstants.kMinAngleRads, 
        WristConstants.kMaxAngleRads,
        true
      );

      m_wristDisplay = new Mechanism2d(90, 90);

      m_pivot = m_wristDisplay.getRoot("ArmPivot", 45, 45);
  
      m_stationaryAppendage = m_pivot.append(new MechanismLigament2d("ArmTower", 60, -90));
  
      m_movingAppendage = m_pivot.append(
          new MechanismLigament2d(
              "Arm",
              30,
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));
  
      SmartDashboard.putData("Wrist", m_wristDisplay); 
    }

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  @Override
  public void simulationPeriodic() {
    m_armSim.setInput(m_wristMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    //This also updates the real encoder as used in the PID loop. 
    m_encoderSim.setDistance(m_armSim.getAngleRads());
    
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_movingAppendage.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));

    //uncomment below line if using the instant command bindings(second set in WristControls.java)
    //moveMotorsWithPID(getSetpoint()); 
  }

  public void moveMotorsWithPID(double setpoint){

    m_pidValue = m_controller.calculate(m_encoder.getDistance(), Units.degreesToRadians(setpoint));
    m_wristMotor.setVoltage(m_pidValue);
    
  }

  public void setSetpoint(double setpoint){
    m_setpoint = setpoint;
  }

  public double getSetpoint(){
    return m_setpoint;
  }

}
