// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  //create the motor
  //why do we not just construct the whole motor here or in the constructor? like do
  //private final WPI_TalonFX m_wristMotor = new m_wristMotor(0); 
  private WPI_TalonFX m_wristMotor;
  private TalonFXSimCollection m_wristMotorSim;  

  //create the wrist display canvas
  private Mechanism2d m_wristDisplay; 
  private MechanismRoot2d m_pivot; 
  private MechanismLigament2d m_stationaryAppendage; 
  private MechanismLigament2d m_movingAppendage; 
  private SingleJointedArmSim m_armSim; 

  private PIDController m_controller;
  private double m_pidValue; 
  
  MechanismRoot2d m_root; 

  public WristSubsystem() {
    m_wristMotor = new WPI_TalonFX(WristConstants.motorPortSim);
    m_wristMotorSim = m_wristMotor.getSimCollection();
    m_wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);


    m_controller = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);
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
    m_pivot = m_wristDisplay.getRoot("pivot", 45, 45); 
    m_stationaryAppendage = m_pivot.append(new MechanismLigament2d("stationary appendage", 90, 180));
    m_movingAppendage = m_pivot.append(new MechanismLigament2d("Moving appendage", 70, m_armSim.getAngleRads(), 10, new Color8Bit(Color.kPurple))); 
    SmartDashboard.putData("Wrist", m_wristDisplay); 
    SmartDashboard.putData("PID Controller", m_controller); 
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_wristMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    m_armSim.setInput(m_wristMotorSim.getMotorOutputLeadVoltage()); 
    m_armSim.update(0.02);
    m_wristMotorSim.setIntegratedSensorRawPosition(rotationToEncoderCount(m_armSim.getAngleRads()));
    m_movingAppendage.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));

  }

  public void moveMotorsWithPID(double setpoint){
    m_pidValue =  m_controller.calculate(m_wristMotor.getSelectedSensorPosition(), setpoint);
    MathUtil.clamp(m_pidValue,-1,1); 
    m_wristMotor.set(m_pidValue); 
  }

  public int rotationToEncoderCount(double angleRads){
    double numRotations = Math.PI*4/angleRads; 
    int encoderCurrentCount = (int)numRotations; 
    return encoderCurrentCount;
  }


}
