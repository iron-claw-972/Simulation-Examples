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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.WristConstants;

public class WristSubsystem extends SubsystemBase {

  //create the motor

  private final WPI_TalonFX m_wristMotor;
  //private final TalonFXSimCollection m_wristMotorSim;  

  //create the wrist display canvas
  private final Mechanism2d m_wristDisplay; 
  private final MechanismRoot2d m_pivot; 
  private final MechanismLigament2d m_stationaryAppendage; 
  private final MechanismLigament2d m_movingAppendage; 
  private final SingleJointedArmSim m_armSim; 

  private final PIDController m_controller;
  private double m_pidValue; 

  private final Encoder m_encoder;
  private final EncoderSim m_encoderSim; 

  public WristSubsystem() {
    m_wristMotor = new WPI_TalonFX(WristConstants.motorPortSim);
    //m_wristMotorSim = m_wristMotor.getSimCollection();
    //m_wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    m_encoder = new Encoder(0,1); 
    m_encoderSim = new EncoderSim(m_encoder); 

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
    m_movingAppendage = m_pivot.append(new MechanismLigament2d("Moving appendage", 70, m_armSim.getAngleRads(), 10, new Color8Bit(Color.kPurple))); 
    m_stationaryAppendage = m_pivot.append(new MechanismLigament2d("stationary appendage", 90, 180));
    SmartDashboard.putData("Wrist", m_wristDisplay); 
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // m_wristMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    // m_armSim.setInput(m_wristMotorSim.getMotorOutputLeadVoltage()); 
    // m_armSim.update(0.02);
    // m_wristMotorSim.setIntegratedSensorRawPosition(rotationToEncoderCount(m_armSim.getAngleRads()));
    // m_movingAppendage.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    //PID loop calculates error from

    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(m_wristMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    //This also updates the real encoder as used in the PID loop
    m_encoderSim.setDistance(m_armSim.getAngleRads());
    
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_movingAppendage.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    
  }

  public void moveMotorsWithPID(double setpoint){
    System.out.println(setpoint); 
    m_pidValue = m_controller.calculate(m_encoder.getDistance(), setpoint);
    m_wristMotor.setVoltage(m_pidValue); 
  }

  public int rotationToEncoderCount(double angleRads){
    double numRotations = Math.PI*4/angleRads; 
    int encoderCurrentCount = (int)numRotations; 
    return encoderCurrentCount;
  }


}
