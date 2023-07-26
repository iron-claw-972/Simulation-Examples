package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * These are various constants for the wrist. 
 */
public class WristConstants {

  public static final int kMotorID = 0;

  public static final  DCMotor m_armGearbox = DCMotor.getVex775Pro(2);
  public static final double kGearing = (20.0/1.0) * (62.0/34.0) * (48.0/18.0);
  public static final double kArmLength = Units.inchesToMeters(16.1);
 
  //calculate MOI using the center of gravity distance and weight
  public static final double kCOGWeight = Units.lbsToKilograms(7.3);
  public static final double kCOGDistance = Units.inchesToMeters(8.11);
 
  /** Wrist moment of inertia represents how hard it is to angularly accelerate (ie spin) something. */
  public static final double kMomentOfInertia = kCOGWeight * kCOGDistance * kCOGDistance; // 0.1405

  public static final double kMinAngleRadsHardStop = -1*(Math.PI/2);
  public static final double kMaxAngleRadsHardStop = 1*Math.PI/2;
  
  //any setpoint that we set to it
  public static final double kMinAngleRadsSoftStop = -1;
  public static final double kMaxAngleRadsSoftStop = 1;

  //minimum and maximum power that we can send to the motor. Can change to make faster/slower. -1 is minimum and 1 is maximum
  public static final double kMinPower = -0.5;
  public static final double kMaxPower = 0.5;

  //sometimes for gravity compensation you have to adjust the deadband of the falcon500 motor. Or else the motor won't move. 
  public static final double kGravityCompensation = 0.035; 

  //PID constants
  public static final double kP = 0.4; 
  public static final double kI = 0; 
  public static final double kD = 0.01; 

  //conversion constant
  public static final double kEncoderTicksToRadsConversion = 2*Math.PI/2048;


}
