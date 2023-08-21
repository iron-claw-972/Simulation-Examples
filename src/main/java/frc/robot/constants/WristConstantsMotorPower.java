package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class WristConstantsMotorPower {
  public static final int kMotorID = 0;

  public static final  DCMotor m_armGearbox = DCMotor.getVex775Pro(2);
  public static final double kGearing = (20.0/1.0) * (62.0/34.0) * (48.0/18.0);
  public static final double kArmLength = Units.inchesToMeters(16.1);

  //COG = center of gravity
  public static final double kCOGWeight = Units.lbsToKilograms(7.3);
  public static final double kCOGDistance = Units.inchesToMeters(8.11);
 
  /** Moment of inertia represents how hard it is to angularly accelerate (ie spin) something. */
  public static final double kMomentOfInertia = kCOGWeight * kCOGDistance * kCOGDistance; // 0.1405

  //the min and max angles of the wrist
  public static final double kMinAngleDegrees = -90;
  public static final double kMaxAngleDegrees = 90;
  
  //PID constants
  public static final double kP = 0.4; 
  public static final double kI = 0; 
  public static final double kD = 0.01; 


}
