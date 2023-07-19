package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * These are various constants for the wrist. 
 */
public class WristConstants {
  public static final int motorPortSim = 0;
  public static final  DCMotor m_armGearbox = DCMotor.getVex775Pro(2);
  public static final double kGearing = (20.0/1.0) * (62.0/34.0) * (48.0/18.0);

  public static final double kArmLength = Units.inchesToMeters(16.1);
  
  //calculate MOI using the center of gravity distance and weight
  public static final double kCOGWeight = Units.lbsToKilograms(7.3);
  public static final double kCOGDistance = Units.inchesToMeters(8.11);
  
  /** Wrist moment of inertia represents how hard it is to angularly accelerate (ie spin) something. */
  public static final double kMomentOfInertia = kCOGWeight * kCOGDistance * kCOGDistance; // 0.1405

  /**
   * The positive/negative sign indicates which way the arm is allowed to rotate. Negative means the arm can rotate
   * clockwise and positive means the arm can rotate counter-clockwise. Take a look at this unit circle:
   * https://www.remind.com/tutoring-answers/what-is-a-unit-circle
   * We are saying the max angle of rotation is Math.PI/2 radians(up at the top of the circle), rotating counterclockwise. 
   * We are saying the min angle of rotation is -3*Math.PI/2 radians(down at the bottom of the circle), rotating clockwise. 
   */
  public static final double kMaxAngleRads = 1*Math.PI/2;
  public static final double kMinAngleRads = -1*(3*Math.PI/2);

  public static final double kP = 1; 
  public static final double kI = 0; 
  public static final double kD = 0.04; 

}
