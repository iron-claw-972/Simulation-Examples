package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * These are various constants for the wrist. 
 */
public class WristConstants {
  public static final int kMotorID = 0;

  //In reality, when we turn on our robot, the wrist will likely be at our bottom hard stop.
  //The wrist looks like this(downard 90 degree angle, in radians this number is -PI/2): ---|
  //                                                                                        |
  //When we turn out robot on, our inbuilt motor encoders get reset to 0 no matter where it is.
  //When we command the robot to go to a 0 radian angle(completely flat), in reality, completely flat is PI/2 radians for this example, NOT 0. 
  //So if we give our wrist a setpoint of 0 radians, we need the wrist to actually go PI/2 radians. Therefore, we add an OFFSET of PI/2 radians.  
  //So when we turn our robot on, we need to add positive PI/2 to whatever setpoints we command the wrist to go to. 
  //We additionally add this offset to soft stops to get the wrist to move to the actual angle it needs to go to. 
  
  public static final double kSetpointOffsetRads = Math.PI/2; 

  public static final  DCMotor m_armGearbox = DCMotor.getVex775Pro(2);
  public static final double kGearing = (20.0/1.0) * (62.0/34.0) * (48.0/18.0);
  public static final double kArmLength = Units.inchesToMeters(16.1);
 
  //calculate MOI using the center of gravity distance and weight
  public static final double kCOGWeight = Units.lbsToKilograms(7.3);
  public static final double kCOGDistance = Units.inchesToMeters(8.11);
 
  /** Wrist moment of inertia represents how hard it is to angularly accelerate (ie spin) something. */
  public static final double kMomentOfInertia = kCOGWeight * kCOGDistance * kCOGDistance; // 0.1405

  //the angle of real hard stops on a wrist
  public static final double kMinAngleRadsHardStop = -1*(Math.PI/2);
  public static final double kMaxAngleRadsHardStop = 1*Math.PI/2;
  
  //realistically we don't want the arm to hit the hard stops to avoid damage, so the wrist won't go past these angles
  public static final double kMinAngleRadsSoftStop = -1.3;
  public static final double kMaxAngleRadsSoftStop = 1.3;

  //minimum and maximum power that we can send to the motor. Can change to make faster/slower. -1 is minimum and 1 is maximum
  public static final double kMinPower = -0.5;
  public static final double kMaxPower = 0.5;

  //PID constants
  public static final double kP = 0.4; 
  public static final double kI = 0; 
  public static final double kD = 0.01; 

  //conversion constant to convert from encoder ticks to radians. There are 2048 ticks per rotation for this encoder(the number varies by encoder)
  public static final double kEncoderTicksToRadsConversion = 2*Math.PI/2048;


}
