// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;


public class ArmSubsystem extends SubsystemBase {

  public final CANSparkMax m_winch = new CANSparkMax(ArmConstants.kWinchPort, CANSparkMaxLowLevel.MotorType.kBrushless); // Assume Brushless, unknown currently
  public static final CANSparkMax m_extender = new CANSparkMax(ArmConstants.kExtenderPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  public static final CANSparkMax m_grabber = new CANSparkMax(ArmConstants.kGrabberPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  public final RelativeEncoder m_extenderEncoder;
 // public final Encoder m_winchEncoder = new Encoder(EncoderConstants.kWinchChannelA, EncoderConstants.kExtenderChannelB);
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Arm");

  public int winchTicks;
  public int extenderTicks;
  public boolean grabber;

  public static final double extenderUpperBound = 33;
  public static final double extenderLowerBound = 0;

  public static boolean dunkState = false;

  private final RobotContainer m_RobotContainer;

  

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    m_RobotContainer = new RobotContainer();
    m_grabber.setInverted(false); // Find out if needs to be T/F Later
    m_grabber.setIdleMode(IdleMode.kCoast);

    m_winch.setInverted(false); // Find out if needs to be T/F
    m_winch.setIdleMode(IdleMode.kBrake);


    m_extender.setInverted(true); //needs to be T

    m_extender.setIdleMode(IdleMode.kBrake);

    m_extenderEncoder = m_extender.getEncoder();
    resetEncoder(m_extenderEncoder);

    m_tab.addDouble("Extender Encoder:", this::getExtenderEncoderPosition);
    m_tab.addDouble("Left Z Axis",this::getLeftZAxis);

    }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    
  }

  public void stopMotor(CANSparkMax motor){
    motor.stopMotor();
  }

  public void runGrabber(double power){
    m_grabber.set(clampPower(power));
  }

  public void stopGrabber(){
    m_grabber.stopMotor();
  }

  public void runExtender(double power){
    m_extender.set(clampPower(power));
  }

  public void runWinch(double power){
    m_winch.set(clampPower(power));
  }

  private static double clampPower(double power) {
    return MathUtil.clamp(power, -1.0, 1.0);
  }

  public double getEncoderPosition(RelativeEncoder encoder) {return encoder.getPosition();} // In rotations

  public double getEncoderVelocity(RelativeEncoder encoder) {return encoder.getVelocity();}

  public double getExtenderEncoderPosition(){
    return (getEncoderPosition(m_extenderEncoder));
  }

  public double getLeftZAxis(){
    return((m_RobotContainer.getLeftZ() + 1) / 2.0);
  }

  public double getExtenderUpperBound(){
    return extenderUpperBound;
  }

  public void resetEncoder(RelativeEncoder encoder) {
    encoder.setPosition(0);
  }

  public static void runToPosition(CANSparkMax motor, RelativeEncoder encoder, double pos){
    // if((dunkState == true) && (encoder.getPosition() < extenderUpperBound) && (encoder.getPosition() > extenderLowerBound)){
    //   pos = pos - 3;
    // }
    // double distance = pos - encoder.getPosition();
    // double coefficient = MathUtil.clamp(distance, -1.0, 1.0);
    // if(pos < encoder.getPosition()){
    //   motor.set(0.05 * coefficient);
    //   System.out.println(0.05 * coefficient);
    // }else if(pos > encoder.getPosition()){
    //   motor.set(-0.04 * coefficient);
    //   System.out.println(-0.04 * coefficient);
    // }

    double speed = .04;
    double dis = pos - encoder.getPosition();
    motor.set(clampPower(dis * speed));
  }
}
