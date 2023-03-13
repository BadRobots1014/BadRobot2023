package frc.robot.subsystems;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GyroConstants;

public class NavXGyroSubsystem extends SubsystemBase {
  
  private final AHRS navx;

  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Navx");

  public NavXGyroSubsystem() {
    navx = new AHRS(SPI.Port.kMXP);
    m_tab.addNumber("Yaw", this::getYaw);
    m_tab.addNumber("Roll", this::getRoll);
    m_tab.addNumber("Pitch", this::getPitch);
  }

  public double getYaw() {
    return navx.getYaw();
  }

  public double getPitch() {
    return navx.getPitch();
  }

  public double getRoll() {
    return navx.getRoll();
  }

  public double getRollPID() {
    // deadzoning
    if(Math.abs(getRoll()) <= GyroConstants.kBalanceThreshold)
      return 0.0;
    // dividing by 20 means that if the robot is 20 deg off target, it will balance with max speed
    // this prevents having controll effort of over one within -20 to 20 range
    return navx.getRoll() / GyroConstants.kMaxSpeedAngle;
  }

  public void reset() {
    navx.reset();
  }
}
