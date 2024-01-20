package frc.robot.subsystems;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavXGyroSubsystem extends SubsystemBase {
  
  private final AHRS navx;

  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Navx");

  public NavXGyroSubsystem() {
    navx = new AHRS(SPI.Port.kMXP);
    m_tab.addNumber("Yaw", this::getYaw);
    m_tab.addNumber("Roll", this::getRoll);
    m_tab.addNumber("Pitch", this::getPitch);
  }

  public void periodic() {
    // System.out.print("Yaw: " + getYaw() + "\n" + "Roll: " + getRoll() + "\n" + "Pitch: " + getPitch() + "\n\n");
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

  public double getRate() {
    return navx.getRate();
  }

  public double getVelocityX() {
    return navx.getVelocityX();
  }

  public double getVelocityY() {
    return navx.getVelocityY();
  }

  public double getVelocityZ() {
    return navx.getVelocityZ();
  }

  public void reset() {
    navx.reset();
    navx.resetDisplacement();
  }
}
