package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class DriverPresetsSubsystem extends SubsystemBase {
  public static HashMap<Integer, String> Drivers = new HashMap<Integer, String>();
  public DriverPresetsSubsystem() {
    Drivers.put(1, "DRIVERNAME");
    DrivetrainSubsystem.m_tab.addString("Current Driver String", this::getDriver);
    DrivetrainSubsystem.m_tab.addInteger("Current Driver Number", this::getCurrentDriver);
    DrivetrainSubsystem.m_tab.addBoolean("6 pressed", RobotContainer.button6::getAsBoolean);
  }

  public static int CurrentDriver = 1;

  public static final int DRIVERNAME = 1;
  //Drivers add desired Constants

  public static int getkThrottleButton()
  {
    if (CurrentDriver == 0)
    {
        return 2;
    }
    if (CurrentDriver == DRIVERNAME)
    {
        return 3;
    }
    else
    {
        CurrentDriver = 0;
        return 2;
    }
    //Drivers can add their desired button here
  }

  public String getDriver()
  {
    if (Drivers.containsKey(CurrentDriver))
    {
        return Drivers.get(CurrentDriver);
    }
    else 
    {
        return "Out Of Range";
    }
    
  }
  private Boolean is6down = false;
  private Boolean is7down = false;
  public int getCurrentDriver()
  {
    if (!RobotContainer.button6.getAsBoolean() && is6down)
    {
      CurrentDriver++;
      is6down = false;
    }
    if (RobotContainer.button6.getAsBoolean())
    {
      is6down = true;
    }

    if (!RobotContainer.button7.getAsBoolean() && is7down)
    {
      CurrentDriver--;
      is7down = false;
    }
    if (RobotContainer.button7.getAsBoolean())
    {
      is7down = true;
    }

    return CurrentDriver;
  } 



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
