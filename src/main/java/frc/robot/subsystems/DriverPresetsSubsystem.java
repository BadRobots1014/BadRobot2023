package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriverPresetsSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public static HashMap<Integer, String> Drivers = new HashMap<Integer, String>();
  public DriverPresetsSubsystem() {
    Drivers.put(1, "DRIVERNAME");
    DrivetrainSubsystem.m_tab.addString("Current Driver String", this::getDriver);
    DrivetrainSubsystem.m_tab.addInteger("Current Driver Number", this::getCurrentDriver);
  }

  public static int CurrentDriver = 0;

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
  public int getCurrentDriver()
  {
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
