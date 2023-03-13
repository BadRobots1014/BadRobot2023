package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import frc.robot.Constants.SensorConstants;

public class ColorSensorSubsystem extends SubsystemBase {
    public ColorSensorV3 sensor = new ColorSensorV3(SensorConstants.kColorSensorPort);
    public double ir;
    public Color detectedColor = Color.kBlack;
    public ColorMatch matcher = new ColorMatch();
    public ColorMatchResult result;

    String colorString = "";

    public ShuffleboardTab mtab = Shuffleboard.getTab("ColorSensor");

    public ColorSensorSubsystem()
    {
      // mtab.addNumber("Blue", this::getBlue);
      // mtab.addNumber("Green", this::getGreen);
      // mtab.addNumber("Red", this::getRed);
      // mtab.addString("Color", this::getColorString);
      mtab.add("Color", getColor());
      mtab.add("Color Match", getClosestColor());

      // matcher.addColorMatch(kBlueTarget);
      // matcher.addColorMatch(kGreenTarget);
      // matcher.addColorMatch(kRedTarget);
      // matcher.addColorMatch(kYellowTarget);
      matcher.addColorMatch(kNeutralColor);
      matcher.addColorMatch(kReachColor);
      matcher.addColorMatch(kLowColor);
    }

    // private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    // private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
    // private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
    // private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

    private final Color kNeutralColor = new Color(0,0,0);
    private final Color kReachColor = new Color(0,0,0);
    private final Color kLowColor = new Color(0,0,0);

    public Color getColor()
    {
      return detectedColor;
    }

    public Color getClosestColor() {
      detectedColor = sensor.getColor();
      ir = sensor.getIR();

      result = matcher.matchClosestColor(detectedColor);

      return result.color;
    }

    public double getRed() {return detectedColor.red;}

    public double getGreen() {return detectedColor.green;}

    public double getBlue() {return detectedColor.blue;}

    // public String getColorString()
    // {
    //     return colorString;
    // }
}
