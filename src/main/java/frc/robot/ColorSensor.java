/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

public class ColorSensor {

  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch m_colorMatcher = new ColorMatch();

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  private final Color kBlueTarget = new Color(0.181396484375, 0.411865234375, 0.406982421875);
  private final Color kFarBlueTarget = new Color(0.32666015625, 0.431884765625, 0.24169921875);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kRedTarget = new Color(0.408203125, 0.39599609375, 0.196044921875);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);


  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
 private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
 private int proximity = m_colorSensor.getProximity();
 private Color detectedColor = m_colorSensor.getColor();
 private double IR =  m_colorSensor.getIR();




 private String colorString;
 private ColorMatchResult match;
 
 public static ColorSensor getInstance()
 {
     return InstanceHolder.mInstance;
 }

  public ColorSensor()
  {

  }

  public void colorInit()
  {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);   
  }

  public Color getDetectedColor()
  {
/**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    return detectedColor;
  }
    
  public double getIR()
  {
    /**
     * The sensor returns a raw IR value of the infrared light detected.
     */
    return IR;
  }

  public int getProximity()
  {
    return proximity;
  }

  public void updateColor()
  {
    proximity = m_colorSensor.getProximity();
    detectedColor = m_colorSensor.getColor();
    IR = m_colorSensor.getIR();

     /**
     * Run the color match algorithm on our detected color
     */
    
    match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kFarBlueTarget) {
      colorString = "FarBlue";
    } else {
      colorString = "Unknown";
    }


    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    
     /**
     * In addition to RGB IR values, the color sensor can also return an 
     * infrared proximity value. The chip contains an IR led which will emit
     * IR pulses and measure the intensity of the return. When an object is 
     * close the value of the proximity will be large (max 2047 with default
     * settings) and will approach zero when the object is far away.
     * 
     * Proximity can be used to roughly approximate the distance of an object
     * or provide a threshold for when an object is close enough to provide
     * accurate color values.
     */
    
  }

  public void colorSmartDashboard()
  {
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putNumber("Proximity", proximity);
    SmartDashboard.putNumber("red", m_colorSensor.getRed());
    SmartDashboard.putString("Detected Color", colorString);

    SmartDashboard.putNumber("Confidence", match.confidence);
  }

  
  private static class InstanceHolder
  {
      private static final ColorSensor mInstance = new ColorSensor();
  } 
}




/*

test for distances

measurmenents from robot to wall 
(robot distance - length of sensor = actual distance)

ir - inches


800 - 1 inch

1500 - 33 inches
the heck happened between these numbers (._.)
2000 - 46 inches

3500 - 57 inches 

4100 - 67 inches

5000 - 108 inches


in conclsion - the color distance sensor is kinda poopy (if my data is correct)

*/