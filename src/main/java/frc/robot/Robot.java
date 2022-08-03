// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AUTO.DISTANGLE;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private Command m_autonomousCommand;
  private MkSwerveTrain train = MkSwerveTrain.getInstance();
  private ColorSensor color = ColorSensor.getInstance();
  private SupaStruct supaKoopa = SupaStruct.getInstance();
  private UltraSensor ultra = UltraSensor.getInstance();
  private SerialPort arduino;
  private Timer timer;
  private String keyIn = "";
  private boolean accessible, in = false;
  private String bullshit = "30937C22";


  @Override
  public void robotInit() 
  {
    try
    {
      arduino = new SerialPort(9600, "/dev/ttyACM0", SerialPort.Port.kUSB, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
      //arduino = new SerialPort(9600, SerialPort.Port.kUSB);
      System.out.println("Connected on usb port zero!");
      in = true;
    }
    catch(Exception e1)
    {
      System.out.println("Failed to connect on usb port two, failed all usb ports. Is your Ardunio plugged in?");
      in = false;
    }
  
    timer = new Timer();
    timer.start();

    train.startTrain();
    navx.getInstance().reset();
  }

  @Override
  public void robotPeriodic() 
  {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() 
  {
    
    train.startTrain();
    navx.getInstance().reset();
    if (m_autonomousCommand != null) 
    {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() 
  {
    train.updateSwerve();
  }

  @Override
  public void teleopInit() 
  {
    color.colorInit();
    if (m_autonomousCommand != null) 
    {
      m_autonomousCommand.cancel();
    }
    train.startTrain();
    navx.getInstance().reset();
    accessible = false;
    keyIn = "";
  }
  
  @Override
  public void teleopPeriodic() 
  {
   // keyIn = arduino.readString();//.substring(10, 24);
    /*if(keyIn.length() > 12)
    {
      //substring because idk why but arduino reads number but when put into a string things get funky
      keyIn = keyIn.substring(0,12);
    }
    */
    if(in)
    {
      if(timer.get() > 2)
      {
  
          keyIn = arduino.readString();
        
        SmartDashboard.putString("string", keyIn);
        System.out.println(keyIn);
        timer.reset();
      }

      SmartDashboard.putBoolean("acess", accessible);

      if(keyIn.equals(bullshit))
      {
        accessible = true;
      }

      SmartDashboard.putBoolean("your uin", true);
    }

    supaKoopa.updateTele();
    color.updateColor();
    ultra.updateUltra();

    ultra.ultraSmartDashboard();
    color.colorSmartDashboard();
  }

  @Override
  public void disabledInit()
  {

  }

  @Override
  public void disabledPeriodic()
  {

  }

  @Override
  public void testInit() 
  {
    train.startTrain();
    train.stopEverything();
    SmartDashboard.putNumber("anglglgl", DISTANGLE.angle);
    SmartDashboard.putNumber("distttt", DISTANGLE.distance);
    SmartDashboard.putNumber("radi", MathFormulas.calculateCircleRadius(50, 10));
  }

  @Override
  public void testPeriodic() 
  {

  }

  
}
