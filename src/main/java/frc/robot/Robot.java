// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.hal.util.UncleanStatusException;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.CANID;
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
  private String bull = "30937C22";
  private Compressor mCompressor ;//= new Compressor(CANID.revphCANID, PneumaticsModuleType.REVPH);


  @Override
  public void robotInit() 
  {
        System.out.println("Robot enabled");

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
  
  try {
    mCompressor = new Compressor(CANID.revphCANID, PneumaticsModuleType.REVPH);
  } catch (UncleanStatusException ex) {
    DriverStation.reportError("Error creating Solenoid", ex.getStackTrace());
  }
  mCompressor.enableDigital();
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
    m_autonomousCommand = new SwrvTest();
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
    System.out.println(mCompressor.toString());
        System.out.println("Robot Teleop Init");
    SmartDashboard.putNumber("distance total", Constants.AUTO.DISTANGLE.distance);
    SmartDashboard.putNumber("angle real", Constants.AUTO.DISTANGLE.angle);
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
    //dont use this (i think)
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
        
        System.out.println(keyIn);
        timer.reset();
      }

    
      if(keyIn.equals(bull))
      {
        accessible = true;
      }

    }

    supaKoopa.updateTele();
  }

  @Override
  public void disabledInit()
  {
        System.out.println("Robot disabled");
    supaKoopa.teleopDisabled();
    //mCompressor.disable();
    m_autonomousCommand = new SwrvTest();
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
