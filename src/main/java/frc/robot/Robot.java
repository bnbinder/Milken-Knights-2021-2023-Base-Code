// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SerialPort.Port;
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
  private XboxController xbox = new XboxController(0);
  private double fwd, str, rcw, rcwX, rcwY;
  private MkSwerveTrain train = MkSwerveTrain.getInstance();
  private boolean head = false;
  @Override
  public void robotInit() {
    train.startTrain();
    navx.getInstance().reset();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    
    train.startTrain();
    navx.getInstance().reset();
    if (m_autonomousCommand != null) {
     // SmartDashboard.putBoolean("yuy", true);
      m_autonomousCommand.schedule();
    }
    //SmartDashboard.putNumber("stupid dumbass", DriverStation.getMatchTime());
  }

  @Override
  public void autonomousPeriodic() {
    train.updateSwerve();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    train.startTrain();
    navx.getInstance().reset();

  }
  
  @Override
  public void teleopPeriodic() {
    train.updateSwerve();
    fwd = (xbox.getRawAxis(1) - 0.1) / (1 - 0.1);
    str = (xbox.getRawAxis(0) - 0.1) / (1 - 0.1);
    rcw = (xbox.getRawAxis(5) - 0.1) / (1 - 0.1);
    
    rcwY = rcw;
    rcwX =  (xbox.getRawAxis(4) - 0.1) / (1 - 0.1);
      
      if(Math.abs(xbox.getRawAxis(1)) < 0.1)
      {
        fwd = 0;
      }
      if(Math.abs(xbox.getRawAxis(0)) < 0.1)
      {
        str = 0;
      }
      
      if(xbox.getAButton())
      {
        navx.getInstance().reset();
      }
      if(xbox.getBButton())
      {
        rcw = train.moveToAngy(90);
      }
      else if(xbox.getYButton())
      {
        rcw  = rcw/5;
      }
      else
      {
        rcw = train.moveToAngy(((((((( Math.toDegrees(Math.atan(rcwY/rcwX))+360 ))+ (MathFormulas.signumV4(rcwX)))%360) - MathFormulas.signumAngleEdition(rcwX,rcwY))+360)%360));
      }
      if(Math.abs(xbox.getRawAxis(5)) < 0.1)
      {
        rcw = 0;
        rcwY = 0;
      }
      if(Math.abs(xbox.getRawAxis(4)) < 0.1)
      {
        rcwX = 0;
      }
      //penis

      if(fwd != 0 || str != 0 || rcw != 0)
      {
        //weird negative cuz robot is weird. should be negative fwd positive str rcw
        train.etherSwerve(fwd/5, -str/5, rcw, ControlMode.PercentOutput); //+,-,+
        //mDrive.updateDriveDriveRaw();
      }
      else
      {
        train.stopEverything();
      }
      //SmartDashboard.putNumber("angleplus180", ((Math.toDegrees(Math.atan(fwd/str))+180))%(360*MathFormulas.signumV2(str)));
     //SmartDashboard.putNumber("angleminus180", ((Math.toDegrees(Math.atan(fwd/str))-180))%(360*MathFormulas.signumV2(str))); 
     SmartDashboard.putNumber("doesthiswork",(((((( Math.toDegrees(Math.atan(rcwY/rcwX))+360 ))+ (MathFormulas.signumV4(rcwX)))%360) - MathFormulas.signumAngleEdition(rcwX,rcwY))+360)%360);
    }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    train.startTrain();
    train.stopEverything();
    SmartDashboard.putNumber("anglglgl", DISTANGLE.angle);
    SmartDashboard.putNumber("distttt", DISTANGLE.distance);
    SmartDashboard.putNumber("radi", MathFormulas.calculateCircleRadius(50, 10));
  }

  @Override
  public void testPeriodic() 
  {}
}
