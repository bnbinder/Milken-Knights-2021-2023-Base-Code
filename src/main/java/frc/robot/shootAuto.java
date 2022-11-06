// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class shootAuto extends CommandBase {
  private Timer shootTimer = new Timer();
  private Timer supportTimer = new Timer();
  private boolean shootTimerFirst, supportTimerFirst = false;
  private Limelight lime = Limelight.getInstance();
  private Shooter shoot = Shooter.getInstance();
  private Elevator elevator = Elevator.getInstance();
  /** Creates a new shootAuto. */
  public shootAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // shoot.setShooter(ControlMode.PercentOutput, xboxOP.getLeftTriggerAxis()/1);
    // shoot.setShooter(ControlMode.Velocity, 8000);
    // lime.setShooterFinal();
    if (!shootTimerFirst) 
    {
      shootTimer.start();
      shootTimerFirst = true;
    }
    if (!supportTimerFirst) 
    {
      supportTimer.start();
      supportTimerFirst = true;
    }

    // shoot.setShooter(ControlMode.Velocity, Math.abs(SHOOOO -
    // shoot.shooterFeedForward(SHOOOO)));
    // lime.setShooterFinal();
    // shoot.setShooter(ControlMode.Velocity, Math.abs(SHOOOO -
    // shoot.shooterFeedForward(SHOOOO)));
    // MkSwerveTrain.getInstance().etherSwerve(0, 0, lime.etherLimeRCWValue(),
    // ControlMode.PercentOutput);
    elevator.setElevator(ControlMode.PercentOutput, -.6);
    if (supportTimer.get() < 1) 
    {
      shoot.setSupport(ControlMode.PercentOutput, .2);
    } 
    
    else 
    {
      Shooter.getInstance().setShooterCalc(4500);
      Shooter.getInstance().setHoodPositionPercent(4500 + 160);
    }
    if (shootTimer.get() > 3) 
    {
      if (shoot.vars.avgShootSpeedNative > 4400) 
      {
        // shoot.setShooter(ControlMode.Velocity, Math.abs(SHOOOO -
        // shoot.shooterFeedForward(SHOOOO)));
        shoot.setSupport(ControlMode.PercentOutput, -.15);
        elevator.setElevator(ControlMode.PercentOutput, .1);
        elevator.setShitter(ControlMode.PercentOutput, -.1);
        SmartDashboard.putBoolean("ShooterSpeed", true);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.getInstance().setShooter(ControlMode.PercentOutput, 0);
    Shooter.getInstance().setHood(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
