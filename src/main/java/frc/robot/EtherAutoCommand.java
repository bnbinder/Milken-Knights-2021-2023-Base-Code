// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class EtherAutoCommand extends CommandBase {
  /** Creates a new EtherAuto. */
  private double totalDistance;
  private double thetaTurn;
  //private double RCWauto;
  //private ETHERAUTO mode;
  //private ETHERRCW turny;
  //private double turnyAuto;
  private double distanceA;
  private double heading;
  private int side;
  private MkSwerveTrain train = MkSwerveTrain.getInstance();
  
  public EtherAutoCommand(double distanceA, double dist, double ang, int side, double heading)
  {
   /* this.RCWauto = RCWauto;
    this.mode = mode;
    this.turny = turny;*/
    this.totalDistance = dist;
    this.thetaTurn = ang;
   // this.turnyAuto = turnyAuto;
   this.distanceA = distanceA;
    this.heading = heading;
    this.side = side;

    //want theta turn, want turny auto, dont want rcw, curve and specific
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    train.startDrive();
    //train.startTrain();
    //SmartDashboard.putNumber("anglglgl", DISTANGLE.angleuno);
    //SmartDashboard.putNumber("distt", DISTANGLE.distanceuno);
    train.setEtherAuto(totalDistance, distanceA);
    //SmartDashboard.putNumber("dist", heading);
    //SmartDashboard.putNumber("calc", side);
    //SmartDashboard.putBoolean("key", true);
    System.out.println("im starting");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    train.etherAutoUpdate(thetaTurn, heading, side);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    train.stopEverything();
    System.out.println("im done");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // SmartDashboard.putBoolean("false", false);
    return train.isFinished();
  }
}