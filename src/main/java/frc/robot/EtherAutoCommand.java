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
  private double lengthB;
  private double heading;
  private MkSwerveTrain train = MkSwerveTrain.getInstance();
  
  public EtherAutoCommand(double distanceA, double lengthB, double dist, double ang, double heading)
  {
   /* this.RCWauto = RCWauto;
    this.mode = mode;
    this.turny = turny;*/
    this.totalDistance = dist;
    this.thetaTurn = ang;
   // this.turnyAuto = turnyAuto;
   this.distanceA = distanceA;
    this.heading = heading;
    this.lengthB = lengthB;
    //want theta turn, want turny auto, dont want rcw, curve and specific
  }
// youre doing a great job - josh
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("start move dist: " + train.vars.avgDistInches);
    System.out.println("motor dist: " + train.tlDist());
    train.startDrive();
    /*while(Math.abs(train.tlDist()) >= 5)
    {
      System.out.println("buggggggggg!");
      train.startDrive();
    }*/
    
    //train.startTrain();
    //SmartDashboard.putNumber("anglglgl", DISTANGLE.angleuno);
    //SmartDashboard.putNumber("distt", DISTANGLE.distanceuno);
    train.setEtherAuto(totalDistance, distanceA, MathFormulas.calculateCircleRadius(distanceA, lengthB));
    //SmartDashboard.putNumber("dist", heading);
    //SmartDashboard.putNumber("calc", side);
    //SmartDashboard.putBoolean("key", true);
    System.out.println("dist after reset: " + train.vars.avgDistInches);
    System.out.println("motor dist: " + train.tlDist());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    train.etherAutoUpdate(thetaTurn, heading);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("end move dist: " + train.vars.avgDistInches);
    //System.out.println("motor dist: " + train.tlDist());
    train.stopEverything();
    train.startDrive();
    train.vars.avgDistTest = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // SmartDashboard.putBoolean("false", false);
    return train.isFinished();
  }
}