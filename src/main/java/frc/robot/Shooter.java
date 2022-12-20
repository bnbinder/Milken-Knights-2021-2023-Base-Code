// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANID;
import frc.robot.Constants.MKELEVATOR;
import frc.robot.Constants.MKHOOD;
import frc.robot.Constants.MKSHOOTER;

/**The Climber class contains everything relating to the climbing mechanism*/
public class Shooter {
    public variables vars;
    
    private TalonFX hood;
    private TalonFX elevatorSupport;
    private TalonFX shootRight;
    private TalonFX shootLeft;
    private Motor mMotor = Motor.getInstance(); 
    private PIDController hoodPID;

    private Shooter()
    {
        vars = new variables();
        hood = mMotor.motor(CANID.hoodCANID, MKHOOD.mode, 0, Constants.nullPID, MKHOOD.inverted);
        elevatorSupport = mMotor.motor(CANID.elevatorSupportCANID, MKELEVATOR.supportMode, 0, Constants.nullPID, MKELEVATOR.supportInverted);
        shootRight = mMotor.motor(CANID.leftShooterCANID, MKSHOOTER.leftShootNeutralMode, 0, MKSHOOTER.pidf, MKSHOOTER.isLeftInverted);
        shootLeft = mMotor.motor(CANID.rightShooterCANID, MKSHOOTER.rightShootNeutralMode, 0, MKSHOOTER.pidf, !MKSHOOTER.isLeftInverted);
        hoodPID = new PIDController(MKHOOD.kP, MKHOOD.kI, MKHOOD.kD);
    }

    public static Shooter getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void updateShooter()
    {
        vars.leftShootVelocityNative = shootLeft.getSelectedSensorVelocity();
        vars.rightShootVelocityNative = shootRight.getSelectedSensorVelocity();
        vars.avgShootSpeedNative = (vars.leftShootVelocityNative + vars.rightShootVelocityNative) / 2;
        vars.leftShootError = shootLeft.getClosedLoopError();
        vars.rightShootError = shootRight.getClosedLoopError();
    }


    public void setShooter(ControlMode mode, double setpoint)
    {
        shootLeft.set(mode, setpoint);
        shootRight.set(mode, setpoint);
    }
    
    public void setShooterLeft(ControlMode mode, double setpoint)
    {
        shootLeft.set(mode, setpoint);
    }
    
    public void setShooterRight(ControlMode mode, double setpoint)
    {
        shootRight.set(mode, setpoint);
    }

    /**
     * Calculates feedforward for the shooter. i have been told i should be using majority kF, and little kP. lol.
     * @param setpoint Native velocity setpoint in the {@link #setShooterNativeVelocity} function
     * @return Feedforward that should be added when setting a setpoint
     * @see {@link #setShooterNativeVeloctiy(setpoint)}
     */
    public double shooterFeedForward(double setpoint)
    {
        return MKSHOOTER.maxError * (Math.cos((Constants.kPi / 2) * (1+(setpoint / MKSHOOTER.maxNativeShooterVelocity))));
    }

    public void setShooterCalc(double setpoint)
    {
        setShooter(ControlMode.Velocity, setpoint - shooterFeedForward(setpoint));
    }







    public void setSupport(ControlMode mode, double setpoint)
    {
        elevatorSupport.set(mode, setpoint);
    }







    public void setHood(ControlMode mode, double setpoint)
    {
        hood.set(mode, setpoint);
    }

    public double calcHood(double pos)
    {
        return MathFormulas.negativeLimit(-pos, -MKHOOD.minPosition, -MKHOOD.maxPosition);
    }

    public double hoodFeedForward(double setpoint)
    {
        //cos zero = pi/2
        //set zero to minimum steady state
        return MathFormulas.limit(MKHOOD.minPosition * (Math.cos(((Constants.kPi * 1.1) / (MKHOOD.maxPosition * 2)) * setpoint)), 0, MKHOOD.minPosition);
        //      original equation that was working 
        //!     600 or 700 * (Math.cos(((Constants.kPi * 1.1 or 1) / (4000 * 2)) * setpoint));
    }

    public double hoodposiitongettt()
    {
        return hood.getSelectedSensorPosition();
    }

    public void setHoodPositionPercentFF(double pos)
    {
        hood.set(ControlMode.PercentOutput, hoodPID.calculate(hood.getSelectedSensorPosition(), MathFormulas.negativeLimit(-pos - hoodFeedForward(pos), -100, -MKHOOD.maxPosition)));
    }

    public void setHoodPositionPercent(double pos)
    {
        hood.set(ControlMode.PercentOutput, MathFormulas.limit(hoodPID.calculate(hood.getSelectedSensorPosition(), -pos),-0.13,0.13));
        SmartDashboard.putNumber("setpoint real", MathFormulas.negativeLimit(-pos, -100, -MKHOOD.maxPosition));
        SmartDashboard.putNumber("hood percent", MathFormulas.limit(hoodPID.calculate(hood.getSelectedSensorPosition(), -pos),-0.13,0.13));
        
    }
    
    public void zeroHood()
    {
        hood.setSelectedSensorPosition(0);
    }

    





    private static class InstanceHolder
    {
        private static final Shooter mInstance = new Shooter();
    } 

    public static class variables
    {

        public double avgShootSpeedNative;
        public double rightShootVelocityNative;
        public double leftShootVelocityNative;

        public double leftShootError;
        public double rightShootError;
    }
}