// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants.CANID;
import frc.robot.Constants.MKELEVATOR;

/** Add your docs here. */
public class Elevator {
    private Motor mMotor = Motor.getInstance();
    private TalonFX elevator;
    private TalonFX shitter;

    private Elevator()
    {
        elevator = mMotor.motor(CANID.elevatorCANID, MKELEVATOR.elevatorNeutralMode, 0, Constants.nullPID, MKELEVATOR.elevatorInverted);
        shitter = mMotor.motor(CANID.shitterCANID, MKELEVATOR.shitterMode, 0, Constants.nullPID, MKELEVATOR.shitterInverted);
    }

    public static Elevator getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void setElevator(ControlMode mode, double setpoint)
    {
        elevator.set(mode, setpoint);
    }

    public void setShitter(ControlMode mode, double setpoint)
    {
        shitter.set(mode, setpoint);
    }
    
    private static class InstanceHolder
    {
        private static final Elevator mInstance = new Elevator();
    }

    public void settopelevator(ControlMode percentoutput, double leftjoy) {
    }

}
