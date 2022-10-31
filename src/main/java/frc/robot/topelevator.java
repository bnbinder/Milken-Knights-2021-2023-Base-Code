// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants.CANID;
import frc.robot.Constants.MKELEVATOR;


public class topelevator {
    private Motor mMotor = Motor.getInstance();
    private TalonFX topElevator;

    private topelevator()
    {
        topElevator = mMotor.motor(CANID.elevatorCANID, MKELEVATOR.topelevator, 0, Constants.nullPID, MKELEVATOR.topelevatorInverted);
    }

    public static topelevator getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void settopelevator(ControlMode mode, double setpoint)
    {
        topelevator.set(mode, setpoint);
    }
    private static void set(ControlMode mode, double setpoint) {
    }
    private static class InstanceHolder
    {
        private static final topelevator mInstance = new topelevator();
    }

    
}
