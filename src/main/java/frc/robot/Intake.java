// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants.CANID;
import frc.robot.Constants.MKINTAKE;

/** Add your docs here. */
public class Intake {
    private Motor mMotor = Motor.getInstance();
    private TalonFX roller;

    private Intake()
    {
        roller = mMotor.motor(CANID.rollerCANID, MKINTAKE.rollerNeutralMode, 0, MKINTAKE.pidf, MKINTAKE.inverted);
    }

    public static Intake getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void move(double setpoint)
    {
        roller.set(ControlMode.PercentOutput, setpoint);
    }

    private static class InstanceHolder
    {
        private static final Intake mInstance = new Intake();
    } 
}
