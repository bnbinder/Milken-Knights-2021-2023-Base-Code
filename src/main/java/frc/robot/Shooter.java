// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants.CANID;
import frc.robot.Constants.MKTURRET;

/** Add your docs here. */
public class Shooter {
    private TalonFX turret;
    private Motor mMotor = Motor.getInstance();

    private Shooter()
    {
        turret = mMotor.motor(CANID.turretCANID, MKTURRET.mode, 0, MKTURRET.pidf, MKTURRET.inverted);
    }
}
