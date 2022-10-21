// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import frc.robot.Constants.MKFALCON;
import frc.robot.Constants.MKTURN;
import frc.robot.Constants.MKCANCODER;
import frc.robot.Constants.MKDRIVE;
import frc.robot.Constants.MKELEVATOR;

/** Add your docs here. */
public class Motor {

    private Motor()
    {

    }

    public static Motor getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public TalonFX turnMotor(int canid)
    {
        TalonFX turn = new TalonFX(canid, "train");
        turn.configFactoryDefault();
        turn.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        turn.setNeutralMode(MKTURN.mode);
        turn.config_kP(0, MKTURN.kP);
        turn.config_kI(0, MKTURN.kI);
        turn.config_kD(0, MKTURN.kD);
        turn.config_kF(0, MKTURN.kF);
        turn.setInverted(MKTURN.inverted);      
        turn.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_100Ms);
        turn.configVelocityMeasurementWindow(MKFALCON.velocityMeasAmount);
        turn.configVoltageCompSaturation(MKFALCON.voltComp);
        turn.enableVoltageCompensation(true);
        turn.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, MKFALCON.statusOneMeas);
        turn.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, MKFALCON.statusTwoMeas);

        return turn;
    }

    public TalonFX driveMotor(int canid)
    {
        TalonFX drive = new TalonFX(canid, "train");
        drive.configFactoryDefault();
        drive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        drive.setNeutralMode(MKDRIVE.mode);
        drive.config_kP(0, MKDRIVE.kP);
        drive.config_kI(0, MKDRIVE.kI);
        drive.config_kD(0, MKDRIVE.kD);
        drive.config_kF(0, MKDRIVE.kF);
        drive.setInverted(MKDRIVE.inverted);      
        drive.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_100Ms);
        drive.configVelocityMeasurementWindow(MKFALCON.velocityMeasAmount);
        drive.configVoltageCompSaturation(MKFALCON.voltComp);
        drive.enableVoltageCompensation(true);
        drive.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, MKFALCON.statusOneMeas);
        drive.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, MKFALCON.statusTwoMeas);
        drive.configMotionCruiseVelocity(MKDRIVE.maxNativeVelocity);
        drive.configMotionAcceleration(MKDRIVE.maxNativeAcceleration);
        drive.configMotionSCurveStrength(MKDRIVE.scurve);

        return drive;
    }

    public TalonFX elevatorMotor(int canid)
    {
        TalonFX elevator = new TalonFX(canid, "train");
        elevator.configFactoryDefault();
        elevator.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        elevator.setNeutralMode(MKELEVATOR.elevatorNeutralMode);
        return elevator; 
    }

    public CANCoder cancoder(int canid, double offset)
    {
        CANCoder encoder = new CANCoder(canid, "train");
        encoder.configAbsoluteSensorRange(MKCANCODER.range);
        encoder.configSensorDirection(MKCANCODER.inverted);
        encoder.configMagnetOffset(offset);

        return encoder;
    }


    private static class InstanceHolder
    {
        private static final Motor mInstance = new Motor();
    } 

}
