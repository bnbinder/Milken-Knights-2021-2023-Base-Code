// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.MKCANCODER;
import frc.robot.Constants.MKDRIVE;
import frc.robot.Constants.MKFALCON;
import frc.robot.Constants.MKTURN;
import frc.robot.MathFormulas;

/** Add your docs here. */
public class MkSwerveModule
{
    private TalonFX drive;
    private TalonFX turn;
    private CANCoder encoder;
    private SimpleMotorFeedforward driveFeed;
    private double setpoint = 0;

    public MkSwerveModule(int[] canid, double offset, double pidd[], double pidt[])
    {  
        drive = new TalonFX(canid[0]);
        turn = new TalonFX(canid[1]);
        encoder = new CANCoder(canid[2]);
        driveFeed = new SimpleMotorFeedforward(MKDRIVE.kS, MKDRIVE.kV, MKDRIVE.kA);
        drive.configFactoryDefault();
        drive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        drive.setNeutralMode(MKDRIVE.mode);
        drive.config_kP(0, pidd[0]);
        drive.config_kI(0, pidd[1]);
        drive.config_kD(0, pidd[2]);
        drive.config_kF(0, pidd[3]);
        drive.setInverted(MKDRIVE.inverted);      
        drive.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        drive.configVelocityMeasurementWindow(MKFALCON.velocityMeasAmount);
        drive.configVoltageCompSaturation(MKFALCON.voltComp);
        drive.enableVoltageCompensation(true);
        drive.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, MKFALCON.statusOneMeas);
        drive.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, MKFALCON.statusTwoMeas);
        drive.configMotionCruiseVelocity(MKDRIVE.maxNativeVelocity);
        drive.configMotionAcceleration(MKDRIVE.maxNativeAcceleration);
        drive.configMotionSCurveStrength(MKDRIVE.scurve);

        turn.configFactoryDefault();
        turn.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        turn.setNeutralMode(MKTURN.mode);
        turn.config_kP(0, pidt[0]);
        turn.config_kI(0, pidt[1]);
        turn.config_kD(0, pidt[2]);
        turn.config_kF(0, pidt[3]);
        turn.setInverted(MKTURN.inverted);      
        turn.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
        turn.configVelocityMeasurementWindow(MKFALCON.velocityMeasAmount);
        turn.configVoltageCompSaturation(MKFALCON.voltComp);
        turn.enableVoltageCompensation(true);
        turn.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, MKFALCON.statusOneMeas);
        turn.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, MKFALCON.statusTwoMeas);

        encoder.configAbsoluteSensorRange(MKCANCODER.range);
        encoder.configSensorDirection(MKCANCODER.inverted);
        encoder.configMagnetOffset(offset);
        //this.turn.setPIDF(MKTURN.pidf);
    }

    public void setModule(double setpoint, ControlMode driveMode, double angle, ControlMode turnMode)
    {
        if(driveMode == ControlMode.Velocity)
        {
            this.setpoint = setpoint;
            //setpoint += driveFeed.calculate(drive.getVelocity(), setpoint, 0.02); //TODO might break code
        }
        drive.set(driveMode, setpoint);
        turn.set(turnMode, angle);
    }

    public void setModule(double setpoint, ControlMode mode, double angle)
    {
        setModule(setpoint, mode, angle, ControlMode.Position);
    }
    
    public double getFF()
    {
       return driveFeed.calculate(drive.getSelectedSensorVelocity(), this.setpoint, 0.02); 
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(MathFormulas.nativePer100MsToMetersPerSec(drive.getSelectedSensorVelocity()), new Rotation2d(Math.toRadians(encoder.getAbsolutePosition())));
      }
      
      public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
            SwerveModuleState.optimize(desiredState, new Rotation2d(Math.toRadians(encoder.getAbsolutePosition())));
            drive.set(ControlMode.Velocity, MathFormulas.metersPerSecondToNativeUnitsPer100Ms(state.speedMetersPerSecond));  // + driveFeedforward);
            turn.set(ControlMode.Position, MathFormulas.degreesToNative(state.angle.getDegrees(), MKTURN.greerRatio));  // + turnFeedforward);
      }            

    public void zeroTurn()
    {
        turn.setSelectedSensorPosition(MathFormulas.degreesToNative(encoder.getAbsolutePosition(), MKTURN.greerRatio));
    }

    public void zeroDrive()
    {
        drive.setSelectedSensorPosition(0);
    }

    public TalonFX driveMotor()
    {
        return drive;
    }

    public TalonFX turnMotor()
    {
        return turn;
    }

    public double getDrivePos()
    {
        return drive.getSelectedSensorPosition();
    }

    public double getTurnDeg()
    {
        return MathFormulas.nativeToDegrees(turn.getSelectedSensorPosition(), MKTURN.greerRatio);
    }

    public void configKp(double setpoint)
    {
        drive.config_kP(0, setpoint);
    }

    public CANCoder encoder()
    {
        return encoder;
    }
}
