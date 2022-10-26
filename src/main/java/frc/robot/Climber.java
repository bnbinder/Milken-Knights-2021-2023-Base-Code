// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANID;
import frc.robot.Constants.CLIMBER;

/**The Climber class contains everything relating to the climbing mechanism*/
public class Climber {
    
    TalonFX telescopeArmLeft;
    TalonFX telescopeArmRight;
    private Motor mMotor = Motor.getInstance();

    private Climber()
    {
        telescopeArmLeft = mMotor.motor(CANID.leftClimberCANID, CLIMBER.leftClimbNeutralMode, 0, CLIMBER.pidf, CLIMBER.isLeftInverted);
        telescopeArmRight = mMotor.motor(CANID.rightClimberCANID, CLIMBER.rightClimbNeutralMode, 0, CLIMBER.pidf, !CLIMBER.isLeftInverted);

    }

    public static Climber getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void climberUpdate()
    {
        SmartDashboard.putNumber("right climb", telescopeArmRight.getSelectedSensorPosition());
        SmartDashboard.putNumber("left climb", telescopeArmLeft.getSelectedSensorPosition());
    }

    public void telescopePercent(double percentleft, double percentright)
    {
        telescopeArmLeft.set(ControlMode.PercentOutput, percentleft);
        telescopeArmRight.set(ControlMode.PercentOutput, percentright);
    }

    public void telescopePercentRight(double setpoint)
    {
        telescopeArmRight.set(ControlMode.PercentOutput, setpoint);
    }

    public void telescopePercentLeft(double setpoint)
    {
        telescopeArmLeft.set(ControlMode.PercentOutput, setpoint);
    }


    public void zeroVClimbb()
    {
        telescopeArmRight.setSelectedSensorPosition(0);
        telescopeArmLeft.setSelectedSensorPosition(0);
    }

    public void zeroLeftClimb()
    {
        telescopeArmLeft.setSelectedSensorPosition(0);
    }

    public void zeroRightClimb()
    {
        telescopeArmRight.setSelectedSensorPosition(0);
    }


/**
 * auto function for left arm
 * @param leftGoingUp if true, go up. else, go down
 */
    public void climbAutoLeft(boolean leftGoingUp)
    {
        if(isLeftBelow() && leftGoingUp)
        {
            telescopePercentLeft(1);
        }
        else if(isLeftAbove() && !leftGoingUp)
        {
            telescopePercentLeft(-1);
        }
    }

/**
 * auto function for right arm
 * @param rightGoingUp if true, go up. else, go down
 */
    public void climbAutoRight(boolean rightGoingUp)
    {
        if(isRightBelow() && rightGoingUp)
        {
            telescopePercentRight(1);
        }
        else if(isRightAbove() && !rightGoingUp)
        {
            telescopePercentRight(-1);
        }
    }

    /**is right arm above low point */
    public boolean isRightAbove()
    {
        return telescopeArmRight.getSelectedSensorPosition() > CLIMBER.minNativePosition;
    }
    /**is right arm below high point */
    public boolean isRightBelow()
    {
        return telescopeArmRight.getSelectedSensorPosition() < CLIMBER.maxNativePosition;
    }
    /**is left arm above low point */
    public boolean isLeftAbove()
    {
        return telescopeArmLeft.getSelectedSensorPosition() > CLIMBER.minNativePosition;
    }
    /**is left arm below high point */
    public boolean isLeftBelow()
    {
        return telescopeArmLeft.getSelectedSensorPosition() < CLIMBER.maxNativePosition;
    }
    /**are both arms below high point */
    public boolean isBelow()
    {
        return isLeftBelow() && isRightBelow();
    }
    /**are both arms above low point */
    public boolean isAbove()
    {
        return isLeftAbove() && isRightAbove();
    }

    private static class InstanceHolder
    {
        private static final Climber mInstance = new Climber();
    } 
    
}