// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.MathFormulas;

/** Add your docs here. */
public final class Constants {

    public static final double kPi = 3.14159265359;

    public static class MKFALCON 
    {
        public static final int velocityMeasAmount = 16;
        public static final int statusOneMeas = 25;
        public static final int statusTwoMeas = 25;
        public static final double voltComp = 12;
        public static final double oneEncoderRotation = 2048;
    }

    public static class MKDRIVE 
    {
        public static final double kS = 0.1;
        public static final double kA = 0.1;
        public static final double kV = 0.1;

        public static final double maxNativeVelocity = 21600; 
        public static final double maxNativeAcceleration = maxNativeVelocity / 10;
        
        public static final double kP = 0.21;
        public static final double kI = 0;
        public static final double kD = 0 * kP;
        public static final double kF = 0;// 1023.0 / maxNativeVelocity; //TODO might break the pidf, delete if does

        public static final double[] pidf = {kP, kI, kD, kF};

        public static final NeutralMode mode = NeutralMode.Brake;

        public static final boolean inverted = false;

        public static final int scurve = 6;

        public static final double greerRatio = 6.75;

        public static final double wheelDiameterInches = 4; 
        public static final double wheelCircumference = wheelDiameterInches * kPi;    
    }

    public static class MKTURN 
    {
        public static final double kP = 0.087;//0.00008;
        public static final double kI = 0;
        public static final double kD = 0.00000001;
        public static final double kF = 0;
        
        public static final double[] pidf = {kP, kI, kD, kF};

        public static final NeutralMode mode = NeutralMode.Brake;

        public static final boolean inverted = true;

        public static final int scurve = 6;

        public static final double greerRatio = 12.8;
    }

    public static class MKCANCODER
    {
        public static final double topLeftOffset = -74.53125;
        public static final double topRightOffset = 131.1328125;
        public static final double bottomLeftOffset = -121.640625;
        public static final double bottomRightOffset = 47.197265625;

        public static final double[] offset = {MKCANCODER.topLeftOffset, MKCANCODER.topRightOffset, MKCANCODER.bottomLeftOffset, MKCANCODER.bottomRightOffset};

        public static final AbsoluteSensorRange range = AbsoluteSensorRange.Signed_PlusMinus180;

        public static final boolean inverted = true;
    }

    public static class MKTRAIN 
    {
        public static final double L = 22.57;
        public static final double W = 22.57;

        public static final double widthInches = 29;
        public static final double heightInches = 29;

        public static final double R = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2));

        public static final double hP = 0.001, hI = 0.0001, hD = hP * 0.1;

        public static final double speedLimit = 5;
    }

    public static class CLIMBER 
    {
        public static final double maxNativePosition = 290000;
        public static final double minNativePosition = 9000;
        public static final boolean isLeftInverted = false;
        public static final NeutralMode leftClimbNeutralMode = NeutralMode.Brake;
        public static final NeutralMode rightClimbNeutralMode = NeutralMode.Brake;
        
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        
        public static final double[] pidf = {kP, kI, kD, kF};

        public static final int scurve = 0;

        public static final double climbUpSpeed = 0.1;
        public static final double climbDownSpeed = -0.1;
    }

    public static class SHOOTER 
    {
        public static final double maxNativeShooterVelocity = 18900;
        public static final double maxError = 1580;
        public static final double lowGoalNativeVelocity = 0;
        public static final boolean isLeftInverted = false;
        public static final NeutralMode leftShootNeutralMode = NeutralMode.Coast;
        public static final NeutralMode rightShootNeutralMode = NeutralMode.Coast;

        public static final double kP = 0.55;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;

        public static final double[] pidf = {kP, kI, kD, kF};

        public static final int scurve = 0;

        public static final double kS = 0;
        public static final double kA = 0;
        public static final double kV = 0;
    }

    public static class INTAKE 
    {
        public static final double maxIntakeNativePosition = 7350;
        public static final NeutralMode intakeNeutralMode = NeutralMode.Brake;
        public static final NeutralMode rollerNeutralMode = NeutralMode.Coast;

        public static final double kP = 0.04;
        public static final double kI = 0;
        public static final double kD = kP * 0.6;
        public static final double kF = 0;

        public static final double[] pidf = {kP, kI, kD, kF};

        public static final int scurve = 0;

        public static final double intakePercentSpeed = 0.2;
        public static final double rollerPercentSpeed = 0.5;
    }

    public static class ELEVATOR 
    {
        public static final NeutralMode elevatorNeutralMode = NeutralMode.Coast;

        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;

        public static final double[] pidf = {kP, kI, kD, kF};

        public static final int scurve = 0;

        public static final double elevatorPercentSpeedForward = 0.5;
        public static final double elevatorPercentSpeedBack = 0.2;
    }

    public static class NAVX 
    {
        public static final double offset = 180;
    }

    public static class CONTROLLERS 
    {
        public static final int driverPort = 0;
        public static final int opPort = 1;

        public static class DriveInput 
        {
            public static final int fwd = 1;
            public static final int str = 0;
            public static final int rcw = 4;

            public static final int zeroNavxButton = 3;
            public static final int moveZeroButton = 1;
            public static final int resetDriveButton = 2;
            public static final int zeroEncoderButton = 4;
        }

        public static class ShootInput 
        {
            public static final int forwardShootTrigger = 3;
            public static final int backwardShootTrigger = 2;
        }

        public static class ClimbInput 
        {
            public static final int leftClimbUpPOV = 180;
            public static final int leftClimbDownPOV = 0;
            public static final int rightClimbUpPOV = 270;
            public static final int rightClimbDownPOV = 90;
            public static final int climbAxis = 5;
            public static final int autoClimbButton = 7;
        }

        public static class ElevatorInput 
        {
            public static final int elevatorAxis = 1;
        }

        public static class IntakeInput
        {
            public static final int rollerForwardBumper = 5;
            public static final int rollerBackwardBumper = 6;
            public static final int intakeButton = 1;
        }

        public static final int fakeLimelight = 1;

        public static final int topPOV = 0;
        public static final int rightPOV = 90;
        public static final int bottomPOV = 180;
        public static final int leftPOV = 270;
    }

    public static class CANID 
    {
        public static final int topDriveLeftCANID = 3; 
        public static final int topDriveRightCANID = 5; 
        public static final int bottomDriveLeftCANID = 2; 
        public static final int bottomDriveRightCANID = 7;

        public static final int topTurnLeftCANID = 4; 
        public static final int topTurnRightCANID = 6; 
        public static final int bottomTurnLeftCANID = 1;
        public static final int bottomTurnRightCANID = 8; 

        public static final int topTurnLeftCANCoderCANID = 16; 
        public static final int topTurnRightCANCoderCANID = 18; 
        public static final int bottomTurnLeftCANCoderCANID = 15; 
        public static final int bottomTurnRightCANCoderCANID = 17;

        public static final int leftClimberCANID = 22;
        public static final int rightClimberCANID = 23;

        public static final int leftShooterCANID = 19;
        public static final int rightShooterCANID = 20;

        public static final int intakeCANID = 11;
        public static final int rollerCANID = 9;

        public static final int elevatorCANID = 10;
        
        public static final int[] topLeftCANID = {topDriveLeftCANID, topTurnLeftCANID, topTurnLeftCANCoderCANID};
        public static final int[] topRightCANID = {topDriveRightCANID, topTurnRightCANID, topTurnRightCANCoderCANID};
        public static final int[] bottomLeftCANID = {bottomDriveLeftCANID, bottomTurnLeftCANID, bottomTurnLeftCANCoderCANID};
        public static final int[] bottomRightCANID = {bottomDriveRightCANID, bottomTurnRightCANID, bottomTurnRightCANCoderCANID};
    }

    public static class AUTO
    {

        public static class DISTANGLE 
        {                             
            public static final double distanceA = 50;
            public static final double lengthB = 10;///2;
                                                                              //          /2
            public static final double distanceuno = MathFormulas.calculateArcOfPath(distanceA, lengthB);
            public static final double angleuno = -MathFormulas.calculateAngleOfPath(distanceA, lengthB);
            public static final double distancedos = MathFormulas.calculateArcOfPath(distanceA, lengthB);
            public static final double angledos = MathFormulas.calculateAngleOfPath(distanceA, lengthB);
        }
        //auto controlling pid
        public static final double turnSwerveControlKp = 1;
        public static final double driveSwerveControlKpY = 1;
        public static final double driveSwerveControlKpX = 1;

        public static final double heightMeters = MathFormulas.inchesToMeters(MKTRAIN.heightInches / 2);
        public static final double widthMeters = MathFormulas.inchesToMeters(MKTRAIN.widthInches / 2);

        public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
        new Translation2d(heightMeters, widthMeters),
        new Translation2d(heightMeters, -widthMeters),
        new Translation2d(-heightMeters, widthMeters),
        new Translation2d(-heightMeters, -widthMeters));
      
        
        //actual drive module stats
        public static final double maxModuleTurnVelo = kPi;
        public static final double maxModuleTurnAccel = kPi;
        
        //actual drive module stats
        public static final double maxModuleDriveVelo = 1;
        public static final double maxModuleDriveAccel = 1;
        

        //for turning constraints
        public static final double maxAutoTurnVelo = kPi;
        public static final double maxAutoTurnAccel = kPi;
        
        //for trajectory config
        public static final double maxAutoDriveVelo = 1; //2;
        public static final double maxAutoDriveAccel = 1; //2;


        public static final double maxDriveVelo = 1;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            maxAutoTurnVelo, maxAutoTurnAccel);
    }


    public static class ODO 
    {
        public static final double goalXInches = 120;
        public static final double goalYInches = 120;
        public static final double goalRadius = 60;
    }

    public static class LIGHTS 
    {
        public static final int PWMPORT = 0; 
        public static final int bufferNum = 151; 
        public static final int MaxRGBValue = 60;
    }

    public static class LOGS
    {
      public static final int maxSizeThreshold = 100000;
    }
}
