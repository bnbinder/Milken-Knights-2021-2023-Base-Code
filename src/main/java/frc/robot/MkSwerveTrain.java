// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.navx;
import frc.robot.Constants.CANID;
import frc.robot.Constants.MKCANCODER;
import frc.robot.Constants.MKDRIVE;
import frc.robot.Constants.MKTRAIN;
import frc.robot.Constants.MKTURN;
import frc.robot.MkSwerveModule;
import frc.robot.MathFormulas;


/** Add your docs here. */
public class MkSwerveTrain 
{
    public variables vars;
    private MkSwerveModule topLeftModule = new MkSwerveModule(CANID.topLeftCANID, MKCANCODER.offset[0], MKDRIVE.pidf, MKTURN.pidf);
    private MkSwerveModule topRightModule = new MkSwerveModule(CANID.topRightCANID, MKCANCODER.offset[1], MKDRIVE.pidf, MKTURN.pidf);
    private MkSwerveModule bottomLeftModule = new MkSwerveModule(CANID.bottomLeftCANID, MKCANCODER.offset[2], MKDRIVE.pidf, MKTURN.pidf);
    private MkSwerveModule bottomRightModule = new MkSwerveModule(CANID.bottomRightCANID, MKCANCODER.offset[3], MKDRIVE.pidf, MKTURN.pidf);

    private MkSwerveTrain()
    {
        vars = new variables();
        vars.mod1 = new double[4];
        vars.mod2 = new double[4];
        vars.mod3 = new double[4];
        vars.mod4 = new double[4];
    }

    public static MkSwerveTrain getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void startTrain()
    {
        topLeftModule.zeroDrive();
        topRightModule.zeroDrive();
        bottomLeftModule.zeroDrive();
        bottomRightModule.zeroDrive();

        topLeftModule.zeroTurn();
        topRightModule.zeroTurn();
        bottomLeftModule.zeroTurn();
        bottomRightModule.zeroTurn();
    }

    public void startDrive()
    {
        topLeftModule.zeroDrive();
        topRightModule.zeroDrive();
        bottomLeftModule.zeroDrive();
        bottomRightModule.zeroDrive();
    }

    public void updateSwerve()
    {
        //SmartDashboard.putNumber("anglet", vars.deg[0]);
        //SmartDashboard.putNumber("distancet", vars.posInch[0]);
        SmartDashboard.putNumber("distancetopleft", vars.posInchTL);
        SmartDashboard.putNumber("distancetopright", vars.posInchTR);
        SmartDashboard.putNumber("distancetbotleft", vars.posInchBL);
        SmartDashboard.putNumber("distancetbotright", vars.posInchBR);
        vars.posInchTL = MathFormulas.nativeToInches(topLeftModule.getDrivePos());
        vars.posInchTR = MathFormulas.nativeToInches(topRightModule.getDrivePos());
        vars.posInchBL = MathFormulas.nativeToInches(bottomLeftModule.getDrivePos());
        vars.posInchBR = MathFormulas.nativeToInches(bottomRightModule.getDrivePos());

        vars.degTL = topLeftModule.getTurnDeg();
        vars.degTR = topRightModule.getTurnDeg();
        vars.degBL = bottomLeftModule.getTurnDeg();
        vars.degBR = bottomRightModule.getTurnDeg();

        vars.avgDistInches = (Math.abs(vars.posInchTL) + Math.abs(vars.posInchTR) + Math.abs(vars.posInchBL) + Math.abs(vars.posInchBR)) /4.0;
     //vars.avgVelInches = (vars.velInch[0] + vars.velInch[1] + vars.velInch[2] + vars.velInch[3]) / 4.0;
       // vars.avgVelNative = (vars.velNative[0] + vars.velNative[1] + vars.velNative[2] + vars.velNative[3]) / 4.0;
        vars.avgDeg = (vars.degTL + vars.degTR + vars.degBL + vars.degBR) / 4.0;
    }

    public void setModuleTurn(double angle)
    {
        topLeftModule.turnMotor().set(ControlMode.Position, angle);
        topRightModule.turnMotor().set(ControlMode.Position, angle);
        bottomLeftModule.turnMotor().set(ControlMode.Position, angle);
        bottomRightModule.turnMotor().set(ControlMode.Position, angle);
    }

    public void stopEverything()
    {
        topLeftModule.setModule(0, ControlMode.PercentOutput, 0, ControlMode.PercentOutput);
        topRightModule.setModule(0, ControlMode.PercentOutput, 0, ControlMode.PercentOutput);
        bottomLeftModule.setModule(0, ControlMode.PercentOutput, 0, ControlMode.PercentOutput);
        bottomRightModule.setModule(0, ControlMode.PercentOutput, 0, ControlMode.PercentOutput);
    }

     /**
     * See <a href="https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383">this thread</a>
     * for more information.
     * <p>
     * Note - this function uses 180 minus yaw due to the positioning of our navx.
     * @param FWD Forward axis of controller
     * @param STR Strafe axis of controller
     * @param RCW Rotational axis of controller
     * @author Ether
     */
    public void etherSwerve(double FWD, double STR, double RCW, ControlMode mode)
    {
        vars.yaw = navx.getInstance().getNavxYaw();
        vars.temp = FWD * Math.cos(Math.toRadians(vars.yaw)) + STR * Math.sin(Math.toRadians(vars.yaw));
        STR = -FWD * Math.sin(Math.toRadians(vars.yaw)) + STR * Math.cos(Math.toRadians(vars.yaw));
        FWD = vars.temp;

        //SmartDashboard.putNumber("frd", FWD);
        //SmartDashboard.putNumber("str", STR);

        vars.A = STR - RCW*(MKTRAIN.L/MKTRAIN.R);
        vars.B = STR + RCW*(MKTRAIN.L/MKTRAIN.R);
        vars.C = FWD - RCW*(MKTRAIN.W/MKTRAIN.R);
        vars.D = FWD + RCW*(MKTRAIN.W/MKTRAIN.R);

        vars.mod2[1] = Math.atan2(vars.B,vars.C)*180/Constants.kPi;
        vars.mod1[1] = Math.atan2(vars.B,vars.D)*180/Constants.kPi;
        vars.mod3[1] = Math.atan2(vars.A,vars.D)*180/Constants.kPi;
        vars.mod4[1] = Math.atan2(vars.A,vars.C)*180/Constants.kPi; 

      
        if(mode == ControlMode.MotionMagic)
        {
            vars.mod1[0] = vars.autoDist;
            vars.mod2[0] = vars.autoDist;
            vars.mod3[0] = vars.autoDist;
            vars.mod4[0] = vars.autoDist;
        }

        else
        {
            
            vars.mod2[0] = Math.sqrt((Math.pow(vars.B, 2)) + (Math.pow(vars.C, 2)));      
            vars.mod1[0] = Math.sqrt((Math.pow(vars.B, 2)) + (Math.pow(vars.D, 2))); 
            vars.mod3[0] = Math.sqrt((Math.pow(vars.A, 2)) + (Math.pow(vars.D, 2)));           
            vars.mod4[0] = Math.sqrt((Math.pow(vars.A, 2)) + (Math.pow(vars.C, 2)));
        
            vars.max=vars.mod1[0]; if(vars.mod2[0]>vars.max)vars.max=vars.mod2[0]; if(vars.mod3[0]>vars.max)vars.max=vars.mod3[0]; if(vars.mod4[0]>vars.max)vars.max=vars.mod4[0];
            if(vars.max>1){vars.mod1[0]/=vars.max; vars.mod2[0]/=vars.max; vars.mod3[0]/=vars.max; vars.mod4[0]/=vars.max;}
        }



        vars.mod1 = setDirection(topLeftModule.getTurnDeg(), vars.mod1);
        vars.mod2 = setDirection(topLeftModule.getTurnDeg(), vars.mod2);
        vars.mod3 = setDirection(topLeftModule.getTurnDeg(), vars.mod3);
        vars.mod4 = setDirection(topLeftModule.getTurnDeg(), vars.mod4);

      /*if(mode == ControlMode.MotionMagic)
        {
            vars.mod1[0] = Math.abs(vars.mod1[0]);
            vars.mod2[0] = Math.abs(vars.mod2[0]);
            vars.mod3[0] = Math.abs(vars.mod3[0]);
            vars.mod4[0] = Math.abs(vars.mod4[0]);
        }*/
        SmartDashboard.putNumber("wa1", vars.mod1[1]);
        topLeftModule.setModule(vars.mod1[0], mode, MathFormulas.degreesToNative(vars.mod1[1], MKTURN.greerRatio));
        topRightModule.setModule(vars.mod2[0], mode, MathFormulas.degreesToNative(vars.mod2[1], MKTURN.greerRatio));
        bottomLeftModule.setModule(vars.mod3[0], mode, MathFormulas.degreesToNative(vars.mod3[1], MKTURN.greerRatio));
        bottomRightModule.setModule(vars.mod4[0], mode, MathFormulas.degreesToNative(vars.mod4[1], MKTURN.greerRatio));
        //TODO velocity might break the drive pidf
    }



   /**
     * decides whether a driving motor should flip based on where the angular motor's setpoint is.
     * @param position position of the motor
     * @param setpoint setpoint for the motor
     * @return returns best angle of travel for the angular motor, as well as the flip value for the driving motor (as an array so it can return two things in one instead of two seperatly)
     * @author team 6624
     */
    public static double[] setDirection(double pos, double[] mod)
    {
        double currentAngle = pos;
        // find closest angle to setpoint
        double setpointAngle = MathFormulas.closestAngle(currentAngle, mod[1]);
        // find closest angle to setpoint + 180
        double setpointAngleFlipped = MathFormulas.closestAngle(currentAngle, mod[1] + 180.0);
        // if the closest angle to setpoint is shorter
        if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped))
        {
            // unflip the motor direction use the setpoint
            //an = currentAngle + setpointAngle;
            return new double[] {mod[0],(currentAngle + setpointAngle)};
        }
        // if the closest angle to setpoint + 180 is shorter
        else
        {
            // flip the motor direction and use the setpoint + 180
            //di = Math.abs(di) * -1.0; 
            //an = currentAngle + setpointAngleFlipped;
            return new double[] {Math.abs(mod[0]) * -1,(currentAngle + setpointAngleFlipped)};
        }
    }

    //programming done right
    public double headerStraighter(double hSetpoint)
    {
        if(hSetpoint != 361)
        {
            vars.hError = hSetpoint -  navx.getInstance().getNavxYaw();// Error = Target - Actual
            vars.hIntegral += (vars.hError*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
            vars.hDerivative = (vars.hError - vars.hPreviousError) / .02;
            return vars.hP*vars.hError + vars.hI*vars.hIntegral + vars.hD*vars.hDerivative;
        }
        else
        {
            return 0;
        }
    }

    //turn distance is degrees

    public void setEtherAuto(double totalDistance, double distanceA, int side, double heading)
    {
        startDrive();
        vars.autoDist = MathFormulas.inchesToNative(totalDistance);
        vars.totalDistance = totalDistance;
        vars.avgDistInches = 0;
        vars.distanceA = distanceA;
        vars.side = side;
        vars.heading = heading;
    }

    /**
     * Using the {@link #swerveAutonomousEther} and motion magic, an autonomous angled path of motion can be achieved
     * @param totalDistance Length of curved path
     * @param thetaTurn Angle of curved path
     * @param RCWauto [-1, 1] For spinny, 0 for no spinny
     * @param mode Curve or Straight
     * @param turny Specific or Infinite
     * @param turnyAuto (if using specific for turny) angle that robot tries to keep when moving
    */
    public void etherAutoUpdate(double thetaTurn, double RCWauto, ETHERAUTO mode, ETHERRCW turny, double turnyAuto)
    {
                                            //numbers fall short of high by 3ish inches and short of length by 4ish inches
        double RCWtemp = RCWauto; //50,10 = 15 ... 40,10 = 10 ... 30,10 = 5 ... 20,10 = 0 <-- (even if just circle, 4 inches from height but hits target)
                                                                            //minus subtracotr
        double calcangle = ((vars.heading) + (vars.side * (thetaTurn/2))+((vars.avgDistInches/(vars.distanceA))*(thetaTurn)));
        if(mode == ETHERAUTO.Curve)
        {
            vars.FWDauto = Math.cos(calcangle* (Constants.kPi/180));//(90-(thetaTurn/2))+((vars.avgDistInches/vars.totalDistance)*(thetaTurn)) * (Constants.kPi/180));//(((-1 * thetaTurn) + (2 * ((vars.avgDistInches/vars.totalDistance)*thetaTurn))) * Constants.kPi / 180);
            vars.STRauto = Math.sin(calcangle* (Constants.kPi/180));//(90-(thetaTurn/2))+((vars.avgDistInches/vars.totalDistance)*(thetaTurn)) * (Constants.kPi/180));//(((-1 * thetaTurn) + (2 * ((vars.avgDistInches/vars.totalDistance)*thetaTurn))) * Constants.kPi / 180);
        }
        else if(mode == ETHERAUTO.Straight)
        {
            vars.FWDauto = Math.cos(thetaTurn* (Constants.kPi/180));
            vars.STRauto = Math.sin(thetaTurn* (Constants.kPi/180));
        }
        if(turny == ETHERRCW.Specific)
        {
            RCWtemp = headerStraighter(turnyAuto);
        }
        else 
        {
            RCWtemp = RCWauto;
        }
        etherSwerve(vars.FWDauto, -vars.STRauto, RCWtemp, ControlMode.MotionMagic);
        SmartDashboard.putNumber("calc", calcangle);
        SmartDashboard.putNumber("dist", vars.avgDistInches);
    }

    public boolean isFinished()
    {
        return Math.abs(vars.avgDistInches) >= Math.abs(vars.totalDistance) - 0.1;
    }

    /**Mode of the ether auto's path*/
    public enum ETHERAUTO
    {
        Straight, Curve
    }

    /**Mode of the ether auto's turn */
    public enum ETHERRCW
    {
        Specific, Forever
    }

    public enum MODE
    {
        auto, tele;
    }

    private static class InstanceHolder
    {
        private static final MkSwerveTrain mInstance = new MkSwerveTrain();
    } 

    public static class variables
    {

        public double heading;
        public int side;
        public double distanceA;
        public double STRauto;
        public double FWDauto;
    public double totalDistance;

        public double temp;
        public double yaw;
        public double A;
        public double B;
        public double C;
        public double D;
        public double[] mod1;
        public double[] mod2;
        public double[] mod3;
        public double[] mod4;
        public double max;

        /**Distance variable for driving in autonomous*/
        public double straightDistance;

    /**Position of the driving motor in native units*/
    public double posNativeTL, posNativeTR, posNativeBL, posNativeBR;
       
    /**Position of the driving motor in inches*/
    public double posInchTL, posInchTR, posInchBL, posInchBR;
   
    /**Position of the driving motor in meters*/
    public double posMetersTL, posMetersTR, posMetersBL, posMetersBR;
       
    /**Velocity of the driving motor in inches*/
    public double velInchTL, velInchTR, velInchBL, velInchBR;
   
    /**Velocity of the driving motor in native units*/
    public double velNativeTL, velNativeTR, velNativeBL, velNativeBR;
       
    /**Velocity of the driving motor in meters*/
    public double velMetersTL, velMetersTR, velMetersBL, velMetersBR;
   
    /**Position of the turning motor in degrees*/
    public double degTL, degTR, degBL, degBR;
   
    /**Driving motor values for autonomous*/
    //public double[] output;
   
    /**Average velocity of driving motors in inches*/
    public double avgVelInches;
   
    /**Average velocity of driving motors in native units*/
    public double avgVelNative;
   
    /**Average distance of driving motors in inches*/
    public double avgDistInches;

    public double avgDeg;

    public variables var;
    public double hP = 0.001, hI = 0.0001, hD = hP * 0.1;
    public double hIntegral, hDerivative, hPreviousError, hError;

    public double autoDist;
    }
}
