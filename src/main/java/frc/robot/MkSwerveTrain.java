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
    private MkSwerveModule[] modules = {
        new MkSwerveModule(CANID.MkTrainIds[0], MKCANCODER.offset[0], MKDRIVE.pidf, MKTURN.pidf),
        new MkSwerveModule(CANID.MkTrainIds[1], MKCANCODER.offset[1], MKDRIVE.pidf, MKTURN.pidf),
        new MkSwerveModule(CANID.MkTrainIds[2], MKCANCODER.offset[2], MKDRIVE.pidf, MKTURN.pidf),
        new MkSwerveModule(CANID.MkTrainIds[3], MKCANCODER.offset[3], MKDRIVE.pidf, MKTURN.pidf)
    };

    private MkSwerveTrain()
    {
        vars = new variables();
        vars.deg = new double[4];
        vars.posInch = new double[4];
        vars.posMeters = new double[4];
        vars.posNative = new double[4];
        vars.velInch = new double[4];
        vars.velMeters = new double[4];
        vars.velNative = new double[4];
        vars.mod1 = new double[2];
        vars.mod2 = new double[2];
        vars.mod3 = new double[2];
        vars.mod4 = new double[2];
    }

    public static MkSwerveTrain getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void startTrain()
    {
        for(int i = 0; i < modules.length; i++)
        {
            modules[i].zeroTurn();
            modules[i].zeroDrive();
        }
    }

    public void startDrive()
    {
        modules[0].zeroDrive();
        modules[1].zeroDrive();
        modules[2].zeroDrive();
        modules[3].zeroDrive();
    }

    public void updateSwerve()
    {
        SmartDashboard.putNumber("angle", vars.deg[0]);
        for(int i = 0; i < modules.length; i++)
        {
            //vars.velNative[i] = modules[i].driveMotor().getVelocity();
            

           vars.posNative[i] = modules[i].driveMotor().getSelectedSensorPosition();
            
           // vars.velInch[i] = MathFormulas.nativePer100MstoInchesPerSec(vars.velNative[i]);
            

            vars.posInch[i] = MathFormulas.nativeToInches(vars.posNative[i]);
        
            
           // vars.velMeters[i] = MathFormulas.nativePer100MsToMetersPerSec(vars.velNative[i]);
            

           // vars.posMeters[i] = MathFormulas.nativeToMeters(vars.posNative[i]);

           vars.deg[i] = MathFormulas.nativeToDegrees(modules[i].turnMotor().getSelectedSensorPosition(), MKTURN.greerRatio);
        }
        vars.avgDistInches = (Math.abs(vars.posInch[0]) + Math.abs(vars.posInch[1]) + Math.abs(vars.posInch[2]) + Math.abs(vars.posInch[3])) /4.0;
     //vars.avgVelInches = (vars.velInch[0] + vars.velInch[1] + vars.velInch[2] + vars.velInch[3]) / 4.0;
       // vars.avgVelNative = (vars.velNative[0] + vars.velNative[1] + vars.velNative[2] + vars.velNative[3]) / 4.0;
      vars.avgDeg = (vars.deg[0] + vars.deg[1] + vars.deg[2] + vars.deg[3]) / 4.0;
    }

    public MkSwerveModule[] getModules()
    {
        return modules;
    }

    public void setModuleTurn(double angle)
    {
        modules[0].turnMotor().set(ControlMode.Position, angle);
        modules[1].turnMotor().set(ControlMode.Position, angle);
        modules[2].turnMotor().set(ControlMode.Position, angle);
        modules[3].turnMotor().set(ControlMode.Position, angle);
    }

    public void zero()
    {
        modules[0].zeroDrive();
        modules[1].zeroDrive();
        modules[2].zeroDrive();
        modules[3].zeroDrive();

        modules[0].zeroTurn();
        modules[1].zeroTurn();
        modules[2].zeroTurn();
        modules[3].zeroTurn();
    }

    public void stopEverything()
    {
        modules[0].setModule(0, ControlMode.PercentOutput, 0, ControlMode.PercentOutput);
        modules[1].setModule(0, ControlMode.PercentOutput, 0, ControlMode.PercentOutput);
        modules[2].setModule(0, ControlMode.PercentOutput, 0, ControlMode.PercentOutput);
        modules[3].setModule(0, ControlMode.PercentOutput, 0, ControlMode.PercentOutput);
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

        vars.mod1[1] = Math.atan2(vars.B,vars.C)*180/Constants.kPi;
        vars.mod2[1] = Math.atan2(vars.B,vars.D)*180/Constants.kPi;
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
            
            vars.mod1[0] = Math.sqrt((Math.pow(vars.B, 2)) + (Math.pow(vars.C, 2)));      
            vars.mod2[0] = Math.sqrt((Math.pow(vars.B, 2)) + (Math.pow(vars.D, 2))); 
            vars.mod3[0] = Math.sqrt((Math.pow(vars.A, 2)) + (Math.pow(vars.D, 2)));           
            vars.mod4[0] = Math.sqrt((Math.pow(vars.A, 2)) + (Math.pow(vars.C, 2)));
        
            vars.max=vars.mod1[0]; if(vars.mod2[0]>vars.max)vars.max=vars.mod2[0]; if(vars.mod3[0]>vars.max)vars.max=vars.mod3[0]; if(vars.mod4[0]>vars.max)vars.max=vars.mod4[0];
            if(vars.max>1){vars.mod1[0]/=vars.max; vars.mod2[0]/=vars.max; vars.mod3[0]/=vars.max; vars.mod4[0]/=vars.max;}
        }



        vars.mod1 = setDirection(modules[1].turnMotor().getSelectedSensorPosition(), vars.mod1);
        vars.mod2 = setDirection(modules[0].turnMotor().getSelectedSensorPosition(), vars.mod2);
        vars.mod3 = setDirection(modules[2].turnMotor().getSelectedSensorPosition(), vars.mod3);
        vars.mod4 = setDirection(modules[3].turnMotor().getSelectedSensorPosition(), vars.mod4);

      /*if(mode == ControlMode.MotionMagic)
        {
            vars.mod1[0] = Math.abs(vars.mod1[0]);
            vars.mod2[0] = Math.abs(vars.mod2[0]);
            vars.mod3[0] = Math.abs(vars.mod3[0]);
            vars.mod4[0] = Math.abs(vars.mod4[0]);
        }*/
        SmartDashboard.putNumber("wa1", vars.mod1[1]);
        modules[1].setModule(vars.mod1[0], mode, MathFormulas.degreesToNative(vars.mod1[1], MKTURN.greerRatio));
        modules[0].setModule(vars.mod2[0], mode, MathFormulas.degreesToNative(vars.mod2[1], MKTURN.greerRatio));
        modules[2].setModule(vars.mod3[0], mode, MathFormulas.degreesToNative(vars.mod3[1], MKTURN.greerRatio));
        modules[3].setModule(vars.mod4[0], mode, MathFormulas.degreesToNative(vars.mod4[1], MKTURN.greerRatio));
        //TODO velocity might break the drive pidf
    }



   /**
     * decides whether a driving motor should flip based on where the angular motor's setpoint is.
     * @param position position of the motor
     * @param setpoint setpoint for the motor
     * @return returns best angle of travel for the angular motor, as well as the flip value for the driving motor (as an array so it can return two things in one instead of two seperatly)
     * @author team 6624
     */
    public static double[] setDirection(double position, double[] mod)
    {
        double currentAngle = MathFormulas.nativeToDegrees(position, MKTURN.greerRatio);
        // find closest angle to setpoint
        double setpointAngle = MathFormulas.closestAngle(currentAngle, mod[1]);
        // find closest angle to setpoint + 180
        double setpointAngleFlipped = MathFormulas.closestAngle(currentAngle, mod[1] + 180.0);
        // if the closest angle to setpoint is shorter
        if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped))
        {
            // unflip the motor direction use the setpoint
            return new double[] {mod[0],(currentAngle + setpointAngle)};
        }
        // if the closest angle to setpoint + 180 is shorter
        else
        {
            // flip the motor direction and use the setpoint + 180
            return new double[] {Math.abs(mod[0]) * -1.0, (currentAngle + setpointAngleFlipped)};
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

    public void setDist(double setpoint)
    {
        
    }

    //turn distance is degrees

    public void setEtherAuto(double totalDistance)
    {
        vars.autoDist = MathFormulas.inchesToNative(totalDistance);
        vars.totalDistance = totalDistance;
        vars.avgDistInches = 0;
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
        double RCWtemp = RCWauto;
        double calcangle = (90-(thetaTurn/2))+((vars.avgDistInches/vars.totalDistance)*(thetaTurn));
        if(mode == ETHERAUTO.Curve)
        {
            vars.FWDauto = Math.cos(calcangle* (Constants.kPi/180));//(90-(thetaTurn/2))+((vars.avgDistInches/vars.totalDistance)*(thetaTurn)) * (Constants.kPi/180));//(((-1 * thetaTurn) + (2 * ((vars.avgDistInches/vars.totalDistance)*thetaTurn))) * Constants.kPi / 180);
            vars.STRauto = Math.sin(calcangle* (Constants.kPi/180));//(90-(thetaTurn/2))+((vars.avgDistInches/vars.totalDistance)*(thetaTurn)) * (Constants.kPi/180));//(((-1 * thetaTurn) + (2 * ((vars.avgDistInches/vars.totalDistance)*thetaTurn))) * Constants.kPi / 180);
        }
        else if(mode == ETHERAUTO.Straight)
        {
            vars.FWDauto = Math.cos(thetaTurn);
            vars.STRauto = Math.sin(thetaTurn);
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

        public double STRauto;
        public double FWDauto;
    public double totalDistance;

        public double temp;
        public double yaw;
        public double A;
        public double B;
        public double C;
        public double D;
        public double mod1[];
        public double mod2[];
        public double mod3[];
        public double mod4[];
        public double max;

        /**Distance variable for driving in autonomous*/
        public double straightDistance;

    /**Position of the driving motor in native units*/
    public double[] posNative;
       
    /**Position of the driving motor in inches*/
    public double[] posInch;
   
    /**Position of the driving motor in meters*/
    public double[] posMeters;
       
    /**Velocity of the driving motor in inches*/
    public double[] velInch;
   
    /**Velocity of the driving motor in native units*/
    public double[] velNative;
       
    /**Velocity of the driving motor in meters*/
    public double[] velMeters;
   
    /**Position of the turning motor in degrees*/
    public double[] deg;
   
    /**Driving motor values for autonomous*/
    public double[] output;
   
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
