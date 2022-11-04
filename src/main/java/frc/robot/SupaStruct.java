// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.ProcessBuilder.Redirect;
import java.sql.Driver;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.Drake;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MKBABY;
import frc.robot.Constants.MKCOLOR;
import frc.robot.Constants.CONTROLLERS.DriveInput;

/** Add your docs here. */
public class SupaStruct {
    
    private XboxController xbox = new XboxController(0);
    private XboxController xboxOP = new XboxController(1);
    private double fwd, fwdSignum, str, strSignum, leftjoy, rcw, rcwX, rcwY, inverseTanAngleOG, inverseTanAngleDrive, povValue, hoodPosSet, hoooooodvaaaalluuueee, SHOOOO = 0;
    private MkSwerveTrain train = MkSwerveTrain.getInstance();
    private Shooter shoot = Shooter.getInstance();
    private Intake intake = Intake.getInstance();
    private Climber Climb = Climber.getInstance();
    private Elevator elevator = Elevator.getInstance();
    private Limelight lime = Limelight.getInstance();
    private ColorSensor color = ColorSensor.getInstance();
    private boolean resetNavx, shootTimerFirst, elevatorOvveride, ballEnterOvverride, colorCheckStartTimer, resetDrive, xbutton, ybutton,rbbutton,rbbutton2, lbbutton2, lbbutton,abutton, ltrigger, rtrigger,  pov, povToggled, itsreal = false;
    private Climber mClimb = Climber.getInstance();
    private Timer colorCheckTimer = new Timer();
    private Timer shootTimer = new Timer();
   
    public static SupaStruct getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void initTele()
    {
        SmartDashboard.putNumber("hoodPosSet", 0);
        SmartDashboard.putNumber("percentoutputvalauueueue", 0);
        SmartDashboard.putNumber("SHOOOO", 0);
    }

    public void updateTele()
    {
        //TODO seperate all smartdashboard update from main update in update function in all classes
        //--------------------------------------------------------------------//
        //  UPDATES
        //--------------------------------------------------------------------//

        train.updateSwerve();
        color.updateColor();
        //ultra.updateUltra();
        mClimb.climberUpdate();
        shoot.updateShooter();
        lime.updateSensors();
        //ultra.ultraSmartDashboard();
        color.colorSmartDashboard();
        lime.limeSmartDashboard();

        //--------------------------------------------------------------------//
        //  VARIABLES
        //--------------------------------------------------------------------//

        fwd = (xbox.getRawAxis(DriveInput.fwd) - 0.1) / (1 - 0.1);
        fwdSignum = Math.signum(fwd) * -1;
        str = (xbox.getRawAxis(DriveInput.str) - 0.1) / (1 - 0.1);
        strSignum = Math.signum(str) * -1;
        rcw = (xbox.getRawAxis(DriveInput.rcwY) - 0.1) / (1 - 0.1);
        rcwY = rcw;
        //Todo see if making this x breaks it
        rcwX =  (xbox.getRawAxis(DriveInput.rcwX) - 0.1) / (1 - 0.1);
        resetNavx = xbox.getRawButton(DriveInput.resetNavxButton);
        resetDrive = xbox.getRawButton(DriveInput.resetDriveButton);
        xbutton = xbox.getXButton();
        abutton = xbox.getAButtonPressed();
        rbbutton = xbox.getRightBumper();
        rbbutton2 = xboxOP.getRightBumper();
        lbbutton = xbox.getLeftBumper();
        lbbutton2 = xboxOP.getLeftBumper();
        ltrigger = Math.abs(xboxOP.getRawAxis(2)) > 0.1;
        rtrigger = Math.abs(xboxOP.getRawAxis(3)) > 0.1;
        leftjoy = Math.abs(xboxOP.getRawAxis(1));
        ybutton = xboxOP.getYButton();
        pov = xbox.getPOV() != -1;

        hoodPosSet = SmartDashboard.getNumber("hoodPosSet", 0);
        //hoooooodvaaaalluuueee = SmartDashboard.getNumber("percentoutputvalauueueue", 0);
        SHOOOO = SmartDashboard.getNumber("SHOOOO", 0);
//      i dont remember how i got this lol
        inverseTanAngleOG = ((((((Math.toDegrees(Math.atan(rcwY/rcwX))+360 )) + 
                            (MathFormulas.signumV4(rcwX)))%360) - 
                            MathFormulas.signumAngleEdition(rcwX,rcwY))+360)
                            %360;

     inverseTanAngleDrive = ((((((Math.toDegrees(Math.atan(fwd/str))+360 )) + 
                            (MathFormulas.signumV4(str)))%360) - 
                            MathFormulas.signumAngleEdition(str,fwd))+360)
                            %360;
        
        //--------------------------------------------------------------------//
        // BUTTONS
        //--------------------------------------------------------------------//
        
        if(resetNavx)
        {
            /*navx.getInstance().reset();
            povValue = 0;
            inverseTanAngleOG = 0;
            shoot.zeroHood();*/
            lime.autoRotate();
            SmartDashboard.putBoolean("fuckyou", resetNavx);
        }
        if(resetDrive)
        {
            MkSwerveTrain.getInstance().vars.avgDistInches = 0;
            MkSwerveTrain.getInstance().startDrive();
            //str = Math.cos(inverseTanAngleDrive* (Constants.kPi/180));
            //fwd = Math.sin(inverseTanAngleDrive* (Constants.kPi/180));
        }

        //--------------------------------------------------------------------//
        //  POV ROTATION
        //--------------------------------------------------------------------//
        
//      for toggle so povValue doesnt equal -1 and toggle for povToggle
        if(pov)
        {
            povValue = xbox.getPOV();
            povToggled = true;
        }
        
//      if statments
        /*if(ybutton)
        {
            rcw = rcwX/5;
            povToggled = false;
        }       */
        if(Math.abs(xbox.getRawAxis(DriveInput.rcwY)) >= 0.1 || Math.abs(xbox.getRawAxis(DriveInput.rcwX)) >= 0.1)
        {
            //rcw = train.moveToAngy((inverseTanAngleOG + 270) % 360);
            rcw = rcwX;
            povToggled = false;
            //!povToggled is so moving the stick disabled the auto rotate pov function (like in video games, shooting a gun disables the ability to sprint)
        }
        else if(povToggled)
        {
            rcw = train.moveToAngy((povValue+180)% 360);
        }
        
        //this is useless, remove entire variable if you want
//      else statements
        if(/*!ybutton&&*/ !povToggled && /*!bbutton&&*/ Math.abs(xbox.getRawAxis(DriveInput.rcwY)) < 0.1 && Math.abs(xbox.getRawAxis(DriveInput.rcwX)) < 0.1)
        {
            rcw = 0;
        }

        

        //--------------------------------------------------------------------//
        //  ROLLER CONTROL
        //--------------------------------------------------------------------//
        
        if(rbbutton)
        {
            intake.rollerSet(-.3);
            elevator.setElevator(ControlMode.PercentOutput, -0.3);
        }
        else if(lbbutton)
        {
            intake.rollerSet(.3);
            elevator.setElevator(ControlMode.PercentOutput, 0.3);
        }
        else
        {
            intake.rollerSet(0);
        }

        //--------------------------------------------------------------------//
        //  INTAKE DEPLOY CONTROL
        //--------------------------------------------------------------------//
        if(abutton)
        {
            System.out.println(!itsreal);
            itsreal = !itsreal;
            intake.intakeSet(!intake.getIntakeState());
        }

        //--------------------------------------------------------------------//
        //  ELEVATOR AND SHITTER CONTROL
        //--------------------------------------------------------------------//
    if(!ballEnterOvverride)
    {
        if(rbbutton2)
        { 
            //elevatorOvveride = true;
            elevator.setElevator(ControlMode.PercentOutput,.3);
            elevator.setShitter(ControlMode.PercentOutput,-.3);
        }
        else if(lbbutton2)
        {
            //elevatorOvveride = true;
            elevator.setElevator(ControlMode.PercentOutput,-.3);
            elevator.setShitter(ControlMode.PercentOutput,.3);
        }
        else
        {
            elevator.setShitter(ControlMode.PercentOutput,0);
            //elevatorOvveride = false;
        }
    }

    ////////////////////////////////////////////////////////////////////////////

//if(!elevatorOvveride)
//{
        if(color.getColor() == DriverStation.getAlliance().toString())
        {
            ballEnterOvverride = true;
            if(!colorCheckStartTimer)
            {
                colorCheckTimer.start();
                colorCheckStartTimer = true;
            }
            if(colorCheckTimer.get() > 0.8)
            {
                elevator.setShitter(ControlMode.PercentOutput, 0.2);
                elevator.setElevator(ControlMode.PercentOutput, -0.2);
                shoot.setSupport(ControlMode.PercentOutput, -.05);
            }
        }
        else if(color.getColor() == MKCOLOR.unkown)
        {
            elevator.setShitter(ControlMode.PercentOutput, 0);
        
            colorCheckTimer.stop();
            colorCheckStartTimer = false;
            ballEnterOvverride = false;
        }
        else
        {
            ballEnterOvverride = true;
            elevator.setShitter(ControlMode.PercentOutput, -0.2);
            elevator.setElevator(ControlMode.PercentOutput, -.2);
            shoot.setSupport(ControlMode.PercentOutput, .05);
            colorCheckTimer.stop();
            colorCheckTimer.reset();
            colorCheckStartTimer = false;
        }
//    }
       
       
    /*   
            }(DriverStation.getAlliance().toString()==color.getColor())
        {
            elevator.setShitter(ControlMode.PercentOutput, 0);
        }
        else if (DriverStation.getAlliance().toString()!=color.getColor())
        {
            elevator.setShitter(ControlMode.PercentOutput, 0.3);
        }

*/
        //--------------------------------------------------------------------//
        //  CLIMBER CONTROL
        //--------------------------------------------------------------------//

        //--------------------------------------------------------------------//
        //  SHOOTER CONTROL
        //--------------------------------------------------------------------//
        SmartDashboard.putNumber("timeeeeeeee", shootTimer.get());
        if(ltrigger)
        {
            //shoot.setShooter(ControlMode.PercentOutput, xboxOP.getLeftTriggerAxis()/1);
            //shoot.setShooter(ControlMode.Velocity, 8000);
            //lime.setShooterFinal();
            if(!shootTimerFirst)
            {
                shootTimer.start();
                shootTimerFirst = true;
            }

            shoot.setShooter(ControlMode.Velocity, Math.abs(SHOOOO - shoot.shooterFeedForward(SHOOOO)));
            
            elevator.setElevator(ControlMode.PercentOutput,-.1);
            if(shootTimer.get() > 3)
            {
            if(shoot.vars.avgShootSpeedNative > SHOOOO-76)
            {
            shoot.setSupport(ControlMode.PercentOutput, .15);
            elevator.setElevator(ControlMode.PercentOutput,-.1);
            elevator.setShitter(ControlMode.PercentOutput,.1);
            SmartDashboard.putBoolean("fuck", true);
            }
        }
            

        }
        else
        {
            shoot.setShooter(ControlMode.PercentOutput, 0);
            shoot.setSupport(ControlMode.PercentOutput, .0);
            shootTimer.stop();
            shootTimer.reset();
            shootTimerFirst = false;
            SmartDashboard.putBoolean("fuck", false);
        }


        SmartDashboard.putNumber("possssssss", hoodPosSet);
        SmartDashboard.putNumber("shooterlsidieer", SHOOOO);
        SmartDashboard.putNumber("fffff subtracto numero uno", hoodPosSet+shoot.hoodposiitongettt());
        SmartDashboard.putNumber("ffshoot", shoot.shooterFeedForward(SHOOOO) - SHOOOO);
        if(xbutton)
        {
            SmartDashboard.putNumber("fffffff", shoot.hoodFeedForward(hoodPosSet));
            //SmartDashboard.putNumber("possssssss", hoodPosSet = SmartDashboard.getNumber("hoodPosSet", 0));
            //shoot.setHoodPositionPercent(3000);
            shoot.setHoodPositionPercent(hoodPosSet + 160);
        }
        else
        {
            shoot.setHood(ControlMode.PercentOutput, 0);
        }


        //--------------------------------------------------------------------//
        //  ELSE STATEMENTS
        //--------------------------------------------------------------------//
        
        if(Math.abs(xbox.getRawAxis(DriveInput.rcwY)) < 0.1)
        {
            rcwY = 0;
        }
        if(Math.abs(xbox.getRawAxis(DriveInput.rcwX)) < 0.1)
        {
            rcwX = 0;
        }

        if(Math.abs(xbox.getRawAxis(DriveInput.fwd)) < 0.1)
        {
            fwd = 0;
        }
        if(Math.abs(xbox.getRawAxis(DriveInput.str)) < 0.1)
        {
            str = 0;
        }

        if(!rbbutton && !lbbutton && !rbbutton2 && !lbbutton2 && !elevatorOvveride)
        {
            elevator.setElevator(ControlMode.PercentOutput, 0);
        }
        

//     applying numbers
        if(!resetNavx && fwd != 0 || str != 0 || rcw != 0)
        {//+,-,+
            train.etherSwerve(fwd/MKBABY.fwdBABY, -str/MKBABY.strBABY, rcw/MKBABY.rcwBABY, ControlMode.PercentOutput); //+,-,+
          
        }
        else
        {
            SmartDashboard.putBoolean("fuckyou", resetNavx);
            train.stopEverything();
        }
        SmartDashboard.putBoolean("ballovverride", ballEnterOvverride);
        SmartDashboard.putBoolean("elevatorovverride", elevatorOvveride);
        
    }

    public void teleopDisabled()
    {
        resetNavx = false;
        resetDrive = false;
        xbutton = false;
        ybutton = false;
        pov = false;
        povToggled = false;
        itsreal = false;
    }

    private static class InstanceHolder
    {
            private static final SupaStruct mInstance = new SupaStruct();
    } 



}