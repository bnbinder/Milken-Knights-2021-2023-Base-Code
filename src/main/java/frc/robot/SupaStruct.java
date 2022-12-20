// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MKBABY;
import frc.robot.Constants.MKCOLOR;
import frc.robot.Constants.CONTROLLERS.ClimbInput;
import frc.robot.Constants.CONTROLLERS.DriveInput;
import frc.robot.Constants.CONTROLLERS.ElevatorInput;

/**Robot stuff in here*/
public class SupaStruct {
    
    private XboxController xbox = new XboxController(0);
    private XboxController xboxOP = new XboxController(1);
    private double fwd, fwdSignum, str, strSignum, leftjoy, rcw, rcwX, rcwY, inverseTanAngleOG, inverseTanAngleDrive, povValue, hoodPosSet, hoooooodvaaaalluuueee, SHOOOO, navxRotate = 0;
    private MkSwerveTrain train = MkSwerveTrain.getInstance();
    private Shooter shoot = Shooter.getInstance();
    private Intake intake = Intake.getInstance();
    private Climber Climb = Climber.getInstance();
    private Elevator elevator = Elevator.getInstance();
    private Limelight lime = Limelight.getInstance();
    private ColorSensor color = ColorSensor.getInstance();
    private boolean resetNavx, shootTimerFirst, supportTimerFirst, elevatorOvveride, ballEnterOvverride, colorCheckStartTimer, resetDrive, xbutton, ybutton,rbbutton,rbbutton2, lbbutton2, lbbutton,abutton, ltrigger, rtrigger,  pov, /*povToggled,*/ itsreal = false;
    private boolean isRCWrunningWithNavx = false; 
    private Climber mClimb = Climber.getInstance();
    private AprilTags april = AprilTags.getInstance();
    private Timer colorCheckTimer = new Timer();
    private Timer shootTimer = new Timer();
    private Timer supportTimer = new Timer();


    private Timer turntesttimer = new Timer();
    private Timer turntesttimertwo = new Timer();
    private double count = 0;
    
    public static SupaStruct getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void initTele()
    {
        navxRotate = navx.getInstance().getNavxYaw();
        SmartDashboard.putNumber("hoodPosSet", 0);
        SmartDashboard.putNumber("SHOOOO", 0);
    }

    public void updateTele()
    {
        //TODO seperate all smartdashboard update from main update in update function in all classes
        //--------------------------------------------------------------------//
        //  UPDATES
        //--------------------------------------------------------------------//
        mClimb.climberUpdate();
        train.updateSwerve();
        color.updateColor();
        april.updateApril();
        //ultra.updateUltra();
        mClimb.climberUpdate();
        shoot.updateShooter();
        lime.updateSensors();
        april.aprilSmartDashboard();
        //ultra.ultraSmartDashboard();
        //color.colorSmartDashboard();
        //lime.limeSmartDashboard();

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
        lbbutton = xbox.getLeftBumper();
        ltrigger = Math.abs(xbox.getRawAxis(2)) > 0.1;
        rtrigger = Math.abs(xbox.getRawAxis(3)) > 0.1;
        pov = xbox.getPOV() != -1;

        hoodPosSet = SmartDashboard.getNumber("hoodPosSet", 0);
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
            navx.getInstance().reset();
            povValue = 0;
            inverseTanAngleOG = 0;
            train.vars.avgDistTest = 0;
            train.vars.avgDistInches = 0;
            train.startDrive();
        }

        //--------------------------------------------------------------------//
        //  POV ROTATION
        //--------------------------------------------------------------------//
       
        if(Math.abs(xbox.getRawAxis(DriveInput.rcwX)) >= 0.1)
        {
            rcw = rcwX;
        }
        if(Math.abs(rcwX) >= 0.1)
        {
            navxRotate = navx.getInstance().getNavxYaw();
        }
        else if(!ltrigger && isRCWrunningWithNavx)
        {
            rcw = train.moveToAngy(navxRotate);
        }
        
        //this is useless, remove entire variable if you want
//      else statements (should be at bottom but what the heck ill do it next season)
        if(!ltrigger && Math.abs(xbox.getRawAxis(DriveInput.rcwY)) < 0.1 && Math.abs(xbox.getRawAxis(DriveInput.rcwX)) < 0.1)
        {
            rcw = 0;
        }

        //no rcw<0.1 = rcw = 0 because they want rcw running constantly for heading correction for navx, only ovverride is shooter for now

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
            intake.intakeSet(!intake.getIntakeState());
        }

        //--------------------------------------------------------------------//
        //  ELEVATOR AND SHITTER CONTROL
        //--------------------------------------------------------------------//
        if(!ballEnterOvverride)
        {
            if(rtrigger)
            { 
                elevator.setElevator(ControlMode.PercentOutput,-.3);
                elevator.setShitter(ControlMode.PercentOutput,.3);
            }
            else
            {

            }
        }

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
                elevator.setElevator(ControlMode.PercentOutput, -0.4);
                shoot.setSupport(ControlMode.PercentOutput, -.05);
            }
        }
        else if(color.getColor() == MKCOLOR.unkown)
        {        
            colorCheckTimer.stop();
            colorCheckStartTimer = false;
            ballEnterOvverride = false;
        }
        else
        {
            ballEnterOvverride = true;
            elevator.setShitter(ControlMode.PercentOutput, -0.2);
            elevator.setElevator(ControlMode.PercentOutput, -.4);
            shoot.setSupport(ControlMode.PercentOutput, .05);
            colorCheckTimer.stop();
            colorCheckTimer.reset();
            colorCheckStartTimer = false;
        }

        //--------------------------------------------------------------------//
        //  CLIMBER CONTROL
        //--------------------------------------------------------------------//
        
        
        if(xbox.getRawButton(ClimbInput.zeroClimb))
        {
            mClimb.zeroLeftClimb();
            mClimb.zeroRightClimb();
        }
        
        
        if(xbox.getRawButton(2))
        {
            mClimb.telescopePercentLeft(-0.5);
        }
        else if(xbox.getRawButton(ClimbInput.upClimbButton))
        {
            mClimb.telescopePercentLeft(0.5);
        }
        else
        {
            mClimb.telescopePercentLeft(0);
        }

        if(xbox.getRawButton(2))
        {
            mClimb.telescopePercentRight(-0.5);
        }
        else if(xbox.getRawButton(ClimbInput.upClimbButton))
        {
            mClimb.telescopePercentRight(0.5);
        }
        else
        {
            mClimb.telescopePercentRight(0);
        }





//TODO climber could be simple but we copy paste cuz we late loooool        

        //--------------------------------------------------------------------//
        //  SHOOTER CONTROL
        //--------------------------------------------------------------------//

        SmartDashboard.putNumber("timeeeeeeee", shootTimer.get());
        if(ltrigger)
        {
            if(!shootTimerFirst)
            {
                shootTimer.start();
                shootTimerFirst = true;
            }
            if(!supportTimerFirst)
            {
                supportTimer.start();
                supportTimerFirst = true;
            }
            rcw = lime.etherLimeRCWValue();
            elevator.setElevator(ControlMode.PercentOutput,-.6);
            if(supportTimer.get() < 1)
            {

            }
            else
            {
                lime.setShooterFinal();
            }
            if(shootTimer.get() > 3)
            {
                if(shoot.vars.avgShootSpeedNative > InterpoLerpo.getInstance().shooterInterpoLerpo(lime.getDistance())-100)
                {
                    elevator.setElevator(ControlMode.PercentOutput,.6);
                    elevator.setShitter(ControlMode.PercentOutput,.1);
                    SmartDashboard.putBoolean("ShooterSpeed", true);
                }
            }
        }
        else
        {
            shoot.setShooter(ControlMode.PercentOutput, 0);
            shootTimer.stop();
            shootTimer.reset();
            shootTimerFirst = false;
            supportTimer.stop();
            supportTimer.reset();
            supportTimerFirst = false;
            SmartDashboard.putBoolean("ShooterSpeed", false);
        }

        if(xbox.getRawButton(ElevatorInput.supportForward))
        {
            shoot.setSupport(ControlMode.PercentOutput, .15);
        }
        else
        {
            shoot.setSupport(ControlMode.PercentOutput, 0);
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

        if(!rbbutton && !lbbutton && !rtrigger && !elevatorOvveride)
        {
            elevator.setElevator(ControlMode.PercentOutput, 0);
            
        }

        if(!rtrigger && color.getColor() == MKCOLOR.unkown && !ltrigger)
        {
            elevator.setShitter(ControlMode.PercentOutput, 0);
        }
        

//     applying numbers
        if((fwd != 0 || str != 0 || rcw != 0))
        {//+,-,+
            train.etherSwerve(fwd/MKBABY.fwdBABY, -str/MKBABY.strBABY, rcw/MKBABY.rcwBABY, ControlMode.PercentOutput); //+,-,+
          
        }
        else
        {
            train.stopEverything();
        }
    }

    public void teleopDisabled()
    {
        resetNavx = false;
        resetDrive = false;
        xbutton = false;
        ybutton = false;
        pov = false;
        itsreal = false;
        turntesttimer.stop();
        turntesttimer.reset();
    }

   public void initTest()
   {
    train.vars.avgDistTest = 0;
    turntesttimer.stop();
    turntesttimer.reset();
    turntesttimertwo.stop();
    turntesttimertwo.reset();
    train.startTrain();
   }

//measured over predicted * predicted
    public void updateTest()
    {
        double fwd = 0;
        double rcw = 0;
        if(xbox.getAButtonPressed())
        {
            turntesttimer.start();
        }
        if(turntesttimer.get() > 0.00000000000000001 && turntesttimer.get() < 5)
        {
            fwd = 0.3;
        }
        if(xbox.getRawAxis(4) > 0.1 && (turntesttimer.get() > 0.00000000000000001 && turntesttimer.get() < 5))
        {
            rcw = 0.5;
            count++;
        }
        
        if(fwd == 0.3 || rcw == 0.5)
        {
            train.etherSwerve(fwd, 0, rcw, ControlMode.PercentOutput);
            train.etherRCWFinder(fwd, 0, 0);
        }
        else
        {
            train.stopEverything();
        }

        SmartDashboard.putNumber("count", count);
        SmartDashboard.putNumber("meastopredictratio", train.vars.avgDistInches/train.vars.avgDistTest);
        SmartDashboard.putNumber("delta", train.vars.avgDistTest);
    }

    private static class InstanceHolder
    {
            private static final SupaStruct mInstance = new SupaStruct();
    }
}