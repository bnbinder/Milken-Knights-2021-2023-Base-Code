// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.Drake;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CONTROLLERS.DriveInput;

/** Add your docs here. */
public class SupaStruct {
    
    private XboxController xbox = new XboxController(0);
    private XboxController xboxOP = new XboxController(1);
    private double fwd, fwdSignum, str, strSignum, leftjoy, rcw, rcwX, rcwY, inverseTanAngleOG, inverseTanAngleDrive, povValue = 0;
    private MkSwerveTrain train = MkSwerveTrain.getInstance();
    private Shooter shoot = Shooter.getInstance();
    private Intake intake = Intake.getInstance();
    private Climber Climb = Climber.getInstance();
    private Elevator elevator = Elevator.getInstance();
    private ColorSensor color = ColorSensor.getInstance();
    private boolean resetNavx, resetDrive, xbutton, ybutton,rbbutton,lbbutton,abutton, rtrigger, pov, povToggled, itsreal = false;
    private Climber mClimb = Climber.getInstance();
   
    public static SupaStruct getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void updateTele()
    {
        //TODO seperate all smartdashboard update from main update in update function in all classes
        train.updateSwerve();
        color.updateColor();
        //ultra.updateUltra();
        mClimb.climberUpdate();
        //ultra.ultraSmartDashboard();
        color.colorSmartDashboard();
     //   shoot.updateShooter();

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
        xbutton = xboxOP.getXButtonPressed();
        abutton = xboxOP.getAButtonPressed();
        rbbutton = xboxOP.getRightBumper();
        lbbutton = xboxOP.getLeftBumper();
        rtrigger = Math.abs(xboxOP.getRawAxis(2)) > 0;
        leftjoy = Math.abs(xboxOP.getRawAxis(1));
        ybutton = xboxOP.getYButton();
        pov = xbox.getPOV() != -1;

//      i dont remember how i got this lol
        inverseTanAngleOG = ((((((Math.toDegrees(Math.atan(rcwY/rcwX))+360 )) + 
                            (MathFormulas.signumV4(rcwX)))%360) - 
                            MathFormulas.signumAngleEdition(rcwX,rcwY))+360)
                            %360;

     inverseTanAngleDrive = ((((((Math.toDegrees(Math.atan(fwd/str))+360 )) + 
                            (MathFormulas.signumV4(str)))%360) - 
                            MathFormulas.signumAngleEdition(str,fwd))+360)
                            %360;
        if(resetNavx)
        {
            navx.getInstance().reset();
            povValue = 0;
            inverseTanAngleOG = 0;
        }
        if(resetDrive)
        {
            MkSwerveTrain.getInstance().vars.avgDistInches = 0;
            MkSwerveTrain.getInstance().startDrive();
            //str = Math.cos(inverseTanAngleDrive* (Constants.kPi/180));
            //fwd = Math.sin(inverseTanAngleDrive* (Constants.kPi/180));
        }
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
            rcw = rcwX/5;
            povToggled = false;
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
        //  SD SWERVE OUTPUT
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
        //--------------------------------------------------------------------//
        //  ROLLER CONTROL
        //--------------------------------------------------------------------//
        
        if(rbbutton)
        {
            intake.rollerSet(-.5);
        }
        else if(lbbutton)
        {
            intake.rollerSet(.5);
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
        elevator.setElevator(ControlMode.PercentOutput,leftjoy);
        elevator.setShitter(ControlMode.PercentOutput,leftjoy);


        //--------------------------------------------------------------------//
        //  CLIMBER CONTROL
        //--------------------------------------------------------------------//

        //--------------------------------------------------------------------//
        //  SHOOTER CONTROL
        //--------------------------------------------------------------------//


//     applying numbers
        if(fwd != 0 || str != 0 || rcw != 0)
        {//+,-,+
            train.etherSwerve(fwd/2, -str/2, rcw, ControlMode.PercentOutput); //+,-,+
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
        povToggled = false;
        itsreal = false;
    }

    private static class InstanceHolder
    {
            private static final SupaStruct mInstance = new SupaStruct();
    } 
}