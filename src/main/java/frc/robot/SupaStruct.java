// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SupaStruct {
    
    private XboxController xbox = new XboxController(0);
    private double fwd, fwdSignum, str, strSignum, rcw, rcwX, rcwY, inverseTanAngleOG, inverseTanAngleDrive, povValue = 0;
    private MkSwerveTrain train = MkSwerveTrain.getInstance();
    private boolean abutton, bbutton, xbutton, ybutton, pov, povToggled, itsreal = false;
   
   
    public static SupaStruct getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void updateTele()
    {
        train.updateSwerve();
        
        fwd = (xbox.getRawAxis(1) - 0.1) / (1 - 0.1);
        fwdSignum = Math.signum(fwd) * -1;
        str = (xbox.getRawAxis(0) - 0.1) / (1 - 0.1);
        strSignum = Math.signum(str) * -1;
        rcw = (xbox.getRawAxis(5) - 0.1) / (1 - 0.1);
        
        rcwY = rcw;
        rcwX =  (xbox.getRawAxis(4) - 0.1) / (1 - 0.1);
        abutton = xbox.getAButton();
        bbutton = xbox.getBButton();
        xbutton = xbox.getXButton();
        ybutton = xbox.getYButton();
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






       

        if(abutton)
        {
            navx.getInstance().reset();
            povValue = 0;
            inverseTanAngleOG = 0;
        }
        if(bbutton)
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
        if(Math.abs(xbox.getRawAxis(5)) >= 0.1 || Math.abs(xbox.getRawAxis(4)) >= 0.1)
        {
            //rcw = train.moveToAngy((inverseTanAngleOG + 270) % 360);
            rcw = rcwX/5;
            povToggled = false;
        }
        else if(povToggled)
        {
            rcw = train.moveToAngy((povValue+180)% 360);
        }
        
        itsreal = false;

//      else statements
        if(/*!ybutton&&*/ !povToggled && /*!bbutton&&*/ Math.abs(xbox.getRawAxis(5)) < 0.1 && Math.abs(xbox.getRawAxis(4)) < 0.1)
        {
            rcw = 0;
            itsreal = true;
        }
        if(Math.abs(xbox.getRawAxis(5)) < 0.1)
        {
            rcwY = 0;
        }
        if(Math.abs(xbox.getRawAxis(4)) < 0.1)
        {
            rcwX = 0;
        }
        if(Math.abs(xbox.getRawAxis(1)) < 0.1)
        {
            fwd = 0;
        }
        if(Math.abs(xbox.getRawAxis(0)) < 0.1)
        {
            str = 0;
        }



//      applying numbers
        if(fwd != 0 || str != 0 || rcw != 0)
        {//+,-,+
            train.etherSwerve(fwd/3, -str/3, rcw, ControlMode.PercentOutput); //+,-,+
        }
        else
        {
            train.stopEverything();
        }
        
        
        SmartDashboard.putNumber("doesthiswork", inverseTanAngleOG);
        SmartDashboard.putNumber("rcwrobotperiod", rcw);
        SmartDashboard.putBoolean("pov", pov);
        SmartDashboard.putBoolean("povtoggled", povToggled);
        SmartDashboard.putNumber("povvalue", povValue);
        SmartDashboard.putNumber("inverseTanAngleOG with the 90", (inverseTanAngleOG + 270) % 360);
        SmartDashboard.putNumber("inverse tan angle drive", inverseTanAngleDrive);
        SmartDashboard.putNumber("rcwy", rcwY);
        SmartDashboard.putNumber("rcwx", rcwX);
        SmartDashboard.putBoolean("itsreal", itsreal);
        SmartDashboard.putNumber("fwd", fwd);
        SmartDashboard.putNumber("str", str);
    }

    public void teleopDisabled()
    {
        abutton = false;
        bbutton = false;
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