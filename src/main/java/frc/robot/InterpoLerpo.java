// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class InterpoLerpo 
{
    public static InterpoLerpo getInstance()
    {
        return InstanceHolder.mInstance;
    }

    private InterpoLerpo()
    {

    }

    public double hoodInterpoLerpo(double inches)
    {
        return 748+(-12*inches)+(0.282*(Math.pow(inches,2)));
    }

    public double shooterInterpoLerpo(double inches)
    {
        return (10.5*inches) + 4668;
    }

    private static class InstanceHolder 
    {
        private static final InterpoLerpo mInstance = new InterpoLerpo();
    }
}
