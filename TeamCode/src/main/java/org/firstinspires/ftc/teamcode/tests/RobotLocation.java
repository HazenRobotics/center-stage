package org.firstinspires.ftc.teamcode.tests;

public class RobotLocation
{
    double x = 0;
    public double getX()
    {
        return x;
    }

    public void changeX(double change)
    {
        x += change;
    }

    public void setX(double x)
    {
        this.x = x;
    }
}
