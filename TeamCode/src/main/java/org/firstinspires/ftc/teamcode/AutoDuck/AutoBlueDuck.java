package org.firstinspires.ftc.teamcode.AutoDuck;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants.*;

@Autonomous(name="BlueDuck")
public class AutoBlueDuck extends AutoDuck {
    @Override
    protected AllianceColor getAllianceColor()  {
        return AllianceColor.BLUE;
    }
}