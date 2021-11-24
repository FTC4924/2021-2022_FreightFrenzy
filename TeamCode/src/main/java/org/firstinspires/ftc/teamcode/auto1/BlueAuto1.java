package org.firstinspires.ftc.teamcode.auto1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants.*;

@Autonomous(name="BlueAuto1")
public class BlueAuto1 extends Auto1 {
    @Override
    protected AllianceColor getAllianceColor()  {
        return AllianceColor.BLUE;
    }
}