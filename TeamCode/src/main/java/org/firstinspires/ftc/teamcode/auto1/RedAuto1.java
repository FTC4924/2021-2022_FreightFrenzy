package org.firstinspires.ftc.teamcode.auto1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants.*;

@Autonomous(name="RedAuto1")
public class RedAuto1 extends Auto1 {
    @Override
    protected AllianceColor getAllianceColor()  {
        return AllianceColor.RED;
    }
}
