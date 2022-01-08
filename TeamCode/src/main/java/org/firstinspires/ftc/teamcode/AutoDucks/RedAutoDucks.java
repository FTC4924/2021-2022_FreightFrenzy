package org.firstinspires.ftc.teamcode.AutoDucks;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Constants.*;

@Disabled
@Autonomous(name="RedAutoDucks")
public class RedAutoDucks extends AutoLED {
    @Override
    protected AllianceColor getAllianceColor()  {
        return AllianceColor.RED;
    }
}
