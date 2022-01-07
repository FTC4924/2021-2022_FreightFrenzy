package org.firstinspires.ftc.teamcode.auto1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Constants.*;

@Autonomous(name="RedAuto")
public class RedAuto extends Auto {
    @Override
    protected AllianceColor getAllianceColor()  {
        return AllianceColor.RED;
    }
}
