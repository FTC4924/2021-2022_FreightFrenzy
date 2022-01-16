package org.firstinspires.ftc.teamcode.auto2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Constants.*;

@Autonomous(name="BlueAuto2")
public class BlueAuto2 extends Auto2 {
    @Override
    protected AllianceColor getAllianceColor()  {
        return AllianceColor.BLUE;
    }
}
