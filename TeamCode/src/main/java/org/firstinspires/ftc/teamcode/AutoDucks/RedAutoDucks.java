package org.firstinspires.ftc.teamcode.auto1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AutoDucks.AutoDucks;
import org.firstinspires.ftc.teamcode.Constants.*;

@Autonomous(name="RedAutoDucks")
public class RedAutoDucks extends AutoDucks {
    @Override
    protected AllianceColor getAllianceColor()  {
        return AllianceColor.RED;
    }
}
