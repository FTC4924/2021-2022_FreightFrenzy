package org.firstinspires.ftc.teamcode.AutoWarehouse;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants.*;

@Autonomous(name="RedWarehouse")
public class AutoRedWarehouse extends AutoWarehouse {
    @Override
    protected AllianceColor getAllianceColor()  {
        return AllianceColor.RED;
    }
}
