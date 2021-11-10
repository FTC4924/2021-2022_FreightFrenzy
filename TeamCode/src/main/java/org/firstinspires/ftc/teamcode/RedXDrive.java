package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.*;

@TeleOp(name="RedXDrive")
public class RedXDrive extends XDrive {
    protected AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }

}