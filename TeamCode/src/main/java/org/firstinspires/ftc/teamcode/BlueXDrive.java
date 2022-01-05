package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.*;

@TeleOp(name="BlueXDrive")
public class BlueXDrive extends XDrive {
    protected AllianceColor getAllianceColor() {
        return AllianceColor.BLUE;
    }

}
