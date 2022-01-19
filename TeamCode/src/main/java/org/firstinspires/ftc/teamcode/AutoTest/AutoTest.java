package org.firstinspires.ftc.teamcode.AutoTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Constants.AllianceColor;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name="AutoTest")
public class AutoTest extends AutoBase {
    protected AllianceColor getAllianceColor()  {
        return AllianceColor.BLUE;
    }
    protected ArrayList<Command> getCommands() {
        return new ArrayList<Command>(
                Arrays.asList(
                        new Pause(100)
                )
        );
    }
}
