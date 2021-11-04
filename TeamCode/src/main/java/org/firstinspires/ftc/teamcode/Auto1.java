package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.AllianceColor;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Constants.CommandType.MOVE;

@Autonomous(name="Autonomous")
public class Auto1 extends AutoBase{
    protected AllianceColor getAllianceColor() {
        return AllianceColor.BLUE;
    }
    protected ArrayList<Command> getCommands() {
        return new ArrayList<>(
                Arrays.asList(
                        new Command(MOVE, AngleUnit.DEGREES, 5, 0.0, 0.2)
                )
        );
    }
}
