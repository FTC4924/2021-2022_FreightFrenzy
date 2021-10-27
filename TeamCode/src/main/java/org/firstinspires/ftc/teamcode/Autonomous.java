package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.AllianceColor;

import java.util.ArrayList;
import java.util.Arrays;

public class Autonomous extends AutoBase{
    protected AllianceColor getAllianceColor() {
        return AllianceColor.BLUE;
    }
    protected ArrayList<Command> getCommands() {
        return new ArrayList<>(
                Arrays.asList(
                        new Command(Constants.CommandType.MOVE, AngleUnit.DEGREES, 5, 0.0, 0.8)
                )
        );
    }
}
