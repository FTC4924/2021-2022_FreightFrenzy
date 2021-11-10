package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.AllianceColor;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Constants.CommandType.DUCKS;
import static org.firstinspires.ftc.teamcode.Constants.CommandType.MOVE;
import static org.firstinspires.ftc.teamcode.Constants.CommandType.WAIT;

@Autonomous(name="RedAutonomous")
public class RedAutonomous extends AutoBase{
    protected AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }
    protected ArrayList<Command> getCommands() {
        return new ArrayList<>(
                Arrays.asList(
                        //new Command(MOVE, AngleUnit.RADIANS, 1, -45.0, 0.2),
                        new Command(MOVE, AngleUnit.DEGREES, 4.25, -85, 0.5),
                        new Command(DUCKS),
                        new Command(WAIT, 5),
                        new Command(DUCKS),
                        new Command(MOVE, AngleUnit.DEGREES, 1.34, 0, 0.5)
                )
        );
    }
}
