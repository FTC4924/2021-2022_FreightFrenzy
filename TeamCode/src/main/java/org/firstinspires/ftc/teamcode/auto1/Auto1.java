package org.firstinspires.ftc.teamcode.auto1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Command;
import org.firstinspires.ftc.teamcode.Constants.AllianceColor;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Constants.CommandType.*;


public abstract class Auto1 extends AutoBase {
    protected abstract AllianceColor getAllianceColor();
    protected ArrayList<Command> getCommands() {
        AllianceColor allianceColor = getAllianceColor();
        return new ArrayList<>(
                Arrays.asList(
                        //new Command(MOVE, AngleUnit.RADIANS, 1, -45.0, 0.2),
                        new Command(MOVE, AngleUnit.DEGREES, 6.0, 0, 0.5),
                        new Command(WAIT, 5)/*,
                        new Command(RED_BLUE,
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Command(TURN, AngleUnit.DEGREES, 180)
                                        )
                                ), new ArrayList<>(
                                        Arrays.asList(
                                                new Command(NONE)
                                        )
                                )
                        ),
                        new Command(MOVE, AngleUnit.DEGREES, 4.0, 90 * allianceColor.direction, 0.5),
                        new Command(DUCKS),
                        new Command(WAIT, 3),
                        new Command(DUCKS),
                        new Command(MOVE, AngleUnit.DEGREES, .8, 0, 0.5)*/
                )
        );
    }
}
