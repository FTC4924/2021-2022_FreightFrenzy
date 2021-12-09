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
                        new Command(RED_BLUE,
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Command(MOVE, AngleUnit.DEGREES, 0.45, 90, .5)
                                        )
                                ),
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Command(MOVE, AngleUnit.DEGREES, 0.5, -90, .5)
                                        )
                                )
                        ),
                        new Command(WAIT, 30)
                        /*new Command(MOVE, AngleUnit.RADIANS, 1, -45.0, 0.2),
                        new Command(RED_BLUE,
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Command(MOVE, AngleUnit.DEGREES, .3, 0, .5),
                                                new Command(TURN, AngleUnit.DEGREES, -90),
                                                new Command(MOVE, AngleUnit.DEGREES, 4.4,90,.5)
                                        )
                                ),
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Command(MOVE, AngleUnit.DEGREES, 4.75, -86, .5)
                                        )
                                )
                        ),
                        new Command(DUCKS),
                        new Command(WAIT, 6),
                        new Command(DUCKS),
                        new Command(MOVE, AngleUnit.DEGREES, 1.8, -10, 1)*/
                )
        );
    }
}
