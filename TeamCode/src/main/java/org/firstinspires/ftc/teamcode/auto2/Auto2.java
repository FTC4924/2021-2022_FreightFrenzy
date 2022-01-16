package org.firstinspires.ftc.teamcode.auto2;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.ArrayList;
import java.util.Arrays;

public abstract class Auto2 extends AutoBase {
    protected abstract Constants.AllianceColor getAllianceColor();
    protected ArrayList<Command> getCommands() {
        return new ArrayList<>(
                Arrays.asList(
                        new BlueRed(
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Move(.45, 90, .5)
                                        )
                                ),
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Move(.575, 90, .5)
                                        )
                                )
                        ),
                        new Pause(1),
                        new DetectDuckPosition(),
                        //new ArmRotate(.35),
                        //new ArmExtend(.75),
                        //new ArmFullRetract(),
                        new BlueRed(
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Move(1.95, -90, .5)
                                        )
                                ),
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Move(2.125, -90, .5)
                                        )
                                )
                        ),
                        /*new LoadDuckCommands(
                                new ArrayList<>(
                                        Arrays.asList(
                                                new ArmRotate(.5)
                                        )
                                ),
                                new ArrayList<>(
                                        Arrays.asList(
                                                new ArmRotate(.75)
                                        )
                                ),
                                new ArrayList<>(
                                        Arrays.asList(
                                                new ArmRotate(.95)
                                        )
                                )
                        ),*/
                        new Move(1.65,0,.5),
                        //new BristlesOut(),
                        new Pause(5),
                        //new BristlesOut(),
                        new Turn(90),
                        new Move(6,90,1)
                )
        );
    }
}

