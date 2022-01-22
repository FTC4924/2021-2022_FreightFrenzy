package org.firstinspires.ftc.teamcode.AutoWarehouse;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.ArrayList;
import java.util.Arrays;

public abstract class AutoWarehouse extends AutoBase {
    protected abstract Constants.AllianceColor getAllianceColor();
    protected ArrayList<Command> getCommands() {
        return new ArrayList<>(
                Arrays.asList(
                        new BlueRed(
                                new ArrayList<>(Arrays.asList(
                                        new Move(.45, 90, .5)
                                )),
                                new ArrayList<>(Arrays.asList(
                                        new Move(.625, 90, .5)
                                ))
                        ),//Moves so the camera can see both barcode positions
                        new Pause(1),
                        new DetectBarcodePosition(),
                        new ArmRotate(.35),//Deploys arm
                        new ArmExtend(.75),
                        new ArmFullRetract(),
                        new BlueRed(//lines up with the alliance hub
                                new ArrayList<>(Arrays.asList(
                                        new Move(1.95, -90, .5)
                                )),
                                new ArrayList<>(Arrays.asList(
                                        new Move(2.125, -90, .5)
                                ))
                        ),
                        new LoadBarcodeCommands(//Raises the arm to respective level
                                new ArrayList<>(Arrays.asList(
                                        new ArmRotate(.5)
                                )),
                                new ArrayList<>(Arrays.asList(
                                        new ArmRotate(.75)
                                )),
                                new ArrayList<>(Arrays.asList(
                                        new ArmRotate(.95)
                                ))
                        ),
                        new Move(1.65,0,.5),//Moves toward the alliance hub
                        new BristlesOut(),//spits the block out
                        new Pause(5),
                        new BristlesOut(),
                        new Turn(90),//parks in the warehouse
                        new Move(6,90,1)
                )
        );
    }
}

