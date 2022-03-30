package org.firstinspires.ftc.teamcode.AutoDuck;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Constants.AllianceColor;
import org.firstinspires.ftc.teamcode.Commands.*;

import java.util.ArrayList;
import java.util.Arrays;


public abstract class AutoDuck extends AutoBase {
    protected abstract AllianceColor getAllianceColor();
    protected ArrayList<Command> getCommands() {
        return new ArrayList<>(
                Arrays.asList(
                        //new ArmFullRetract(),
                        new BlueRed(// Moves so the camera can see both barcode positions
                                new ArrayList<>(Arrays.asList(
                                        new Move(.625, -90, .5)
                                )),
                                new ArrayList<>(Arrays.asList(
                                        new Move(.45, -90, .5)
                                ))
                        ),
                        new Pause(1),
                        new DetectBarcodePosition(),
                        new Move(.5, -45, 0.5),// Moves away from the wall
                        new BlueRed(
                                new ArrayList<>(Arrays.asList(
                                        new Move(.2, -45, 0.5),
                                        new Move(1.5, -90, .5),// Strafes to the carousel
                                        new Move(0.25,-90,3),
                                        new Move(.1, 90, .25)
                                )),
                                new ArrayList<>(Arrays.asList(
                                        new Turn(90), // Turns so the duck wheel faces the carousel
                                        new Move(.85, -45, 0.5),
                                        new Move(1.55, -105, .5), // Strafes to the carousel
                                        new Move(0.25,-180,3)
                                ))
                        ),
                        new Ducks(),// spins the duck wheel
                        new Pause(5),
                        new Ducks(),
                        new BlueRed(
                                new ArrayList<>(Arrays.asList()),
                                new ArrayList<>(Arrays.asList(
                                        new Turn(0)
                                ))
                        ),// Turns back to straight
                        new ArmExtend(.9),
                        new ArmExtend(.3),
                        new LoadBarcodeCommands(// Raises the arm to the respective level
                                new ArrayList<>(Arrays.asList(
                                        new ArmRotate(.25)
                                )),
                                new ArrayList<>(Arrays.asList(
                                        new ArmRotate(.625)
                                )),
                                new ArrayList<>(Arrays.asList(
                                        new ArmRotate(1)
                                ))
                        ),// Loads the commands from detecting the barcode positions
                        new Move(4.25,90,.5),// Lines up with the alliance hub
                        new BlueRed(
                                new ArrayList<>(Arrays.asList(
                                        new Move(1,0, .5)// Moves towards the alliance hub
                                )),
                                new ArrayList<>(Arrays.asList(
                                        new Move(.75, 0, .5)
                                ))
                        ),
                        new BristlesOut(),// spits the block out
                        new Pause(3),
                        new BristlesOut(),
                        new Move(.5, 180,1.0),
                        new Turn(90),// Partially parks in the warehouse
                        new BlueRed(
                                new ArrayList<>(Arrays.asList(
                                        new Move(2,90, 1.0)// Moves towards the alliance hub
                                )),
                                new ArrayList<>(Arrays.asList(
                                        new Move(3,90,1.0)
                                ))
                        )/*,
                        //new ArmExtend(0),
                        new ArmRotate(0)*/
                )
        );
    }
}
