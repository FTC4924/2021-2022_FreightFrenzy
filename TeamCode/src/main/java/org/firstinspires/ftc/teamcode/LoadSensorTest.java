package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import LoadSensorI2cDriver.NAU7802;

@TeleOp(name="LoadSensorTest")
public class LoadSensorTest extends OpMode {

    NAU7802 loadSensor;

    public void init() {
        loadSensor = hardwareMap.get(NAU7802.class, "loadSensor");
    }

    public void loop() {
        telemetry.addData("Available", loadSensor.available());
        telemetry.addData("Weight", loadSensor.getWeight());
        telemetry.addData("Reading", loadSensor.getReading());
    }

}