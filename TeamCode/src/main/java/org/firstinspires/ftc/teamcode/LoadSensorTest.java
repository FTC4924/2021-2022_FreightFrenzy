package org.firstinspires.ftc.teamcode;

import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import LoadSensorI2cDriver.NAU7802;
import androidx.annotation.RequiresApi;

@TeleOp(name="LoadSensorTest")
public class LoadSensorTest extends OpMode {

    NAU7802 loadSensor;
    long reading;
    double weight;

    public void init() {
        loadSensor = hardwareMap.get(NAU7802.class, "loadSensor");
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public void loop() {
        telemetry.addData("Available", loadSensor.available());
        if(loadSensor.available()) {
            reading = loadSensor.getReading();
            weight = loadSensor.getWeight();
        }
        telemetry.addData("Weight", weight);
        telemetry.addData("Reading", reading);
    }
}