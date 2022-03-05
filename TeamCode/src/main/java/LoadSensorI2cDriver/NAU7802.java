package LoadSensorI2cDriver;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddrConfig;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import static LoadSensorI2cDriver.Constants.*;

@I2cDeviceType
@DeviceProperties(name = "NAU7802 Strain Gauge", xmlTag = "NAU7802")
public class NAU7802 extends I2cDeviceSynchDevice<I2cDeviceSynch> implements I2cAddrConfig {

    int zeroOffset = DEFAULT_ZERO_OFFSET;
    double calibrationFactor = DEFAULT_CALIBRATION_FACTOR;

    /**
     * Returns an indication of the manufacturer of this device.
     * @return the device's manufacturer
     */
    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Other;
    }

    /**
     * Returns a string suitable for display to the user as to the type of device.
     * @return device name
     */
    @Override
    public String getDeviceName()
    {
        return "NAU7802 Load Sensor";
    }

    public NAU7802(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        setSampleRate(DEFAULT_SAMPLE_RATE);
        calibrateAFE();

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    /**
     * Initializes the backpack for the 8x8 matrix: turning on the system oscillator,
     * setting everything to default, clearing the display, and turning the display on.
     * @return Whether the initialization was successful or not
     */
    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }

    /**
     * Configures a new I2C address to use.
     * @param newAddress the new I2C address to use
     */
    @Override
    public void setI2cAddress(I2cAddr newAddress) {
        this.deviceClient.setI2cAddress(newAddress);
    }

    /**
     * Returns the I2C address currently in use to communicate with an I2C hardware device.
     * @return the I2C address currently in use
     */
    @Override
    public I2cAddr getI2cAddress() {
        return this.deviceClient.getI2cAddress();
    }

    /**
     * Writes a byte to the indicated register.
     * @param command The first nybble of the command address used for indicating the register
     */
    private byte read8(Command command) {
        return this.deviceClient.read8(command.bVal);
    }

    private void write8(Command command, byte value) {
        this.deviceClient.write8(command.bVal, value);
    }

    public byte getReading() {
        return read8(Command.READ);
    }

    public int getAverageReading() {
        float total = 0;

        for(int i = 0; i < SAMPLE_SIZE; i++) {
            total += getReading();
        }

        return Math.round(total / SAMPLE_SIZE);
    }

    public double getWeight() {
        double averageReading = getAverageReading();
        return (averageReading - zeroOffset) / calibrationFactor;
    }

    public void zero() {
        zeroOffset = getAverageReading();
    }

    public void calibrate(double weight) {
        calibrationFactor = (getAverageReading() - zeroOffset) / weight;
    }

    public void setSampleRate(byte sampleRate) {
        if(sampleRate > DEFAULT_SAMPLE_RATE) sampleRate = DEFAULT_SAMPLE_RATE;
        byte settings = read8(Command.CTRL2);
        settings &= 0b10001111;
        settings |= sampleRate;
        write8(Command.CTRL2, settings);
    }

    public void calibrateAFE() {
        write8(Command.CTRL2, (byte) 0b100);
    }

    public byte available() {
        return read8(Command.PU_CTRL);
    }
}