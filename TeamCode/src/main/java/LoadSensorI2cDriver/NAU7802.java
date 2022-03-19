package LoadSensorI2cDriver;

import android.os.Build;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddrConfig;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import androidx.annotation.RequiresApi;

import static LoadSensorI2cDriver.Constants.*;
import static LoadSensorI2cDriver.Constants.CTRL2_BITS.*;
import static LoadSensorI2cDriver.Constants.GAIN_VALUES.*;
import static LoadSensorI2cDriver.Constants.LDO_VALUES.*;
import static LoadSensorI2cDriver.Constants.PGA_PWR_Bits.*;
import static LoadSensorI2cDriver.Constants.PU_CTRL_Bits.*;
import static LoadSensorI2cDriver.Constants.Register.*;
import static LoadSensorI2cDriver.Constants.SPS_VALUES.*;

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
        boolean result = reset(); //Reset all registers

        result &= powerUp(); //Power on analog and digital sections of the scale

        result &= setLDO(LDO_3V3.value); //Set LDO to 3.3V

        result &= setGain(GAIN_128.value); //Set gain to 128

        result &= setSampleRate(SPS_80.value); //Set samples per second to 10

        result &= setRegister(ADC, (byte)0x30); //Turn off CLK_CHP. From 9.1 power on sequencing.

        result &= setBit(PGA_PWR, PGA_PWR_PGA_CAP_EN.index); //Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note.

        result &= calibrateAFE(); //Re-cal analog front end when we change gain, sample rate, or channel

        return (result);
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
     * @param register The first nybble of the command address used for indicating the register
     */
    private byte getRegister(Register register) {
        return this.deviceClient.read8(register.bVal);
    }

    private boolean setRegister(Register register, byte value) {
        this.deviceClient.write8(register.bVal, value);
        return true;
    }

    private byte[] burstRead(Register register, int amount) {
        return this.deviceClient.read(register.bVal, amount);
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public long getReading() {
        byte[] readings;
        readings = burstRead(ADCO_B2, 3);
        long reading = readings[0] << 16;
        reading |= (long) Byte.toUnsignedInt(readings[1]) << 8;
        return reading | Byte.toUnsignedInt(readings[2]);
    }

    public byte[] getRawReading() {
        return burstRead(ADCO_B2, 3);
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public int getAverageReading() {
        int total = 0;

        for(int i = 0; i < SAMPLE_SIZE; i++) {
            total += getReading();
        }

        return (int) Math.round((double) total / SAMPLE_SIZE);
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public double getWeight() {
        double averageReading = getAverageReading();

        return (averageReading - zeroOffset) / calibrationFactor;
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public void zero() {
        zeroOffset = getAverageReading();
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public void calibrate(double weight) {
        calibrationFactor = (getAverageReading() - zeroOffset) / weight;
    }

    public boolean setSampleRate(byte sampleRate) {
        if(sampleRate > DEFAULT_SAMPLE_RATE) sampleRate = DEFAULT_SAMPLE_RATE;
        byte settings = getRegister(CTRL2);
        settings &= 0b10001111;
        settings |= sampleRate;
        return setRegister(CTRL2, settings);
    }

    public boolean calibrateAFE() {
        setRegister(CTRL2, (byte) 0b100);

        for(int i = 0; i < 1000; i++) {
            if(!getBit(CTRL2, CTRL2_CALS.index)) {
                return !getBit(CTRL2, CTRL2_CAL_ERROR.index);
            }
            try { Thread.sleep(1); }
            catch (InterruptedException e) { System.out.println(e); }
        }
        return false;
    }

    public boolean available() {
        return getBit(PU_CTRL, PU_CTRL_CR.index);
    }

    private boolean getBit(Register register, byte index) {
        byte reading = getRegister(register);
        reading &= 1 << index;
        return reading != 0;
    }

    public boolean powerUp() {
        setBit(PU_CTRL, PU_CTRL_PUD.index);
        setBit(PU_CTRL, PU_CTRL_PUA.index);
        for (int i = 0; i < 100; i++){
            if (getBit(PU_CTRL, PU_CTRL_PUR.index)) return true;
            try { Thread.sleep(1); }
            catch (InterruptedException e) { System.out.println(e); }
        }
        return false;
    }

    public boolean setLDO(byte ldoValue)
    {
        if (ldoValue > 0b111) ldoValue = 0b111; //Error check

        //Set the value of the LDO
        byte value = getRegister(CTRL1);
        value &= 0b11000111;    //Clear LDO bits
        value |= ldoValue << 3; //Mask in new LDO bits
        setRegister(CTRL1, value);

        return (setBit(PU_CTRL, PU_CTRL_AVDDS.index)); //Enable the internal LDO
    }

    public boolean setGain(byte gainValue)
    {
        if (gainValue > 0b111) gainValue = 0b111; //Error check

        byte value = getRegister(CTRL1);
        value &= 0b11111000; //Clear gain bits
        value |= gainValue;  //Mask in new bits

        return (setRegister(CTRL1, value));
    }

    private boolean reset() {
        setBit(PU_CTRL, PU_CTRL_RR.index); //Set RR
        try { Thread.sleep(1); }
        catch (InterruptedException e) { System.out.println(e); }
        return clearBit(PU_CTRL, PU_CTRL_RR.index); //Clear RR to leave reset state
    }

    private boolean setBit(Register register, byte index) {
        byte registerBuffer = getRegister(register);
        registerBuffer |= 1 << index;
        return setRegister(register, registerBuffer);
    }

    private boolean clearBit(Register register, byte index) {
        byte registerBuffer = getRegister(register);
        registerBuffer &= ~(1 << index);
        return setRegister(register, registerBuffer);
    }
}