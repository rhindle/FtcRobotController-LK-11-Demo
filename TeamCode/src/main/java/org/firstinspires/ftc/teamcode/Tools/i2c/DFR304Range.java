package org.firstinspires.ftc.teamcode.Tools.i2c;

import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import androidx.annotation.NonNull;

@SuppressWarnings("WeakerAccess")
@I2cDeviceType
@DeviceProperties(name = "DFR Range Sensor",
        description = "DFR Range Sensor",
        xmlTag = "DFRURM09RangeV2",
        compatibleControlSystems = ControlSystem.REV_HUB, builtIn = true)
public class DFR304Range extends I2cDeviceSynchDeviceWithParameters<I2cDeviceSynch, DFR304Range.Parameters>
{
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // User Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////
    public short getDistanceCm()
    {
        return readShort(Register.DIST_H_INDEX);
    }

    public short getTemperatureC() { return (short)(readShort(Register.TEMP_H_INDEX)/10); }
    public float getDistanceIn()
    {
        return (getDistanceCm() / 2.54f);
    }
    public short getTemperatureF()
    {
        return (short)(getTemperatureC() * 1.8 + 32);
    }
    // in passive mode must force a temperature read
    public void measureRange()
    {
        writeByte(Register.CMD_INDEX, CMD_DISTANCE_MEASURE);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Read and Write Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////
    protected void writeByte(final Register reg, Byte value)
    {
        byte[] txbuf =  {value};
        deviceClient.write(reg.bVal, txbuf);
    }

    protected byte readOneByte(Register reg)
    {
        byte[] raw = deviceClient.read(reg.bVal,1);
        return raw[0];
    }

    protected short readShort(Register reg)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Registers and Config Settings
    // CFG_INDEX
    // Bit7(control bit in ranging mode) 0: passive measurement, send ranging command once,
    // the module ranges the distance once and store the measured value into the distance register.
    // 1: automatic measurement mode, the module keeps ranging distance and updating the distance register all the time.
    // Bit6: save
    // Bit5-bit4(the maximum ranging distance bit that can be set)
    // 00:150CM(the ranging cycle is about 20MS)
    // 01:300CM(the ranging cycle is about 30MS)
    // 10:500CM(the ranging cycle is about 40MS)
    // Bits:   7654321
    // Mode  0bx000000
    // PASSIVE =  0 0x00  -- Must call measureRange() to measure range, then read it.
    // ACTIVE = 128 0x80 -- Continuously measures range, read any time.
    // ----Range----
    // 150CM 0b0000000 =  0 0x00
    // 300CM 0b0100000 = 16 0x10
    // 500CM 0b1000000 = 32 0z20
    ////////////////////////////////////////////////////////////////////////////////////////////////
    public enum Register
    {
        FIRST(0x00),
        PID_INDEX(0x01),
        VERSION_INDEX(0x02),
        DIST_H_INDEX(0x03),
        DIST_L_INDEX(0x04),
        TEMP_H_INDEX(0x05),
        TEMP_L_INDEX(0x06),
        CFG_INDEX(0x07),
        CMD_INDEX(0x08),
        LAST(CMD_INDEX.bVal);

        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }

    public enum MeasureMode
    {
        PASSIVE(0x00),
        ACTIVE(0x80);

        public int bVal;

        MeasureMode(int bVal)
        {
            this.bVal = bVal;
        }
    }

    public enum MaxRange
    {
        CM150(0x00),
        CM300(0x10),
        CM500(0x20);

        public int bVal;

        MaxRange(int bVal)
        {
            this.bVal = bVal;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Construction and Initialization
    ////////////////////////////////////////////////////////////////////////////////////////////////
    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x11);
    public final static byte CMD_DISTANCE_MEASURE = (0x01);

    public DFR304Range(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true, new Parameters());
        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged
        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
    }

    protected void setOptimalReadWindow()
    {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }


    @Override
    protected synchronized boolean internalInitialize(@NonNull Parameters params)
    {
        this.parameters = params.clone();
        deviceClient.setI2cAddress(params.i2cAddr);

        byte configSettings = (byte)(params.measureMode.bVal | params.maxRange.bVal);
        writeByte(Register.CFG_INDEX, configSettings);
        return readOneByte(Register.CFG_INDEX) == configSettings;
    }

    public static class Parameters implements Cloneable
    {
        I2cAddr i2cAddr = ADDRESS_I2C_DEFAULT;

        public MaxRange maxRange = MaxRange.CM150;
        public MeasureMode measureMode = MeasureMode.PASSIVE;

        public Parameters clone()
        {
            try
            {
                return (Parameters) super.clone();
            }
            catch(CloneNotSupportedException e)
            {
                throw new RuntimeException("Internal Error: Parameters not cloneable");
            }
        }
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName()
    {
        return "Gravity: URM09 Ultrasonic Sensor (I²C)";
    }
}

/*
https://wiki.dfrobot.com/URM09_Ultrasonic_Sensor_(Gravity-I2C)_(V1.0)_SKU_SEN0304

DFRobot URM09_Ultrasonic_Sensor_(Gravity-I2C)_(V1.0)_SKU_SEN0304

Introduction

   This is an ultrasonic distance sensor module with open dual probe. It adopts I2C communication and standard interface of Gravity
   PH2.0-4P vertical patch socket. The module is compatible with controllers with 3.3V or 5V logical level, such as Arduino and
   Raspberry Pi. The ultrasonic sensor comes with built-in temperature compensation, providing effective ranging within 2cm to 500cm.
   It offers resolution of 1cm and accuracy of ±1%. There are three measurement ranges designed for programs to select: 150cm, 300cm,
   500cm. Please note that setting shorter range will cause shorter ranging cycle and lower sensitivity. You may need to set it
   according to the actual use.

Specification

     * Supply Voltage: 3.3V～5.5V DC
     * Operating Current: 20mA
     * Operating Temperature Range: -10℃～＋70℃
     * Measurement Ranges: 2cm～500cm (can be set)
     * Resolution: 1cm
     * Precision: 1%
     * Direction Angle: 60°
     * Frequency: 50Hz Max
     * Dimension: 47mm × 22 mm/1.85”× 0.87”

Pinout

   Pinout

   NOTE: Compared with URM09 V1.0, the the latest URM09 V2.0 just improved the layout to improve its stability. And the dimension and
   function of V1.0 and V2.0 is the same.

   Pin    Description
   VCC    Power Supply(3.3V-5.5V)
   GND    Ground
   C      I2C SLC
   D      I2C SDA

Tutorial
   URM09 is a simple and practical ultrasonic sensor. It adopts I2C communication, which is very convenient to communicate with other
   boards that is equipped with I2C interface.

   URM09 Ultrasonic Sensor(Gravity I²C)(V1.0) Register

   Register Name                R/W  Data       Default  Description
   (8bit)                            Range      Value

   0x00     Device Address      R/W  0x08-0x77  0x11     I2C salve address, the default address is 0x11. If the address is
                                                         changed, the new address will be valid after repowering the module.

   0x01     Product ID          R               0x01     Used for product check

   0x02     Version Number      R               0x10     Used for Version check(0x10 means V1.0)

   0x03     Distance Value      R    0x00-0xFF  0x00     LSB represents 1CM.
            High-order bits                              e.g. 0x64 represents 100CM

   0x04     Distance Value      R    0x00-0xFF  0x00     LSB represents 1CM.
            Low-order bits                               e.g. 0x64 represents 100CM

   0x05     Temperature Value   R    0x00-0xFF  0x00     10 times amplified value based on the real temperature.
            High-order bits                              e.g. if the readout value is 0x00fe, the real temperature value
                                                         should be 0x00fe / 10 = 25.4℃

   0x06     Temperature Value   R    0x00-0xFF  0x00     10 times amplified value based on the real temperature.
            Low-order bits                               e.g. if the readout value is 0x00fe, the real temperature value
                                                         should be 0x00fe / 10 = 25.4℃

   0x07     Configure           R/W             0x00     Bit7(control bit in ranging mode)
            Registers                                    0: passive measurement, send ranging command once, the module
                                                         ranges the distance once and store the measured value into the
                                                         distance register.
                                                         1: automatic measurement mode, the module keeps ranging distance
                                                         and updating the distance register all the time. Bit6: save
                                                         Bit5-bit4(the maximum ranging distance bit that can be set)
                                                         00:150CM(the ranging cycle is about 20MS) 01:300CM(the ranging
                                                         cycle is about 30MS) 10:500CM(the ranging cycle is about 40MS)

   0x08     Command Registers   R/W             0x00     Writing 0X01 to this register under passive measurement mode and
                                                         the module ranges distance once. The write data is invalid under
                                                         automatic measurement mode.

 */