package org.firstinspires.ftc.teamcode.Tools.i2c;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddrConfig;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.RobotLog;

import java.lang.reflect.Field;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;

import androidx.annotation.ColorInt;
import androidx.annotation.NonNull;

import static android.os.SystemClock.sleep;

// This code adapted from https://github.com/w8wjb/ftc-neodriver

@I2cDeviceType
@DeviceProperties(name = "Adafruit NeoDriver", xmlTag = "AdafruitNeoDriver", description = "an Adafruit NeoDriver board", builtIn = false)
public class AdafruitNeoDriver extends I2cDeviceSynchDeviceWithParameters<I2cDeviceSynch, AdafruitNeoDriver.Parameters>
        implements I2cAddrConfig, OpModeManagerNotifier.Notifications {

    public static final I2cAddr I2CADDR_DEFAULT = I2cAddr.create7bit(0x60);

    private static final byte SEESAW_BASE_REGISTER_ADDR = 0x00;
    private static final byte NEO_BASE_REGISTER_ADDR = 0x0E;

    /* Number of the Neopixel Pin - This is set in the device hardware, but for some reason needs to be specified */
    private static final byte NEOPIXEL_PIN = 15;

    /* Maximum number of bytes that can be sent in one I2C frame */
    /* LK: I was wondering why not 30? The NeoDriver board supports 32 (2 address, 30 data). [https://learn.adafruit.com/adafruit-seesaw-atsamd09-breakout/neopixel]
        However, I now see in I2cDeviceSynch that WRITE_REGISTER_COUNT_MAX = 26. I suppose that is why we have 24 here. */
    private static final int MAX_TX_BYTES = 24;

    private static final String TAG = "NeoDriver";

    private boolean didInit = false;

    private enum FunctionRegister {

        STATUS(0x00),
        PIN(0x01),
        SPEED(0x02),
        BUF_LENGTH(0x03),
        BUF(0x04),
        SHOW(0x05);

        public final byte bVal;

        FunctionRegister(int i) {
            this.bVal = (byte) i;
        }
    }

    public AdafruitNeoDriver(I2cDeviceSynch deviceClient) {
        this(deviceClient, true, new Parameters());
    }

    public AdafruitNeoDriver(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        this(deviceClient, deviceClientIsOwned, new Parameters());
    }

    protected AdafruitNeoDriver(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned, @NonNull Parameters defaultParameters) {
        super(i2cDeviceSynch, deviceClientIsOwned, defaultParameters);

        this.deviceClient.setI2cAddress(I2CADDR_DEFAULT);
        this.deviceClient.setLogging(true);
        this.deviceClient.setLoggingTag("NeoDriverI2C");

        // We ask for an initial call back here; that will eventually call internalInitialize()
        this.registerArmingStateCallback(true);

        this.deviceClient.engage();
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName() {
        return "NeoDriver";
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        RobotLog.vv(TAG, "resetDeviceConfigurationForOpMode()...");
        if (didInit) {
            setBusSpeed400();
            sendSeesawReset();
            sleep(250);
            setNeopixelSpeed();  // LK: The first write after the reset usually fails if the board was disconnected
            sleep(250);
            setNeopixelSpeed();
            setNeopixelPin();
        }
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {
        RobotLog.vv(TAG, "onOpModePreInit()...");
        // no-op
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {
        RobotLog.vv(TAG, "onOpModePreStart()...");
        // no-op
    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        RobotLog.vv(TAG, "onOpModePostStop()...");
        /* Turn all the lights off when the OpMode is stopped */
        //fill(0);  //LK return this to 0; was useful during debugging
        fill(Color.rgb(0,0,0));
        show();
    }

    @Override
    protected boolean internalInitialize(@NonNull Parameters parameters) { /* ### */

        RobotLog.vv(TAG, "internalInitialize()...");

        /* Make sure we're talking to the correct I2c address */
        this.deviceClient.setI2cAddress(parameters.i2cAddr);

        setBusSpeed400();

        /* Can't do anything if we're not really talking to the hardware */
        if (!this.deviceClient.isArmed()) {
            Log.d(TAG, "not armed");
            return false;
        }

        /* LK: I can't get this to work without a delay here, even setting the bus speed to 400 kHz
            I2C Nack, then nothing works with errors in the log every transaction.
            But the red light on the board is blinking, indicating it's doing something?
            Is time needed for some kind of i2c handshake?
            The adafruit board defaults to 800 kHz, but Rev hub supports 100/400 kHz */
        sleep(200);

        /* LK  tried writing the speed here to see if it helps, but same difference */
        setNeopixelSpeed();
        setNeopixelPin();

        didInit = true;
        return true;
    }

    private void sendSeesawReset() {
        byte[] bytes = new byte[]{SEESAW_BASE_REGISTER_ADDR, 0x7F, (byte) 0xFF};
        RobotLog.vv(TAG, "Resetting Seesaw " + toHex(bytes));
        this.deviceClient.write(NEO_BASE_REGISTER_ADDR, bytes, I2cWaitControl.ATOMIC);
    }

    private void setBusSpeed400(){
        /* Attempt to set the I2C speed for this channel to 400 kHz */
        //LynxI2cDeviceSynch device1 = null;
        try {
            Field field = getField(this.getClass(), "deviceClient");
            field.setAccessible(true);
            I2cDeviceSynchImplOnSimple simple = (I2cDeviceSynchImplOnSimple) field.get(this);

            field = getField(simple.getClass(), "i2cDeviceSynchSimple");
            field.setAccessible(true);
            LynxI2cDeviceSynch device1 = (LynxI2cDeviceSynch) field.get(simple);

            device1.setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
            RobotLog.vv(TAG, "Tried to set bus speed to 400 kHz @ "+device1.getDeviceName()+" > "+device1.getUserConfiguredName()+" > "+device1.getConnectionInfo());
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
    }

    @Override
    public I2cAddr getI2cAddress() {
        return this.parameters.i2cAddr;
    }

    @Override
    public void setI2cAddress(I2cAddr newAddress) {
        this.parameters.i2cAddr = newAddress;
        this.deviceClient.setI2cAddress(newAddress);
    }

    // LK Not sure why this doesn't initialize with parameters (not currently public?), but I don't want to break anything
    public void setColorOrder (ColorOrder colorOrder) {
        this.parameters.colorOrder = colorOrder;
    }

    /**
     * Sets the number of pixels that are available in the strand
     * @param numPixels
     */
    //@Override
    public void setNumberOfPixels(int numPixels) {

        parameters.numPixels = numPixels;

        int bufferLength = parameters.numPixels * parameters.bytesPerPixel;

        //ByteBuffer buffer = ByteBuffer.allocate(3).order(ByteOrder.LITTLE_ENDIAN);
        ByteBuffer buffer = ByteBuffer.allocate(3).order(ByteOrder.BIG_ENDIAN);  // LK Documentation is wrong: not little endian
        buffer.put(FunctionRegister.BUF_LENGTH.bVal);
        buffer.putShort((short) bufferLength);

        byte[] bytes = buffer.array();
        RobotLog.vv(TAG, "Writing NEOPIXEL_BUF_LENGTH " + toHex(bytes));
        deviceClient.write(NEO_BASE_REGISTER_ADDR, bytes, I2cWaitControl.WRITTEN);
    }

    public void setNeopixelPin() {
        byte[] bytes = new byte[]{FunctionRegister.PIN.bVal, NEOPIXEL_PIN};
        RobotLog.vv(TAG, "Writing NEOPIXEL_PIN " + toHex(bytes));
        this.deviceClient.write(NEO_BASE_REGISTER_ADDR, bytes, I2cWaitControl.WRITTEN);
    }

    public void setNeopixelSpeed() {
        byte[] byte0 = new byte[]{FunctionRegister.SPEED.bVal, 0};
        RobotLog.vv(TAG, "Writing NEOPIXEL_SPEED " + toHex(byte0) + " (400 MHz)");
        deviceClient.write(NEO_BASE_REGISTER_ADDR, byte0, I2cWaitControl.WRITTEN);
    }

    /**
     * Sets a specific pixel in a strand to a color
     * @param index Index of pixel, 0-based
     * @param colorString Hex color string, i.e. #RRGGBB
     */
    //@Override
    public void setPixelColor(int index, String colorString) {
        setPixelColor(index, Color.parseColor(colorString));
    }

    /**
     * Sets a specific pixel in a strand to a color
     * @param index Index of pixel, 0-based
     * @param color Color encoded as an int. See {@link  Color} for useful color utilities
     */
    //@Override
    public void setPixelColor(int index, int color) {

        if (index > parameters.numPixels) {
            throw new ArrayIndexOutOfBoundsException("Index " + index + " is out of bounds of the pixel array");
        }

        int bufferSize = 3 + parameters.bytesPerPixel;

        ByteBuffer buffer = ByteBuffer.allocate(bufferSize).order(ByteOrder.BIG_ENDIAN);
        buffer.put(FunctionRegister.BUF.bVal);
        buffer.putShort((short) (index * parameters.bytesPerPixel));
        buffer.put(colorsToBytes(parameters.bytesPerPixel, parameters.colorOrder, color));

        byte[] bytes = buffer.array();
        deviceClient.write(NEO_BASE_REGISTER_ADDR, bytes);
    }

    /**
     * Sets a sequence of pixels to colors, starting at index 0
     * @param colorStrings 1 or more hex color strings, i.e. #RRGGBB
     */
    //@Override
    public void setPixelColors(String... colorStrings) {
        int[] colors = new int[colorStrings.length];
        for (int i = 0; i < colorStrings.length; i++) {
            colors[i] = Color.parseColor(colorStrings[i]);
        }
        setPixelColors(colors);
    }

    /**
     * Sets a sequence of pixels to colors, starting at index 0
     * @param colors Colors encoded as an int array. See {@link  Color} for useful color utilities
     */
    //@Override
    public void setPixelColors(@ColorInt int[] colors) {

        if (colors.length > parameters.numPixels) {
            throw new ArrayIndexOutOfBoundsException("Incoming color array is larger than the pixel array");
        }

        byte[] colorData = colorsToBytes(parameters.bytesPerPixel, parameters.colorOrder, colors);

        for (int chunkStart = 0; chunkStart < colorData.length; chunkStart += MAX_TX_BYTES) {
            int chunkLength = Math.min(MAX_TX_BYTES, colorData.length - chunkStart);
            sendPixelData((short) chunkStart, colorData, chunkStart, chunkLength);
        }
    }

    // LK new method for single i2c transaction incremental updates, standalone
    public void transmitPixelUpdateA(@ColorInt int[] colors, int startPos) {
        if (colors.length != 8) return;  // MAX_TX_BYTES/3 = 8
        //^^ temp  if (startPos < 0 || startPos > parameters.numPixels - 8 ) return;

        byte[] colorData = colorsToBytes(parameters.bytesPerPixel, parameters.colorOrder, colors);

        int bufferSize = 3 + 24;

        ByteBuffer buffer = ByteBuffer.allocate(bufferSize).order(ByteOrder.BIG_ENDIAN);
        buffer.put(FunctionRegister.BUF.bVal);
        buffer.putShort((short)(startPos*3));
        //buffer.put(colorData, 0, 24);
        buffer.put(colorData);

        byte[] bytes = buffer.array();
        deviceClient.write(NEO_BASE_REGISTER_ADDR, bytes);
    }

    // LK new method for single i2c transaction incremental updates, using existing method sendPixelData
    public void transmitPixelUpdateB(@ColorInt int[] colors, int startPos) {
        if (colors.length != 8) return;  // MAX_TX_BYTES/3 = 8
        if (startPos < 0 || startPos > parameters.numPixels - 8 ) return;
        byte[] colorData = colorsToBytes(parameters.bytesPerPixel, parameters.colorOrder, colors);
        sendPixelData((short)(startPos*3), colorData, 0, 24);
    }

    private void sendPixelData(short memOffset, byte[] colorData, int offset, int length) {
        int bufferSize = 3 + length;

        ByteBuffer buffer = ByteBuffer.allocate(bufferSize).order(ByteOrder.BIG_ENDIAN);
        buffer.put(FunctionRegister.BUF.bVal);
        buffer.putShort(memOffset);
        buffer.put(colorData, offset, length);

        byte[] bytes = buffer.array();
        deviceClient.write(NEO_BASE_REGISTER_ADDR, bytes);
    }

    //@Override
    public void fill(int color) {
        int[] colors = new int[parameters.numPixels];
        Arrays.fill(colors, color);
        setPixelColors(colors);
    }

    //@Override
    public void fill(String colorString) {
        fill(Color.parseColor(colorString));
    }

    /**
     * This must be called to signal that the color data should be sent to the NeoPixel strand
     */
    //@Override
    public void show() {
        byte[] bytes = new byte[]{FunctionRegister.SHOW.bVal};
        deviceClient.write(NEO_BASE_REGISTER_ADDR, bytes, I2cWaitControl.WRITTEN);
    }

    private static byte[] colorsToBytes(int bytesPerPixel, ColorOrder order, int... colors) {

        byte[] colorData = new byte[colors.length * bytesPerPixel];

        for (int colorIndex = 0; colorIndex < colors.length; colorIndex++) {
            int color = colors[colorIndex];
            int dataIndex = colorIndex * bytesPerPixel;

            colorData[dataIndex + order.redIndex] = (byte) (color >> 16);
            colorData[dataIndex + order.greenIndex] = (byte) (color >> 8);
            colorData[dataIndex + order.blueIndex] = (byte) color;
            if (bytesPerPixel == 4) {
                colorData[dataIndex + 3] = (byte) (color >> 24);
            }
        }

        return colorData;
    }

    private String toHex(byte[] bytes) {
        StringBuilder builder = new StringBuilder(bytes.length * 3);

        for (byte b : bytes) {
            builder.append(String.format(" %02X", b));
        }

        return builder.toString();
    }

    public enum ColorOrder {
        RGB(0, 1, 2),
        RBG(0, 2, 1),
        GRB(1, 0, 2),
        GBR(2, 0, 1),
        BRG(1, 2, 0),
        BGR(2, 1, 0);

        private final int redIndex;
        private final int greenIndex;
        private final int blueIndex;

        private ColorOrder(int redIndex, int greenIndex, int blueIndex) {
            this.redIndex = redIndex;
            this.greenIndex = greenIndex;
            this.blueIndex = blueIndex;
        }
    }

    static class Parameters implements Cloneable {
        /**
         * the address at which the sensor resides on the I2C bus.
         */
        public I2cAddr i2cAddr = I2CADDR_DEFAULT;

        /**
         * Number of pixels in the string
         */
        public int numPixels = 1;

        /**
         * Number of bytes per pixel. Use 3 for RGB or 4 for RGBW
         */
        public int bytesPerPixel = 3;

        /**
         * Order that the pixel colors are in
         */
        public ColorOrder colorOrder = ColorOrder.GRB;


        @Override
        public Parameters clone() {
            try {
                return (Parameters) super.clone();
            } catch (CloneNotSupportedException e) {
                throw new AssertionError();
            }
        }
    }

    public static Field getField(Class clazz, String fieldName) {
        try {
            Field f = clazz.getDeclaredField(fieldName);
            f.setAccessible(true);
            return f;
        } catch (NoSuchFieldException e) {
            Class superClass = clazz.getSuperclass();
            if (superClass != null) {
                return getField(superClass, fieldName);
            }
        }
        return null;
    }
}