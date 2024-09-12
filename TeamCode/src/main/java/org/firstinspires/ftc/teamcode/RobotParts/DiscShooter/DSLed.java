package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.Shooter.DSShooter;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;

public class DSLed implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   public int cols = 8;  //18,16
   public int rows = 8;

   public int attractDelay = 250;
   public long attractTime = 0;
//   int[][] attractMatrix = new int[1][8];

   int[][] messageMatrix = new int[cols][rows];
   int[][] normalMatrix = new int[cols][rows];
   int[][] finalMatrix = new int[cols][rows];

   private static Servo servoBlinkin;
   //private static ServoImplEx servoBlinkin;
   int servoBlinkinSetting, servoBlinkinSettingLast;

   /* Constructor */
   public DSLed(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize(){
      if (parts.useNeoMatrix) {
         parts.neo.initialize();
         parts.neo.flipVert=true;
         parts.neo.flipHoriz=true;
         parts.neo.setUpdateLimit(0);
         parts.neo.setPreventTearing(true);
         parts.neo.setDimmingValue(255);
         parts.neo.drawRectangle(0, 7, 0, 7, Color.rgb(10, 10, 0));
         normalMatrix = parts.neo.buildPixelMapFromString("abcd", marquis, Color.rgb(10,10,0), Color.rgb(0,0,0));
      }
      servoBlinkin = parts.robot.servo2B;
      ServoImplEx adjust = parts.opMode.hardwareMap.get(ServoImplEx.class,"servo2B");
      adjust.setPwmRange(new PwmControl.PwmRange(500, 2500));  // Default is 400-2400, but Blinkin wants 500-2500.
      setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_LARSON_SCANNER);  //60
   }

   public void preInit() {
   }

   public void initLoop() {
      if (parts.useNeoMatrix) {
         parts.neo.applyPixelMapToBuffer(normalMatrix, 0, 7, 0, true);
         //parts.neo.applyPixelMapToBuffer(parts.neo.reversePixelMap(normalMatrix), 8, 15, 0, true);
         normalMatrix = parts.neo.shiftPixelMap(normalMatrix, -8, 0, true);
         clearMessage();
         parts.neo.runLoop();
      }
   }

   public void preRun() {
      if (parts.useNeoMatrix) {
         parts.neo.clearMatrix();
         normalMatrix = new int[cols][rows];
         updateGraphic('4', MessageColor.G_LTGRAY);
//         parts.neo.applyPixelMapToBuffer(finalMatrix, 0, 15, 0, true);
         parts.neo.applyPixelMapToBuffer(finalMatrix, 0, 7, 0, true);
         parts.neo.forceUpdateMatrix();
         parts.neo.setUpdateLimit(2);
//         attractMatrix[0] = chase;
      }
   }

   public void runLoop() {
      if (parts.useNeoMatrix) {
         clearMessage();
//         parts.neo.applyPixelMapToBuffer(finalMatrix, 0, 15, 0, true);
         parts.neo.applyPixelMapToBuffer(finalMatrix, 0, 7, 0, true);

         //attract();
         parts.neo.runLoop();
      }
      autoSetBlinkin();
   }

   public void stop() {
   }

   long clearMessageTimer = 0;
   int messageDisplayTime = 1000;

   public void displayMessage (char msgChar, boolean boo) {
      if (boo) displayMessage(msgChar, MessageColor.GREEN);
      else displayMessage(msgChar, MessageColor.RED);
   }
   public void displayMessage (char msgChar, MessageColor messageColor) {
      displayMessage(msgChar, messageColor.color);
   }
   public void displayMessage (char msgChar, int color) {
      if (!parts.useNeoMatrix) return;
      clearMessageTimer = System.currentTimeMillis() + messageDisplayTime;
//      int msgColor;
      int[][] textMatrix;
//      switch (color) {
//         case 2:
//            msgColor = Color.rgb(0,40,0);
//            break;
//         case 3:
//            msgColor = Color.rgb(40,0,0);
//            break;
//         case 4:
//            msgColor = Color.rgb(0,0,40);
//            break;
//         case 1:
//         default:
//            msgColor = Color.rgb(20,20,20);
//      }
      textMatrix = parts.neo.buildPixelMapFromString(String.valueOf(msgChar), parts.neo.bigLetters, color);
      messageMatrix = new int[cols][rows];
      messageMatrix = parts.neo.overlayPixelMap(textMatrix, messageMatrix, 2);
      //messageMatrix = parts.neo.overlayPixelMap(textMatrix, messageMatrix, 10);
      finalMatrix = parts.neo.cloneArray(messageMatrix);
   }

   public void updateGraphic (char msgChar, MessageColor messageColor) {
      updateGraphic(msgChar, messageColor.color);
   }
   public void updateGraphic (char msgChar, int color) {
      if (!parts.useNeoMatrix) return;
      int[][] textMatrix;
      textMatrix = parts.neo.buildPixelMapFromString(String.valueOf(msgChar), circles, color);
      normalMatrix = parts.neo.overlayPixelMap(textMatrix, normalMatrix,0);
      //normalMatrix = parts.neo.overlayPixelMap(textMatrix, normalMatrix,8);
   }

   public void clearMessage () {
      if (clearMessageTimer == 0) {
         finalMatrix = parts.neo.cloneArray(normalMatrix);
         return;
      }
      if (System.currentTimeMillis() >= clearMessageTimer) {
         clearMessageTimer = 0;
      }
   }

//   public void attract () {
//      if (System.currentTimeMillis() > attractTime) {
//         attractTime = System.currentTimeMillis() + attractDelay;
//         attractMatrix = parts.neo.shiftPixelMap(attractMatrix,0,1,true);
//      }
//      parts.neo.applyPixelMapToBuffer(attractMatrix, 16, 16,0,true);
//   }

   public void setBlinkinPattern(int pattern) {
      // Q: Why do this craziness instead of defining the servo port as a Blinkin?
      // A: Because that changes the stored config and the Robot class. Easier to just use all the servo ports as servos.
      if (pattern<1 || pattern>100) return;              // Or throw an error? Legal Blinkin patters are between 1 and 100
      int pulseWidth = 995 + 10*pattern;                 // Convert pattern number to pulse width between 1000-2000 μs)
      double setting = (pulseWidth - 500) / 2000.0;      // Covert pulse width to 0-1 servo position (based on 500-2500 μs)
      servoBlinkin.setPosition(setting);
   }

   public void setBlinkinPattern(BlinkinPattern pattern) {
      setBlinkinPattern(pattern.ordinal()+1);            // Ordinal starts at 0, so need to add 1 (convert 0-99 to 1-100)
   }

   public void autoSetBlinkin() {
      if (DSShooter.isArmed) servoBlinkinSetting = BlinkinPattern.FIRE_MEDIUM.ordinal();  //21
      else if (parts.autoDrive.isNavigating) servoBlinkinSetting = BlinkinPattern.CP1_2_COLOR_GRADIENT.ordinal();  //71
      else if (parts.userDrive.isDriving) servoBlinkinSetting = BlinkinPattern.CP1_LIGHT_CHASE.ordinal();   //51
      else servoBlinkinSetting = BlinkinPattern.TWINKLES_OCEAN_PALETTE.ordinal(); //25 //43;
      servoBlinkinSetting++;                            // Convert ordinal 0-99 to 1-100
      if (servoBlinkinSetting != servoBlinkinSettingLast) setBlinkinPattern(servoBlinkinSetting);
      servoBlinkinSettingLast = servoBlinkinSetting;
   }

   public enum MessageColor {
      GREEN (Color.rgb(0,40,0)),
      RED (Color.rgb(40,0,0)),
      BLUE (Color.rgb(0,0,40)),
      GRAY (Color.rgb(20,20,20)),
      G_GREEN (Color.rgb(0,20,0)),
      G_RED (Color.rgb(20,0,0)),
      G_ORANGE (Color.rgb(20,10,0)),
      G_GRUE(Color.rgb(0,40,10)),
      G_LTGRAY (Color.rgb(2, 2, 2)),
      G_BLUE (Color.rgb(0,0,20)),
      G_PURPLE (Color.rgb(5,0,15));
      public final int color;
      MessageColor(int color) {
         this.color = color;
      }
   }

   public final char[][] marquis = {
           {'a', 17, 128, 0, 0, 1, 128, 0, 34},
           {'b', 34, 0, 128, 1, 0, 0, 128, 17},
           {'c', 68, 0, 1, 128, 0, 0, 1, 136},
           {'d', 136, 1, 0, 0, 128, 1, 0, 68},
           {'e', 238, 1, 129, 129, 128, 1, 129, 221},
           {'f', 221, 129, 1, 128, 129, 129, 1, 238},
           {'g', 187, 129, 128, 1, 129, 129, 128, 119},
           {'h', 119, 128, 129, 129, 1, 128, 129, 187} };

   public final char[][] circles = {
           {'1', 0, 0, 0, 24, 24, 0, 0, 0},
           {'2', 0, 0, 60, 36, 36, 60, 0, 0},
           {'3', 0, 60, 66, 66, 66, 66, 60, 0},
           {'4', 60, 66, 129, 129, 129, 129, 66, 60} };

   public final int[] chase = { 10, 20, 40, 60, 20, 0, 0, 0};
}

/* For reference, here are the Blinkin colors by index and name:
1	RAINBOW_RAINBOW_PALETTE
2	RAINBOW_PARTY_PALETTE
3	RAINBOW_OCEAN_PALETTE
4	RAINBOW_LAVA_PALETTE
5	RAINBOW_FOREST_PALETTE
6	RAINBOW_WITH_GLITTER
7	CONFETTI
8	SHOT_RED
9	SHOT_BLUE
10	SHOT_WHITE
11	SINELON_RAINBOW_PALETTE
12	SINELON_PARTY_PALETTE
13	SINELON_OCEAN_PALETTE
14	SINELON_LAVA_PALETTE
15	SINELON_FOREST_PALETTE
16	BEATS_PER_MINUTE_RAINBOW_PALETTE
17	BEATS_PER_MINUTE_PARTY_PALETTE
18	BEATS_PER_MINUTE_OCEAN_PALETTE
19	BEATS_PER_MINUTE_LAVA_PALETTE
20	BEATS_PER_MINUTE_FOREST_PALETTE
21	FIRE_MEDIUM
22	FIRE_LARGE
23	TWINKLES_RAINBOW_PALETTE
24	TWINKLES_PARTY_PALETTE
25	TWINKLES_OCEAN_PALETTE
26	TWINKLES_LAVA_PALETTE
27	TWINKLES_FOREST_PALETTE
28	COLOR_WAVES_RAINBOW_PALETTE
29	COLOR_WAVES_PARTY_PALETTE
30	COLOR_WAVES_OCEAN_PALETTE
31	COLOR_WAVES_LAVA_PALETTE
32	COLOR_WAVES_FOREST_PALETTE
33	LARSON_SCANNER_RED
34	LARSON_SCANNER_GRAY
35	LIGHT_CHASE_RED
36	LIGHT_CHASE_BLUE
37	LIGHT_CHASE_GRAY
38	HEARTBEAT_RED
39	HEARTBEAT_BLUE
40	HEARTBEAT_WHITE
41	HEARTBEAT_GRAY
42	BREATH_RED
43	BREATH_BLUE
44	BREATH_GRAY
45	STROBE_RED
46	STROBE_BLUE
47	STROBE_GOLD
48	STROBE_WHITE
49	CP1_END_TO_END_BLEND_TO_BLACK
50	CP1_LARSON_SCANNER
51	CP1_LIGHT_CHASE
52	CP1_HEARTBEAT_SLOW
53	CP1_HEARTBEAT_MEDIUM
54	CP1_HEARTBEAT_FAST
55	CP1_BREATH_SLOW
56	CP1_BREATH_FAST
57	CP1_SHOT
58	CP1_STROBE
59	CP2_END_TO_END_BLEND_TO_BLACK
60	CP2_LARSON_SCANNER
61	CP2_LIGHT_CHASE
62	CP2_HEARTBEAT_SLOW
63	CP2_HEARTBEAT_MEDIUM
64	CP2_HEARTBEAT_FAST
65	CP2_BREATH_SLOW
66	CP2_BREATH_FAST
67	CP2_SHOT
68	CP2_STROBE
69	CP1_2_SPARKLE_1_ON_2
70	CP1_2_SPARKLE_2_ON_1
71	CP1_2_COLOR_GRADIENT
72	CP1_2_BEATS_PER_MINUTE
73	CP1_2_END_TO_END_BLEND_1_TO_2
74	CP1_2_END_TO_END_BLEND
75	CP1_2_NO_BLENDING
76	CP1_2_TWINKLES
77	CP1_2_COLOR_WAVES
78	CP1_2_SINELON
79	HOT_PINK
80	DARK_RED
81	RED
82	RED_ORANGE
83	ORANGE
84	GOLD
85	YELLOW
86	LAWN_GREEN
87	LIME
88	DARK_GREEN
89	GREEN
90	BLUE_GREEN
91	AQUA
92	SKY_BLUE
93	DARK_BLUE
94	BLUE
95	BLUE_VIOLET
96	VIOLET
97	WHITE
98	GRAY
99	DARK_GRAY
100	BLACK
*/