// Googly Eye Goggles
// By Bill Earl
// For Adafruit Industries
//
// The googly eye effect is based on a physical model of a pendulum.
// The pendulum motion is driven by accelerations in 2 axis.
// Eye color varies with orientation of the magnetometer

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

#define GND1   2
#define GND2   3
#define VCC1   5
#define VCC2   7
#define neoPixelPinLeft 9
#define neoPixelPinRight 10
#define IRDIODE  12
#define IRTRANSL A1
#define IRTRANSR A3

#define IR_INITDELAY 2

#define BRIGHTNESS 50

#define WAVE_DETECT_FACTOR 2
#define TRASFER_THRESHOLD 400


const int numPixelRing = 24;

// Pi for calculations - not the raspberry type
const float Pi = 3.14159;

// Borrowed from _lsm303Accel_MG_LSB in Adafruit_LSM303_U.cpp
//const float _lsm303Accel_MG_LSB     = 0.001F;   // 1, 2, 4 or 12 mg per lsb
const float mpu6050Accel_MG_LSB     = 0.0001F;   // 0.1 mg per lsb

// We could do this as 2 16-pixel rings wired in parallel.
// But keeping them separate lets us do the right and left
// eyes separately if we want.
//Adafruit_NeoPixel ringLeft = Adafruit_NeoPixel(numPixelRing, neoPixelPinLeft, NEO_GRB + NEO_KHZ800);
//Adafruit_NeoPixel ringRight = Adafruit_NeoPixel(numPixelRing, neoPixelPinRight, NEO_GRB + NEO_KHZ800);

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float maxRGB;
boolean swipeOverrside = false;

 
float pos = numPixelRing/2;  // Starting center position of pupil
float increment = 2 * Pi / numPixelRing; // distance between pixels in radians
float MomentumH = 0; // horizontal component of pupil rotational inertia
float MomentumV = 0; // vertical component of pupil rotational inertia

// Tuning constants. (a.k.a. "Fudge Factors)  
// These can be tweaked to adjust the liveliness and sensitivity of the eyes.
const float friction = 0.985; // <- manually tweaked. Original was 0.995; // frictional damping constant.  1.0 is no friction.
const float swing = 60;  // arbitrary divisor for gravitational force
const float gravity = 200;  // arbitrary divisor for lateral acceleration


bool antiGravity = false;  // The pendulum will anti-gravitate to the top.
bool mirroredEyes = false; // The left eye will mirror the right.

bool spinLeft = false;
bool spinRight = false;

const float halfWidth = 2.75; //1.25; // half-width of pupil (in pixels)


byte factorLeft = 0;
byte factorRight = 0;
byte factorLeftPrevious = 99;
byte factorRightPrevious = 99;

long lastKnownWaveInTimeLeft = 0;
long lastKnownWaveInTimeRight = 2000; //purpose set it 2 seconds apart from lastKnownWaveInTimeLeft
long lastKnownWaveOutTimeLeft = 4000; //purpose set it 2 seconds apart
long lastKnownWaveOutTimeRight = 8000; //purpose set it 2 seconds apart





const byte WIPEINTERVAL = 20;
const byte RAINBOWINTERVAL = 1;


/*
const int maxColorWipe = 5;
const int totalColorChase = 10;
const byte SCANNERINTERVAL = 40;
const byte CHASEINTERVAL = 10;
const byte BRI_SCANNER = 100;
const byte BRI_THEATER = 20;
const byte BRI_RAINBOW = 5;
const byte BRI_WIPE = 10;
const byte BRI_WAVE = 5;
const byte BRI_CHASE = 20;
long count = 0;
int colorWipeCount = 0;
int G_flag = 1;
int RGB = 0;
int RGB_val[3];
*/


boolean enableInterlude = false;

// Pattern types supported:
enum  pattern { NONE, RAINBOW_CYCLE, THEATER_CHASE, COLOR_WIPE, SCANNER, FADE };
// Patern directions supported:
enum  direction { FORWARD, REVERSE };

// NeoPattern Class - derived from the Adafruit_NeoPixel class
class NeoPatterns : public Adafruit_NeoPixel
{
    public:

    // Member Variables:  
    pattern  ActivePattern;  // which pattern is running
    direction Direction;     // direction to run the pattern
    
    unsigned long Interval;   // milliseconds between updates
    unsigned long lastUpdate; // last update of position
    
    uint32_t Color1, Color2;  // What colors are in use
    uint16_t TotalSteps;  // total number of steps in the pattern
    //uint16_t Index;  // current step within the pattern
    int Index;
    
    void (*OnComplete)();  // Callback on completion of pattern
    
    // Constructor - calls base-class constructor to initialize strip
    NeoPatterns(uint16_t pixels, uint8_t pin, uint8_t type, void (*callback)())
    :Adafruit_NeoPixel(pixels, pin, type)
    {
        OnComplete = callback;
    }
    
    // Update the pattern
    void Update()
    {
        if((millis() - lastUpdate) > Interval) // time to update
        {
            lastUpdate = millis();
            switch(ActivePattern)
            {
                case RAINBOW_CYCLE:
                    RainbowCycleUpdate();
                    break;
                case THEATER_CHASE:
                    TheaterChaseUpdate();
                    break;
                case COLOR_WIPE:
                    ColorWipeUpdate();
                    break;
                case SCANNER:
                    ScannerUpdate();
                    break;
                case FADE:
                    FadeUpdate();
                    break;
                default:
                    break;
            }
        }
    }
    
    
    void OnCompleteLocal() {
            switch(ActivePattern)
            {
                case RAINBOW_CYCLE:
                    //RainbowCycle(random(0,10));
                    break;
                case THEATER_CHASE:
                    //Reverse();
                    break;
                case COLOR_WIPE:
                    Color1 = Wheel(random(255));
                    //Interval = 20000;
                    break;
                    /*
                case SCANNER:
                    Color1 = Wheel(random(255));
                    count++;        
                    break;
                    */
                case FADE:
                    Color1 = Wheel(random(255));
                    //Interval = 20000;
                    break;
                default:
                    break;
            }      
      
    }
	
    // Increment the Index and reset at the end
    void Increment()
    {
        if (Direction == FORWARD)
        {
           Index++;
           if (Index >= TotalSteps)
            {
                Index = 0;
                if (OnComplete != NULL)
                {
                    OnComplete(); // call the comlpetion callback
                }
                else
                {
                    OnCompleteLocal();
                }
            }
        }
        else // Direction == REVERSE
        {
            --Index;
            if (Index < 0)
            {
                Index = TotalSteps-1;
                if (OnComplete != NULL)
                {
                    OnComplete(); // call the comlpetion callback
                }
                else
                {
                    OnCompleteLocal();
                }
            }
        }
    }
    
    
    // Reverse pattern direction
    void Reverse()
    {
        if (Direction == FORWARD)
        {
            Direction = REVERSE;
            Index = TotalSteps-1;
        }
        else
        {
            Direction = FORWARD;
            Index = 0;
        }
    }
    
    // Initialize for a RainbowCycle
    void RainbowCycle(uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = RAINBOW_CYCLE;
        Interval = interval;
        TotalSteps = 255;
        Index = 0;
        Direction = dir;
        
        //setBrightness(BRI_RAINBOW);
    }
    
    // Update the Rainbow Cycle Pattern
    void RainbowCycleUpdate()
    {
        for(int i=0; i< numPixels(); i++)
        {
            setPixelColor(i, Wheel(((i * 256 / numPixels()) + Index) & 255));
        }
        show();
        Increment();
    }

    // Initialize for a Theater Chase
    void TheaterChase(uint32_t color1, uint32_t color2, uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = THEATER_CHASE;
        Interval = interval;
        TotalSteps = numPixels();
        Color1 = color1;
        Color2 = color2;
        Index = 0;
        Direction = dir;
        
        //setBrightness(BRI_THEATER);
   }
    
    // Update the Theater Chase Pattern
    void TheaterChaseUpdate()
    {
        for(int i=0; i< numPixels(); i++)
        {
            if ((i + Index) % 3 == 0)
            {
                setPixelColor(i, Color1);
            }
            else
            {
                setPixelColor(i, Color2);
            }
        }
        show();
        Increment();
    }

    // Initialize for a ColorWipe
    void ColorWipe(uint32_t color, uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = COLOR_WIPE;
        Interval = interval;
        TotalSteps = numPixels();
        Color1 = color;
        Index = 0;
        Direction = dir;
        
        //setBrightness(BRI_WIPE);
    }
    
    // Update the Color Wipe Pattern
    void ColorWipeUpdate()
    {
        setPixelColor(Index, Color1);
        show();
        Increment();
    }
    
    // Initialize for a SCANNNER
    void Scanner(uint32_t color1, uint8_t interval)
    {
        ActivePattern = SCANNER;
        Interval = interval;
        TotalSteps = (numPixels() - 1) * 2;
        Color1 = color1;
        Index = 0;
        
        //setBrightness(BRI_SCANNER);
    }

    // Update the Scanner Pattern
    void ScannerUpdate()
    { 
        for (int i = 0; i < numPixels(); i++)
        {
            if (i == Index)  // Scan Pixel to the right
            {
                 setPixelColor(i, Color1);
            }
            else if (i == TotalSteps - Index) // Scan Pixel to the left
            {
                 setPixelColor(i, Color1);
            }
            else // Fading tail
            {
                 setPixelColor(i, DimColor(getPixelColor(i)));
            }
        }
        show();
        Increment();
    }
    
    // Initialize for a Fade
    void Fade(uint32_t color1, uint32_t color2, uint16_t steps, uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = FADE;
        Interval = interval;
        TotalSteps = steps;
        Color1 = color1;
        Color2 = color2;
        Index = 0;
        Direction = dir;
    }
    
    // Update the Fade Pattern
    void FadeUpdate()
    {
        // Calculate linear interpolation between Color1 and Color2
        // Optimise order of operations to minimize truncation error
        uint8_t red = ((Red(Color1) * (TotalSteps - Index)) + (Red(Color2) * Index)) / TotalSteps;
        uint8_t green = ((Green(Color1) * (TotalSteps - Index)) + (Green(Color2) * Index)) / TotalSteps;
        uint8_t blue = ((Blue(Color1) * (TotalSteps - Index)) + (Blue(Color2) * Index)) / TotalSteps;
        
        ColorSet(Color(red, green, blue));
        show();
        Increment();
    }
   
    // Calculate 50% dimmed version of a color (used by ScannerUpdate)
    uint32_t DimColor(uint32_t color)
    {
        // Shift R, G and B components one bit to the right
        uint32_t dimColor = Color(Red(color) >> 1, Green(color) >> 1, Blue(color) >> 1);
        //uint32_t dimColor = Color(Red(color)*88/100, Green(color)*88/100, Blue(color)*88/100);
        
        return dimColor;
    }

    // Set all pixels to a color (synchronously)
    void ColorSet(uint32_t color)
    {
        for (int i = 0; i < numPixels(); i++)
        {
            setPixelColor(i, color);
        }
        show();
    }

    // Returns the Red component of a 32-bit color
    uint8_t Red(uint32_t color)
    {
        return (color >> 16) & 0xFF;
    }

    // Returns the Green component of a 32-bit color
    uint8_t Green(uint32_t color)
    {
        return (color >> 8) & 0xFF;
    }

    // Returns the Blue component of a 32-bit color
    uint8_t Blue(uint32_t color)
    {
        return color & 0xFF;
    }
    
    // Input a value 0 to 255 to get a color value.
    // The colours are a transition r - g - b - back to r.
    uint32_t Wheel(byte WheelPos)
    {
        WheelPos = 255 - WheelPos;
        if(WheelPos < 85)
        {
            return Color(255 - WheelPos * 3, 0, WheelPos * 3);
        }
        else if(WheelPos < 170)
        {
            WheelPos -= 85;
            return Color(0, WheelPos * 3, 255 - WheelPos * 3);
        }
        else
        {
            WheelPos -= 170;
            return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
        }
    }
};







NeoPatterns ringLeft(numPixelRing, neoPixelPinLeft, NEO_GRB + NEO_KHZ800, NULL);
NeoPatterns ringRight(numPixelRing, neoPixelPinRight, NEO_GRB + NEO_KHZ800, NULL);



void setup(void) 
{
  
     Serial.begin(115200);
  


   // Initialize the sensors
   Wire.begin();
	
    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    
   ringLeft.begin();
   ringLeft.setBrightness(BRIGHTNESS); // Lower brightness and save eyeballs!
   ringLeft.show(); // Initialize all pixels to 'off'  sensor_t sensor;
   ringRight.begin();
   ringRight.setBrightness(BRIGHTNESS); // Lower brightness and save eyeballs!
   ringRight.show(); // Initialize all pixels to 'off'  sensor_t sensor;   
   
   resetModes();
   
    pinMode(VCC1, OUTPUT);
    digitalWrite(VCC1, HIGH);  
    pinMode(VCC2, OUTPUT);
    digitalWrite(VCC2, HIGH);  
    pinMode(GND1, OUTPUT);
    digitalWrite(GND1, LOW);  
    pinMode(GND2, OUTPUT);
    digitalWrite(GND2, LOW);  
    
    pinMode(IRDIODE, OUTPUT);
    digitalWrite(IRDIODE, HIGH);  
    pinMode(IRTRANSL, INPUT_PULLUP);  
    pinMode(IRTRANSR, INPUT_PULLUP);  
   

}

// main processing loop
void loop(void) 
{
   readFromIR();
   /*
   if (lastKnownWaveOutTimeLeft > lastKnownWaveInTimeRight) {
     Serial.print("Case 1 :");
     Serial.println(lastKnownWaveOutTimeLeft-lastKnownWaveInTimeRight);
   }
   
   if (lastKnownWaveInTimeRight > lastKnownWaveOutTimeRight) {
     Serial.print("Case 2 :");
     Serial.println(lastKnownWaveInTimeRight-lastKnownWaveOutTimeRight);     
   }
   */
   
   
   
   
   if (  (lastKnownWaveOutTimeLeft < lastKnownWaveInTimeRight) && ( (lastKnownWaveInTimeRight-lastKnownWaveOutTimeLeft) < TRASFER_THRESHOLD) &&
         (lastKnownWaveInTimeRight < lastKnownWaveOutTimeRight) && ( (lastKnownWaveOutTimeRight-lastKnownWaveInTimeRight) < TRASFER_THRESHOLD) ) {
     //transfer Left to Right
     //Serial.println("actually in case 1-2");
     lastKnownWaveOutTimeLeft = 0;
     clearAll();
     spinRight = true;
     spinUp();
     spinRight = false;
     return;
   }
   
   /*
   
   if (lastKnownWaveOutTimeRight > lastKnownWaveInTimeLeft) {
     Serial.print("Case 3 :");
     Serial.println(lastKnownWaveOutTimeRight-lastKnownWaveInTimeLeft);
   }
   
   if (lastKnownWaveInTimeLeft > lastKnownWaveOutTimeLeft) {
     Serial.print("Case 4 :");
     Serial.println(lastKnownWaveInTimeLeft-lastKnownWaveOutTimeLeft);     
   }   
   */
   
   
   if (  (lastKnownWaveOutTimeRight < lastKnownWaveInTimeLeft) && ( (lastKnownWaveInTimeLeft-lastKnownWaveOutTimeRight) < TRASFER_THRESHOLD) &&
         (lastKnownWaveInTimeLeft < lastKnownWaveOutTimeLeft) && ( (lastKnownWaveOutTimeLeft-lastKnownWaveInTimeLeft) < TRASFER_THRESHOLD) ) {
     //transfer Right to Left
     //Serial.println("actually in case 3-4");
     lastKnownWaveOutTimeRight = 0;
     clearAll();
     spinLeft = true;
     spinUp();
     spinLeft = false;
     return;
   }   
   
   switch (factorLeft) {
     
     case 0:
     ringLeft.ActivePattern = NONE;
     break;
     
     case 1:
     case 2:
     case 3:
      if (ringLeft.ActivePattern == THEATER_CHASE) {
        ringLeft.Update();
      } else {
        ringLeft.TheaterChase(ringLeft.Color(64+random(255-64),64+random(255-64),0), ringLeft.Color(0,0,64+random(128)), 100);
        ringLeft.Reverse();
      }     
      break;
        
      case 4:
      case 5:
      case 6:
        if (ringLeft.ActivePattern == RAINBOW_CYCLE) {
          ringLeft.Update();
        } else {
          ringLeft.RainbowCycle(RAINBOWINTERVAL);
          ringLeft.Reverse();
        }   
      break;
      
      default:
      if (ringLeft.ActivePattern == COLOR_WIPE) {
        ringLeft.Update();
      } else {
        ringLeft.ColorWipe(ringLeft.Color(64+random(255-64),64+random(255-64),64+random(255-64)), WIPEINTERVAL);
      }      
   
   }
   
   switch (factorRight) {
     
     case 0:
     ringRight.ActivePattern = NONE;
     break;
     
     case 1:
     case 2:
     case 3:
      if (ringRight.ActivePattern == THEATER_CHASE) {
        ringRight.Update();
      } else {
        ringRight.TheaterChase(ringRight.Color(64+random(255-64),64+random(255-64),0), ringRight.Color(0,0,64+random(128)), 100);
      }     
      break;
        
      case 4:
      case 5:
      case 6:
        if (ringRight.ActivePattern == RAINBOW_CYCLE) {
          ringRight.Update();
        } else {
          ringRight.RainbowCycle(RAINBOWINTERVAL);
        }  
      break;
      
      default:
      
      if (ringRight.ActivePattern == COLOR_WIPE) {
        ringRight.Update();
      } else {
        ringRight.ColorWipe(ringRight.Color(64+random(255-64),64+random(255-64),64+random(255-64)), WIPEINTERVAL);
        ringRight.Reverse();
      }      
      
    
   }   
   
   
   if ( (ringLeft.ActivePattern != NONE) && (ringRight.ActivePattern != NONE)) {
     //both rings are activated by IR, skip MPU6050 sensor reads
     return;
   }
   
  

   // Read the acceleration and gyro rotation
   sensors_event_t event; 
   accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
   
   // accel.getEvent(&event);
   
   event.acceleration.x = ax * mpu6050Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
   event.acceleration.y = ay * mpu6050Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
   event.acceleration.z = az * mpu6050Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
   
/*

    Serial.print("accel:\t");
        Serial.print(event.acceleration.x); Serial.print("\t");
        Serial.print(event.acceleration.y); Serial.print("\t");
        Serial.println(event.acceleration.z); 
        
        */

   // apply a little frictional damping to keep things in control and prevent perpetual motion
   MomentumH *= friction;
   MomentumV *= friction;

   // Calculate the horizontal and vertical effect on the virtual pendulum
   // 'pos' is a pixel address, so we multiply by 'increment' to get radians.
   float TorqueH = cos(pos * increment);  // peaks at top and bottom of the swing
   float TorqueV = sin(pos * increment);    // peaks when the pendulum is horizontal

   // Add the incremental acceleration to the existing momentum
   // This code assumes that the accelerometer is mounted upside-down, level
   // and with the X-axis pointed forward.  So the Y axis reads the horizontal
   // acceleration and the inverse of the Z axis is gravity.
   // For other orientations of the sensor, just change the axis to match.
   MomentumH += TorqueH * event.acceleration.y / swing;
   
   //if (antiGravity)
   //{
   //  MomentumV += TorqueV * event.acceleration.x / gravity;
   //}
   //else
   //{
     MomentumV -= TorqueV * event.acceleration.x / gravity;
   //}

   // Calculate the new position
   pos += MomentumH + MomentumV;
   
   // handle the wrap-arounds at the top
   while (round(pos) < 0) pos += numPixelRing;
   while (round(pos) > (numPixelRing-1)) pos -= numPixelRing;

   // Now re-compute the display
   for (int i = 0; i < numPixelRing; i++)
   {
      // Compute the distance bewteen the pixel and the center
      // point of the virtual pendulum.
      float diff = i - pos;

      // Light up nearby pixels proportional to their proximity to 'pos'
      if (fabs(diff) <= halfWidth) 
      {
         uint32_t color;

         // pick a color based on heading & proximity to 'pos'
         color = selectColorGyro(gx, gy, gz, fabs(diff));
         
         // do both eyes
         if (ringLeft.ActivePattern == NONE) {
           ringLeft.setPixelColor(i, color);
         }
         if (ringRight.ActivePattern == NONE) {
	   ringRight.setPixelColor(i, color);
         }
      }
      else // all others are off
      {
        if (ringLeft.ActivePattern == NONE) {
         ringLeft.setPixelColor(i, 0);
        }
        if (ringRight.ActivePattern == NONE) {
         ringRight.setPixelColor(i, 0);
        }
      }
   }
   // Now show it!
   if (ringLeft.ActivePattern == NONE) {
     ringLeft.show();
   }
   if (ringRight.ActivePattern == NONE) {
     ringRight.show();
   }
}


// choose a color based on the gyro's rolation
uint32_t selectColorGyro(int16_t gx, int16_t gy, int16_t gz, float diff)
{
    uint32_t color;
	 
	// gyro values
	uint16_t gx_abs = abs(gx);
	uint16_t gy_abs = abs(gy);
	uint16_t gz_abs = abs(gz);	 
	
	uint8_t blue_uint8 = gx_abs / 128;
	uint8_t green_uint8 = gy_abs / 128;
	uint8_t red_uint8 = gz_abs / 128;	 

        //make some color when stand still
        
        if ( (red_uint8 < 10) && (green_uint8 < 10) && (blue_uint8 < 10) ) {
          
          if (red_uint8 == 0) {
            red_uint8 = 2;
          }
          if (green_uint8 == 0) {
            green_uint8 = 2;
          }
          if (blue_uint8 == 0) {
            blue_uint8 = 2;
          }     
     
          red_uint8 *= 16;
          green_uint8 *= 16;
          blue_uint8 *= 16;
          
        }


	
	uint8_t dim_factor = 1;
	
	if (diff < 0.6 ) {
		dim_factor = 1;
	} else if (diff < 1.2) {
		dim_factor = 4;
	} else if (diff < 2.4) {
		dim_factor = 8;
	} else {
		dim_factor = 16;
	}

/*

    Serial.print("mag dim_factor=");
    Serial.print(dim_factor);
    Serial.print("  diff=");
    Serial.print(diff);    
     Serial.print(" :\t");
        Serial.print(red_uint8/dim_factor); Serial.print("\t");
        Serial.print(green_uint8/dim_factor); Serial.print("\t");
        Serial.println(blue_uint8/dim_factor); 
        
        */
        
        if (swipeOverrside) {
          red_uint8 = green_uint8 = blue_uint8 = maxRGB;
        }        
	
	color = ringLeft.Color( red_uint8/dim_factor, green_uint8/dim_factor, blue_uint8/dim_factor);


	 
}



// Reset to default
void resetModes()
{
  //spinRight = true;
  //spinLeft = true;
   //spinUp();
   //swipeOverrside = true;
   
   
   pos = 0;
   // leave it with some momentum and let it 'coast' to a stop
   MomentumH = 2; 

}


// gradual spin up
void spinUp()
{
  
  maxRGB = 255.0;
  

   for (int i = 50; i > 10;  i -= 2)
   {
     spin(ringLeft.Color(maxRGB,maxRGB,maxRGB), 1, i);
     maxRGB *= 0.95;
   }


   
   pos = 0;
   // leave it with some momentum and let it 'coast' to a stop
   if (spinLeft) {
     MomentumH = 2; 
   } 
   if (spinRight) {
     MomentumH = -2; 
   }
}

// Gradual spin down
void spinDown()
{
   for (int i = 10; i < 50; i++)
   {
     spin(ringLeft.Color(255,255,255), 1, i += 1);
   }
   
   pos = 0;
   // leave it with some momentum and let it 'coast' to a stop
   MomentumH = 2;  
   
   // alternativaly stop it dead at the top and let it swing to the bottom on its own
   //MomentumH = MomentumV = 0;
}



void spinUpDown()
{
  
  float maxRGB = 255.0;
  

   for (int i = 50; i > 10;  i -= 2)
   {
     spin(ringLeft.Color(maxRGB,maxRGB,maxRGB), 1, i);
     maxRGB *= 0.96;
   }

   for (int i = 10; i < 1000; i+=200)
   {
     spin(ringLeft.Color(maxRGB,maxRGB,maxRGB), 1, i);
     maxRGB *= 0.9;
   }
   
   pos = 0;
   //stop it dead at the top and let it swing to the bottom on its own
   MomentumH = MomentumV = 0;
}


// utility function for feedback on mode changes.
void spin(uint32_t color, int count, int time)
{
  for (int j = 0; j < count; j++)
  {
    for (int i = 0; i < numPixelRing; i++)
    {
      
      if (spinLeft) {
        ringLeft.setPixelColor(i, color);
        ringLeft.show();
      }
      if (spinRight) {
        ringRight.setPixelColor((numPixelRing-1) - i, color);
        ringRight.show();
      }
      delay(max(time / numPixelRing, 1));
      if (spinLeft) {
        ringLeft.setPixelColor(i, 0);
        ringLeft.show();
      }
      if (spinRight) {
        ringRight.setPixelColor((numPixelRing-1) - i, 0);
        ringRight.show();
      }
    }
  }
}


void clearAll() {
    for (int i = 0; i < numPixelRing; i++)
    {
      ringLeft.setPixelColor(i, 0);
      ringRight.setPixelColor(i, 0);
    } 
    ringLeft.show();
    ringRight.show();
}



void readFromIR() {
  
  digitalWrite(IRDIODE, HIGH);  
  delay(IR_INITDELAY);
  
  int irleft = analogRead(IRTRANSL);
  int irright = analogRead(IRTRANSR);
  
  digitalWrite(IRDIODE, LOW);    
  
  setParametersLeftRing(irleft);
  //setParametersLeftRing(900);
  setParametersRightRing(irright);
  //setParametersRightRing(900);

}

void setParametersLeftRing(int irLeft) {
  
  byte previousFactor = factorLeft;
  
  factorLeftPrevious = factorLeft;
  
  if (irLeft > 968) {
    factorLeft = 0;
  }
  else if (irLeft > 950) {
    factorLeft = 1; 
  }
  else if (irLeft > 900) {
    factorLeft = 2; 
  }
  else if (irLeft > 850) {
    factorLeft = 3; 
  }  
  else if (irLeft > 700) {
    factorLeft = 4; 
  }  
  else if (irLeft > 500) {
    factorLeft = 5; 
  }
  else if (irLeft > 400) {
    factorLeft = 6;
  }
  else if (irLeft > 300) {
    factorLeft = 8; 
  }
  else if (irLeft > 200) {
    factorLeft = 10; 
  }
  else if (irLeft > 100) {
    factorLeft = 12; 
  }
  else if (irLeft > 50) {
    factorLeft = 15; 
  }
  else {
    factorLeft = 20; 
  }
  
  /*
  
  if (factorLeft < 13) {
    if ( (factorLeft - previousFactor) > WAVE_DETECT_FACTOR) {
      Serial.print("lastKnownWaveInTimeLeft triggered: \t");
      
      lastKnownWaveInTimeLeft = millis(); 
      Serial.println(lastKnownWaveInTimeLeft);
    }

    if ( (previousFactor - factorLeft) > WAVE_DETECT_FACTOR) {
      
      Serial.print("lastKnownWaveOutTimeLeft triggered: \t");
      
      lastKnownWaveOutTimeLeft = millis(); 
      
      Serial.println(lastKnownWaveOutTimeLeft);
    }	
  }
  
  */
  
  
  
    if ( (factorLeftPrevious == 0) && (factorLeft > 0) ) {
      Serial.print("lastKnownWaveInTimeLeft triggered: \t");
      
      lastKnownWaveInTimeLeft = millis(); 
      Serial.println(lastKnownWaveInTimeLeft);
    }

    if ( (factorLeftPrevious > 0) && (factorLeft == 0) ) {
      
      Serial.print("lastKnownWaveOutTimeLeft triggered: \t");
      
      lastKnownWaveOutTimeLeft = millis(); 
      
      Serial.println(lastKnownWaveOutTimeLeft);
    }	  
    
    

}


void setParametersRightRing(int irRight) {
  
  byte previousFactor = factorRight;
  factorRightPrevious  = factorRight;
  
  if (irRight > 968) {
    factorRight = 0;
  }
  else if (irRight > 950) {
    factorRight = 1; 
  }
  else if (irRight > 900) {
    factorRight = 2; 
  }
  else if (irRight > 850) {
    factorRight = 3; 
  }  
  else if (irRight > 700) {
    factorRight = 4; 
  }  
  else if (irRight > 500) {
    factorRight = 5; 
  }
  else if (irRight > 400) {
    factorRight = 6;
  }
  else if (irRight > 300) {
    factorRight = 8; 
  }
  else if (irRight > 200) {
    factorRight = 10; 
  }
  else if (irRight > 100) {
    factorRight = 12; 
  }
  else if (irRight > 50) {
    factorRight = 15; 
  }
  else {
    factorRight = 20; 
  }
  
  /*
  if (factorRight < 13) {
    if ((factorRight - previousFactor) > WAVE_DETECT_FACTOR) {
      Serial.print("lastKnownWaveInTimeRight triggered:\t");
      lastKnownWaveInTimeRight = millis(); 
      Serial.println(lastKnownWaveInTimeRight);
    }

    if ( (previousFactor - factorRight) > WAVE_DETECT_FACTOR) {
      Serial.print("lastKnownWaveOutTimeRight triggered\t");
      lastKnownWaveOutTimeRight = millis(); 
      Serial.println(lastKnownWaveOutTimeRight);
    }	
  }
  */
  

    if ((factorRightPrevious == 0) && (factorRight > 0)) {
      Serial.print("lastKnownWaveInTimeRight triggered:\t");
      lastKnownWaveInTimeRight = millis(); 
      Serial.println(lastKnownWaveInTimeRight);
    }

    if ( (factorRightPrevious > 0) && (factorRight == 0)) {
      Serial.print("lastKnownWaveOutTimeRight triggered\t");
      lastKnownWaveOutTimeRight = millis(); 
      Serial.println(lastKnownWaveOutTimeRight);
    }	
    

}


