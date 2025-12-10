# The RC Car!

This project was completed as a group assignment during my first year at LUT University (2022-2023). I was our team leader. As my first deep dive into electronics and embedded programming, it holds a special place in my portfolio. This was a bold project for freshers and took endless hours at the JHC prototyping lab to finalize, but we managed to pull it off.

The Project: We developed a custom wireless control system for an RC car. While the mechanical chassis was pre-made, we designed and built the controller and all vehicle electronics from scratch. We faced challenges with connection reliability (10–20m range) and occasional throttle sticking, but the result was a functional (and a bit temperamental) RC-car. 

My Role: Beyond leadership, my primary responsibility was designing the controller hardware and writing the firmware for both the transmitter and the receiver. The system utilized Arduino Nano microcontrollers, nRF24L01 transceivers, and an L298N motor driver. I later improved the software to include features like toggling between high and low beam headlights and better visualization for throttle values etc.

Key Lessons: This project was a crash course in hardware debugging. I vividly remember the radio modules refusing to work until we realized the critical importance of decoupling capacitors on the power lines. I also learned a practical lesson in power integrity and ground loops: despite using separate batteries, poor grounding layout caused the headlights to dim whenever the motor drew high current.

The hard work paid off, and our team was honored with the 'Best Project of the Year' award of the course.

<div align="center">
  <img src="../../images/rccar/valmis_projekti.jpg" width="700" />
  <p><em></em></p>
</div>

<div align="center">
  <img src="../../images/rccar/valmis_ohjain_2.jpg" width="600" />
  <p><em></em></p>
</div>

<div align="center">
  <img src="../../images/rccar/ohjain_rakennus_1.png" width="600" />
  <p><em></em></p>
</div>

The codes were modified from https://dronebotworkshop.com/nrf24l01-wireless-joystick/ example project "Wireless Joystick for Arduino Robot Car with nRF24L01+". 

## Source codes

### Controller
```cpp linenums="1"

    /*
    nRF24L01+ Joystick Transmitter
    nrf24l01-joy-xmit-car.ino
    nRF24L01+ Transmitter with Joystick for Robot Car
    Use with Joystick Receiver for Robot Car
    DroneBot Workshop 2018
    https://dronebotworkshop.com
    */

    //Libraries
    #include <RHReliableDatagram.h>
    #include <RH_NRF24.h>
    #include <SPI.h>
    #include <Wire.h>
    #include <MicroLCD.h>

    //Display constants
    LCD_SH1106 lcd;  


    //Joystick Connections
    #define joyVert    A0 
    #define joyHorz    A1
    // Define Joystick Values - Start at 512 (middle position)
    int joyposVert = 512;
    int joyposHorz = 512;
    #define CLIENT_ADDRESS 1   
    #define SERVER_ADDRESS 2
    RH_NRF24 RadioDriver;
    RHReliableDatagram RadioManager(RadioDriver, CLIENT_ADDRESS);
    uint8_t motorcontrol[5]; 
    // motorcontrol[0] = throttle direction
    // motorcontrol[1] = steering direction
    // motorcontrol[2] = throttle speed
    // motorcontrol[3] = steering angle
    // motorcontrol[4] = Headlights on/off
    // Define the Message Buffer

    //Other constants
    #define sw1 4
    #define sw2 6
    uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];

    void setup()
    { 
    // Setup Serial Monitor
    Serial.begin(9600);
    lcd.begin();
    lcd.clear();
    pinMode(sw1, INPUT_PULLUP);
    // Initialize RadioManager with defaults - 2.402 GHz (channel 2), 2Mbps, 0dBm
    if (!RadioManager.init())
        Serial.println("init failed");
    // Set initial motor direction as forward
    motorcontrol[2] = 0;
    // headlights initially off
    motorcontrol[4] = -1;
    }

    void loop()
    {
    // Print to Serial Monitor
    Serial.println("Reading motorcontrol values ");

    motorcontrol[4] = digitalRead(sw1);
    joyposVert = analogRead(joyVert); 
    joyposHorz = analogRead(joyHorz);
    Serial.println(digitalRead(sw1));
    draw(joyposVert);
    draw2(joyposHorz);
    // throttle /////////////////////////////////////////////////
    if (joyposVert < 460)
    {
        motorcontrol[0] = 1;
        motorcontrol[2] = map(joyposVert, 460, 0, 0, 255);
    }
    else if (joyposVert > 564)
    {
        motorcontrol[0] = 0;
        motorcontrol[2] = map(joyposVert, 564, 1023, 0, 255);
    }
    else
    {
        motorcontrol[0] = 0;
        motorcontrol[2] = 0;
    }

    // Steering /////////////////////////////////////////////////

    if (joyposHorz < 460)
    {
        motorcontrol[1] = 1;
        motorcontrol[3] = map(joyposHorz, 460, 0, 0, 255);
    }
    else if (joyposHorz > 564)
    {
        motorcontrol[1] = 0;
        motorcontrol[3] = map(joyposHorz, 564, 1023, 0, 255);
    }
    else
    {
        motorcontrol[1] = 0;
        motorcontrol[3] = 0;
    }

    if(digitalRead(sw1)==LOW){
        motorcontrol[4] = motorcontrol[4]*-1;
    }
    
    //Display the Motor Control values in the serial monitor.
    Serial.print("Motor A: ");
    Serial.print(motorcontrol[0]);
    Serial.print(" - Motor B: ");
    Serial.print(motorcontrol[1]);
    Serial.print(" - Direction: ");
    Serial.println(motorcontrol[2]);
    Serial.print("Headlights: ");
    Serial.println(motorcontrol[4]);
    
    //Send a message containing Motor Control data to manager_server
    if (RadioManager.sendtoWait(motorcontrol, sizeof(motorcontrol), SERVER_ADDRESS))
    {
        // Now wait for a reply from the server
        uint8_t len = sizeof(buf);
        uint8_t from;
        if (RadioManager.recvfromAckTimeout(buf, &len, 2000, &from))
        {
        Serial.print("got reply from : 0x");
        Serial.print(from, HEX);
        Serial.print(": ");
        Serial.println((char*)buf);
        }
        else
        {
        Serial.println("No reply, is nrf24_reliable_datagram_server running?");
        }
    }
    else
        Serial.println("sendtoWait failed");
    delay(5);
    lcd.clear();
    

    }

    void draw(int value){
    lcd.setCursor(0,0);
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.print(value);
    lcd.print("throttle");
    lcd.println();
    }
    void draw2(int value){
    lcd.setCursor(0,5);
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.print(value);
    lcd.print("steering");
    lcd.println();

    }

    void error(){
    lcd.setCursor(0,47);
    lcd.setFontSize(1);
    lcd.println("ERROR!");
    delay(2000);
    lcd.clear();
    }

```

### Car

```cpp linenums="1"
    /*
    nRF24L01+ Joystick Transmitter
    nrf24l01-joy-xmit-car.ino
    nRF24L01+ Transmitter with Joystick for Robot Car
    Use with Joystick Receiver for Robot Car
    DroneBot Workshop 2018
    https://dronebotworkshop.com
    */

    // Include RadioHead ReliableDatagram & NRF24 Libraries
    #include <RHReliableDatagram.h>
    #include <RH_NRF24.h>
    #include <Wire.h>
    #include <Adafruit_GFX.h>
    #include <Adafruit_SH110X.h>
    // Include dependant SPI Library 
    #include <SPI.h>

    // Define Joystick Connections
    #define joyVert    A0 
    #define joyHorz    A1

    // Define Joystick Values - Start at 512 (middle position)
    int joyposVert = 512;
    int joyposHorz = 512;

    // Define addresses for radio channels
    #define CLIENT_ADDRESS 1   
    #define SERVER_ADDRESS 2

    // Create an instance of the radio driver
    RH_NRF24 RadioDriver;

    // Sets the radio driver to NRF24 and the client address to 1
    RHReliableDatagram RadioManager(RadioDriver, CLIENT_ADDRESS);

    // Declare unsigned 8-bit motorcontrol array
    // 2 Bytes for motor speeds plus 1 byte for direction control
    uint8_t motorcontrol[3]; 

    // Define the Message Buffer
    uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];

    //CONSTANTS
    #define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's
    #define SCREEN_WIDTH 128 // OLED display width, in pixels
    #define SCREEN_HEIGHT 64 // OLED display height, in pixels
    #define OLED_RESET -1   //   QT-PY / XIAO
    Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
    display.begin(i2c_Address, true); // Address 0x3C default

    void setup()
    {
    // Setup Serial Monitor
    Serial.begin(9600);
    display.display();
    delay(200);
    display.clearDisplay();
    
    // Initialize RadioManager with defaults - 2.402 GHz (channel 2), 2Mbps, 0dBm
    if (!RadioManager.init())
        Serial.println("init failed");
        
    // Set initial motor direction as forward
    motorcontrol[2] = 0;
    }

    void loop()
    {
    // Print to Serial Monitor
    Serial.println("Reading motorcontrol values ");
    
    // Read the Joystick X and Y positions
    joyposVert = analogRead(joyVert); 
    joyposHorz = analogRead(joyHorz);

    // Determine if this is a forward or backward motion
    // Do this by reading the Verticle Value
    // Apply results to MotorSpeed and to Direction

    if (joyposVert < 460)
    {
        // This is Backward
        // Set Motors backward
        motorcontrol[2] = 1;

        //Determine Motor Speeds
        // As we are going backwards we need to reverse readings
        motorcontrol[0] = map(joyposVert, 460, 0, 0, 255);
        motorcontrol[1] = map(joyposVert, 460, 0, 0, 255);

    }
    else if (joyposVert > 564)
    {
        // This is Forward
        // Set Motors forward
        motorcontrol[2] = 0;

        //Determine Motor Speeds
        motorcontrol[0] = map(joyposVert, 564, 1023, 0, 255);
        motorcontrol[1] = map(joyposVert, 564, 1023, 0, 255); 

    }
    else
    {
        // This is Stopped
        motorcontrol[0] = 0;
        motorcontrol[1] = 0;
        motorcontrol[2] = 0; 

    }
    
    // Now do the steering
    // The Horizontal position will "weigh" the motor speed
    // Values for each motor

    if (joyposHorz < 460)
    {
        // Move Left
        // As we are going left we need to reverse readings
        // Map the number to a value of 255 maximum
        joyposHorz = map(joyposHorz, 460, 0, 0, 255);

        motorcontrol[0] = motorcontrol[0] - joyposHorz;
        motorcontrol[1] = motorcontrol[1] + joyposHorz;

        // Don't exceed range of 0-255 for motor speeds
        if (motorcontrol[0] < 0)motorcontrol[0] = 0;
        if (motorcontrol[1] > 255)motorcontrol[1] = 255;

    }
    else if (joyposHorz > 564)
    {
        // Move Right
        // Map the number to a value of 255 maximum
        joyposHorz = map(joyposHorz, 564, 1023, 0, 255);
    
        motorcontrol[0] = motorcontrol[0] + joyposHorz;
        motorcontrol[1] = motorcontrol[1] - joyposHorz;

        // Don't exceed range of 0-255 for motor speeds
        if (motorcontrol[0] > 255)motorcontrol[0] = 255;
        if (motorcontrol[1] < 0)motorcontrol[1] = 0;      

    }

    // Adjust to prevent "buzzing" at very low speed
    if (motorcontrol[0] < 8)motorcontrol[0] = 0;
    if (motorcontrol[1] < 8)motorcontrol[1] = 0;

    //Display the Motor Control values in the serial monitor.
    Serial.print("Motor A: ");
    Serial.print(motorcontrol[0]);
    Serial.print(" - Motor B: ");
    Serial.print(motorcontrol[1]);
    Serial.print(" - Direction: ");
    Serial.println(motorcontrol[2]);
    
    //Send a message containing Motor Control data to manager_server
    if (RadioManager.sendtoWait(motorcontrol, sizeof(motorcontrol), SERVER_ADDRESS))
    {
        // Now wait for a reply from the server
        uint8_t len = sizeof(buf);
        uint8_t from;
        if (RadioManager.recvfromAckTimeout(buf, &len, 2000, &from))
        {
        Serial.print("got reply from : 0x");
        Serial.print(from, HEX);
        Serial.print(": ");
        Serial.println((char*)buf);
        }
        else
        {
        Serial.println("No reply, is nrf24_reliable_datagram_server running?");
        }
    }
    else
        Serial.println("sendtoWait failed");

    delay(100);  // Wait a bit before next transmission
    }
```