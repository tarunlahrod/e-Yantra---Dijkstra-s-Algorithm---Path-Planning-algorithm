#define LS 10      // left sensor
#define MS 11      // middle sensor
#define RS 12     // right sensor


#define LM1 24       // left motor
#define LM2 25       // left motor
#define RM1 26       // right motor
#define RM2 27       // right motor
//TCS230 or TCS3200 pins wiring to Arduino
#define S0 26
#define S1 27
#define S2 28
#define S3 29
#define sensorOut 30

// Stores frequency read ny the photodiodes
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;


void setup()
{
   pinMode(8, INPUT);//LEFT IR SENSOR
   pinMode(9, INPUT);//MIDDLE IR SENSOR
   pinMode(10,INPUT);//RIGHT IR SENSOR
   pinMode(11,INPUT);//SHARP SENSOR

   pinMode(S0, OUTPUT);
   pinMode(S1, OUTPUT);
   pinMode(S2, OUTPUT);
   pinMode(S3, OUTPUT);
   
   //MOTORS
   pinMode(4, OUTPUT);
   pinMode(5, OUTPUT);
   pinMode(6, OUTPUT);
   pinMode(7, OUTPUT);
   // Setting the sensorOut as input
   pinMode(sensorOut, INPUT);

   // Setting frequency scaling to 20%
   digitalWrite(S0, HIGH);
   digitalWrite(S1,LOW);
}

//**********************************************************
//macros

//this is a false macro with value = 0
#define color_sensor_pulse_count 0

//**********************************************************


//**********************************************************
//prototypes

void Stop();
void soft_left();
void soft_right();
//void  velocity(int, int);

void forward_wls(unsigned char);
void forward();
void back();
void left();
void right();
unsigned int ADC_Conversion(int);
int filter_red(void);
int filter_blue(void);
int filter_green(void);
void filter_clear(void);
void pick(void);
void place(void);
void FF(int);


//**********************************************************
// function definition

void Stop()
{
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, LOW);
}

void soft_left()
{
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
}

void soft_right()
{
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, LOW);
}

/*
void velocity()
{
  
}
*/

void back()
{
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, HIGH);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, HIGH);
}

void forward()
{
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
}

void right()
{
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
}

void left()
{
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, LOW);
}

unsigned int ADC_Conversion(int x)
{
  if( digitalRead(x+7) )
    return 200;
  else
    return 10;
}

int filter_red(void)
{
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  
  // Reading the output frequency
  return pulseIn(sensorOut, LOW);
}
 
int filter_green(void)
{ 
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  
  // Reading the output frequency
  return pulseIn(sensorOut, LOW);
}
 
int filter_blue(void)
{
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  
  // Reading the output frequency
  return pulseIn(sensorOut, LOW);
}


void pick()
{
  delay(2000);
}

void place()
{
  delay(2000);
}

//**********************************************************


unsigned char lm;
unsigned char lm_rw = 1;
unsigned char ltm = 0;
unsigned char obs_front = 0;
float iteration = 0;
enum direction { N, E, S, W };
int maze[24][24][2] = { {  {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{46,N },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}}, //0
            { {-1,-1},{-1,-1},{-1,-1},{88,N },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}}, //1
            { {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{88,N },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}}, //2
            { {-1,-1},{88,S },{-1,-1},{-1,-1},{91,E },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{240,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}}, //3
            { {-1,-1},{-1,-1},{-1,-1},{91,W },{-1,-1},{53,E },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}}, //4
            { {-1,-1},{-1,-1},{-1,-1},{-1,-1},{53,W },{-1,-1},{420,S},{-1,-1},{-1,-1},{-1,-1},{103,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}}, //5
            { {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{420,S},{-1,-1},{53,E },{-1,-1},{-1,-1},{-1,-1},{103,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}}, //6
            { {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{53,W },{-1,-1},{91,E },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}}, //7
            { {-1,-1},{-1,-1},{88,S },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{91,W },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{240,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}}, //8
            { {46,S },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{67,N },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}}, //9
            { {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{103,W},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{107,E},{-1,-1},{-1,-1},{223,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}}, //10
            { {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{103,E},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{107,W},{-1,-1},{-1,-1},{-1,-1},{223,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}}, //11
            { {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{67,S },{107,W},{107,E},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{290,N},{-1,-1},{-1,-1},{-1,-1}}, //12
            { {-1,-1},{-1,-1},{-1,-1},{240,S},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{126,E},{-1,-1},{248,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}}, //13
            { {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{240,S},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{126,W},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{248,N}}, //14
            { {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{223,S},{-1,-1},{-1,-1},{126,W},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{132,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}}, //15
            { {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{223,S},{-1,-1},{-1,-1},{126,E},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{132,N},{-1,-1}}, //16
            { {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{248,S},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{92,E },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}}, //17
            { {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{132,S},{-1,-1},{92,W },{-1,-1},{92,E },{-1,-1},{-1,-1},{-1,-1},{-1,-1}}, //18
            { {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{92,W },{-1,-1},{92,E },{-1,-1},{-1,-1},{-1,-1}}, //19
            { {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{290,S},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{92,W },{-1,-1},{92,E },{-1,-1},{-1,-1}}, //20
            { {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{92,W },{-1,-1},{92,E },{-1,-1}}, //21
            { {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{132,S},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{92,W },{-1,-1},{92,E }}, //22
            { {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{248,S},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{92,W },{-1,-1}}, //23
};
char path[24];
char prev_node = 0;
char curr_node = 0;
char next_node = 0;
char target_node = 0;
char obstruction = 0, obs_new_dir = 0;

int nut_color;
int nuts_picked = 0;
unsigned char occupied[4] = { 0,0,0,0 };

#define confidence_max 20     
#define confidence_thresh 14  
#define clear 0
#define red 1
#define green 2
#define blue 3


unsigned char ir_array(void);
void obstracle(int);
void line_track(void);
void forward_wls(unsigned char);
void left_turn_wls(int);
void right_turn_wls(int);
void FF();
void L(int);
void R(int);
int filter_color();
int pick_nut();
void place_nut();
void generate_path(char, char);
void orient(int);

unsigned char ir_array(void)
{
  unsigned char left_sensor, centre_sensor, right_sensor;                 //variables that store the adc values of the white line sensors
  left_sensor = ADC_Conversion(1);
  centre_sensor = ADC_Conversion(2);
  right_sensor = ADC_Conversion(3);
  return ((left_sensor > 180) * 4 + (centre_sensor > 180) * 2 + (right_sensor > 180));  //converting adc values into a single decimal number based on the binary value
}

void obstracle(int condition)
{
  Stop();
  obstruction = 1;
  if (!condition)
  {
    L(1);
    delay(100);
    obs_new_dir = 1;
  }
  else if (condition == 2)
  {
    delay(100);
    obs_new_dir = 0;
  }
  else
  {
    back();   //  velocity(250, 250);
    delay((int)(iteration * 15) / 2);
    Stop();
    delay(100);
    obs_new_dir = 0;
  }
}


void line_track(void)
{
  int confidence = 0, obs_confidence = 0;
  iteration = 0;
  int ir_local = -1;
  while (1)
  {
    if (ADC_Conversion(4) < 60)
    {
      Stop();
      obs_confidence = 0;
      for (int i = 0; i < 10; i++)
      {
        if (ADC_Conversion(4) < 90)
        {
          obs_confidence++;
        }
        delay(50);
      }
      if (obs_confidence > 5)
      {
        if (iteration > 120)
        {
          obstracle(0);
          break;
        }
        else if (iteration < 10)
        {
          obstracle(2);
          break;
        }
        else
        {
          obstracle(1);
          break;
        }
      }
    }
    delay(50);
    ir_local = ir_array();
    //  velocity(250, 250);
    if (ir_local == 0b010)                      // only centre sensor on black line
    {
      if (lm == 2)                    // line track with right-centre sensors 
      {
        forward(); //  velocity(125, 250);              // curve left
        iteration += 0.75;
      }
      else if (lm == 3)                 // line track with left-centre sensors
      {
        forward(); //  velocity(250, 125);              // curve right
        iteration += 0.75;
      }
      else
      {
        forward();                        // when tracking with all sensors
        iteration += 1;
      }
    }
    else if (ir_local == 0b001)                   // only right sensor on black 
    {
      if (lm == 2)                    // line track with right-centre sensors 
      {
        forward(); //  velocity(250, 125);              // curve right
        iteration += 0.75;
      }
      else
      {
        soft_right();
        iteration += 0.5;
      }
      ltm = 1;                    // store the movement 
    }
    else if (ir_array() == 0b100)                 // only left sensor on black  
    {
      if (lm == 3)                    // line track with left-centre sensors
      {
        forward(); //  velocity(125, 250);              // curve left
        iteration += 0.75;
      }
      else
      {
        soft_left();
        iteration += 0.5;
      }
      ltm = 0;                    // store the movement 
    }
    else if (ir_local == 0b011)                   // right-centre sensor on black 
    {
      if (lm == 3)                    // line track with left-centre sensors
      {
        soft_right();
        iteration += 0.5;
      }
      else
      {
        forward();
        iteration += 1;
      }
      ltm = 1;                    // store the movement 
    }
    else if (ir_local == 0b110)                   // left-centre sensor on black  
    {
      if (lm == 2)                    // line track with right-centre sensors 
      {
        soft_left();
        iteration += 0.5;
      }
      else
      {
        forward();
        iteration += 1;
      }
      ltm = 0;                    // store the movement 
    }
    else if (ir_local == 0b101)                   // rare case of left-right sensors
    {
      if (lm == 0)
      {
        soft_left();                      //lm = 0 => if previous was movement right_turn_wls, right is given priority
        iteration += 0.5;
      }
      else if (lm == 1)
      {
        soft_right();                     //lm = 1 => if previous was movement left_turn_wls, left is given priority
        iteration += 0.5;
      }
      else if (lm == 2)                 // line track with right-centre sensors 
      {
        left();
      }
      else if (lm == 3)                 // line track with left-centre sensors
      {
        right();
      }
    }
    else if (ir_local == 0)                     // out of line
    {
      if (ltm == 0)
      {
        soft_left();
        iteration += 0.5;
      }
      else if (ltm == 1)
      {
        soft_right();
        iteration += 0.5;
      }
    }
    else
    {                               // node detected
      Stop();
      for (int i = 0; i < 10; i++)                // Confirms node detection via software debouncing
      {
        if (ir_array() == 7)
        {
          confidence++;
        }
        delay(50);
      }
      if (confidence > 5)
        break;                          // breaks from loop (eventually function) after confirmation
      confidence = 0;
    }
    delay(50);
  }
}

void forward_wls(unsigned char node)
{
  while (node-- > 0)                  // runs for 'node' number of times (number of nodes)
  {
    if (ir_array() == 0b111)
    {
      forward();  //  velocity(250, 250);
      if (obs_front == 1)
        delay(120);             // to align the centre of the bot close to the node and prevent false detection when there is an independant black line (obstracle) in the front (eg. first node)
      else
        delay(230);
      Stop();
      delay(100);
      Stop();
    }
  }
}
void left_turn_wls(int count)
{
  while (count-- > 0)
  {
    int confidence = 0;                 // counter variable for software debounce
    left();   //  velocity(250, 250);
    delay(150);
    for (int i = 0; i < confidence_max; i++)      // confirms if left sensor is on black line
    {
      if ((ir_array() & 0b100) == 0b100)
        confidence++;
    }
    if (confidence > confidence_thresh)
    {
      left();   //  velocity(250, 250);
      while (1)                   // if left sensor is on black, turn left until it is out of the black line
      {
        confidence = 0;               // resets 'confidence' variable
        for (int i = 0; i < confidence_max; i++)  // confirms if out of line via software debouncing
        {
          if ((ir_array() & 0b100) == 0)
            confidence++;
        }
        if (confidence > confidence_thresh)
          break;                  // confirms and breaks out of the loop
      }
    }
    while (1)
    {
      left();                     // Turn left until left ir sensor detects a black line
      if (lm == 3)
        soft_left();             // soft left at critical left turns (eg. last before node)
      else
        //  velocity(250, 250);
      confidence = 0;
      for (int i = 0; i < confidence_max; i++)    // confirms if black line is detected by left sensor
      {
        if ((ir_array() & 0b100) == 0b100)
          confidence++;
      }
      if (confidence > confidence_thresh)       // confirms and breaks out of the loop
        break;
    }
    Stop();
    delay(100);                   // to stabilize after a turn
    if (lm_rw)
      lm = 0;               //  updates 'lm' to set line track preference to left turn at 0b101 case
  }
}

void right_turn_wls(int count)
{
  while (count-- > 0)
  {
    int confidence = 0;                 // counter variable for software debounce
    right();    //  velocity(250, 250);
    delay(150);
    for (int i = 0; i < confidence_max; i++)      // confirms if right sensor is on black line
    {
      if ((ir_array() & 0b001) == 0b001)
        confidence++;
    }
    if (confidence > confidence_thresh)
    {
      right();
      while (1)                   // if right sensor is on black, turn right until it is out of the black line
      {
        confidence = 0;               // resets 'confidence' variable
        for (int i = 0; i < confidence_max; i++)  // confirms if out of line via software debouncing
        {
          if ((ir_array() & 1) == 0)
            confidence++;
        }
        if (confidence > confidence_thresh)
          break;                  // confirms and breaks out of the loop
      }
    }
    while (1)
    {
      right();    //  velocity(250, 250);       // Turn right until right ir sensor detects a black line
      if (lm == 2)
        soft_right();             // soft right at critical right turns
      else
        //  velocity(250, 250);
      confidence = 0;
      for (int i = 0; i < confidence_max; i++)    // confirms if black line is detected by right sensor
      {
        if ((ir_array() & 1) == 1)
          confidence++;
      }
      if (confidence > confidence_thresh)       // confirms and breaks out of the loop
        break;
    }
    Stop();
    delay(100);                   // to stabilize after a turn
    if (lm_rw)
      lm = 1;               //  updates 'lm' to set line track preference to right turn at 0b101 case
  }
}

void FF()
{
  forward_wls(1);
  line_track();
}

void L(int count)
{
  forward_wls(1);
  left_turn_wls(count);
  line_track();
}

void R(int count)
{
  forward_wls(1);
  right_turn_wls(count);
  line_track();
}

int filter_color()
{
  int r = 0, g = 0, b = 0;                            // variables to store r, g, b values
  r = filter_red();                                   // gets r value
  g = filter_green();
                       // gets g value
  b = filter_blue();
                      // gets b value
  if (r > 3000 && g < 1000 && b < 1000)
    return red;                                 // determines if red
  else if (r < 1000 && g>3000 && b < 1500)
    return green;                               // determines if green
  else if (r > 2000 && r < 4000 && g > 1000 && g < 3000 && b > 500 && b < 2000)
    return blue;                               // determines if blue
  return clear;                                 // determines if none of the above colours
}

int pick_nut()
{
  int confidence, i_c;                                    // for software debouncing
  forward_wls(1);
  if (maze[curr_node][prev_node][1] == E)                         // if approach from East, turn right
  {
    right();  //  velocity(150, 150);
    delay(100);
    right_turn_wls(1);
  }
  else if (maze[curr_node][prev_node][1] == W)                      // if approach from West, turn right
  {
    left();   //  velocity(150, 150);
    delay(100);
    left_turn_wls(1);
  }
  delay(100);
  Stop();

  // picks red nut
  confidence = 0;
  for (i_c = 0; i_c < 10; i_c++)                              // confirms if red is detected
  {
    if (filter_color() == red)
      confidence++;
    delay(50);
  }
  if (confidence > 5)
  {
    pick();
    return red;
  }

  //picks green nut
  confidence = 0;
  for (i_c = 0; i_c < 10; i_c++)                              // confirms if green is detected
  {
    if (filter_color() == green)
      confidence++;
    delay(50);
  }
  if (confidence > 5)
  {
    pick();
    return green;
  }

  confidence = 0;
  for (i_c = 0; i_c < 10; i_c++)
  {
    if (filter_color() == blue)
      confidence++;
    delay(50);
  }
  if (confidence > 5)
  {
    return blue;
  }

  // no object to pick
  return clear;
}

void place_nut()
{
  forward_wls(1);                 // cross the node
  if (maze[curr_node][prev_node][1] == W)     // If approach from West, turn right
  {
    right_turn_wls(1);
  }
  else if (maze[curr_node][prev_node][1] == E)  // If approach from East, turn left
  {
    left_turn_wls(1);
  }Stop();
  place();                    // places the nut in the location
  nuts_picked += 1;               // Increment number of nuts picked
}

void generate_path(char start_node, char end_node)
{
  // 1D array that stores details of the priority_node (node to be branched) to be used in Dijikstra`s algorithm
  int priority_node[3];

  // Acts as priority queue in Dijikstra`s algorithm and stores details of corresponding nodes
  int priority_queue[24][3];

  // Holds the stack of previous priority nodes and helps in creation of final path
  int node_pile[24][3];

  // index in for loop for priority queue, priority queue node, node pool and temporary swap variables
  int i_pq = 0, swap_pq[3], pq_node_repeat, i_np = 0, i_path = 1, swap_path;

  // update target node
  target_node = end_node;

  // priority_node = start_node
  priority_node[0] = start_node;  priority_node[1] = 0; priority_node[2] = 0;

  do
  {
    if ((start_node == 9 && end_node == 0) || start_node == end_node)               // Dead End Cases elimination
    {
      priority_node[1] = start_node;                                // update priority node for initial case, and when start and end nodes are same
      break;
    }

    for (int i = 0; i < 24; i++)                                  // Creating/adding to priority_queue
    {
      if (maze[priority_node[0]][i][0] != -1 && i != priority_node[1])
      {
        pq_node_repeat = 0;
        for (int j = 0; j < i_pq; j++)
          if (priority_queue[j][0] == i)
          {
            pq_node_repeat = 1;
            if (priority_node[2] + maze[priority_node[0]][i][0] < priority_queue[j][2])   // If node number of a branch of priority_node is same as a node in priority_queue, replace the node with lesser weight 
            {
              priority_queue[j][2] = priority_node[2] + maze[priority_node[0]][i][0];
              priority_queue[j][1] = priority_node[0];
            }
            break;
          }
        for (int j = 0; j < i_np; j++)
          if (node_pile[j][0] == i)
          {                                         // If branch node already present in node_pile don`t add it to priority_queue
            pq_node_repeat = 1;
            break;
          }
        if (pq_node_repeat == 0)
        {                                           // Adding of all (excluding above cases) branch nodes of priority_node to priority_queue
          priority_queue[i_pq][0] = i;
          priority_queue[i_pq][1] = priority_node[0];
          priority_queue[i_pq][2] = priority_node[2] + maze[priority_node[0]][i][0];
          i_pq += 1;
        }
      }
    }
    priority_queue[i_pq][0] = -1;                                 // Terminating priority queue with '-1' value

    for (int i = 0; i < i_pq; i++)                                  // Sorting priority_queue
    {
      for (int j = i + 1; j < i_pq; j++)
      {
        if (priority_queue[j][2] < priority_queue[i][2])
        {
          for (int k = 0; k < 3; k++)
          {
            swap_pq[k] = priority_queue[i][k];
            priority_queue[i][k] = priority_queue[j][k];
            priority_queue[j][k] = swap_pq[k];
          }
        }
      }
    }

    for (int i = 0; i < 3; i++)                                   // Add priority_node to node_pile
    {
      node_pile[i_np][i] = priority_node[i];
    }
    i_np += 1;

    for (int i = 0; i < 3; i++)                                   // Update priority_node
    {
      priority_node[i] = priority_queue[0][i];
    }

    for (int i = 1; i < 24 && priority_queue[i][0] != -1; i++)                    // Move priority_queue by 1 step
      for (int k = 0; k < 3; k++)
      {
        priority_queue[i - 1][k] = priority_queue[i][k];
      }
    priority_queue[--i_pq][0] = -1;
  } while (priority_node[0] != end_node);                               // Djikstra`s Algorithm stops only when the end node reaches the top of the priority queue

  // Creation of path matrix by backtracking
  path[0] = end_node;
  path[1] = priority_node[1];

  while (1)
  {
    if (path[1] == start_node)
      break;                                            // break if the start node is same as end node
    if (node_pile[--i_np][0] == path[i_path])
    {
      path[++i_path] = node_pile[i_np][1];
      if (path[i_path] == start_node)
        break;
    }
  }

  for (int i = 0; i <= i_path / 2; i++)                               // Reversing path matrix to ascending order of path
  {
    swap_path = path[i];
    path[i] = path[i_path - i];
    path[i_path - i] = swap_path;
  }
  path[i_path + 1] = -1;                                        // set end of path as -1 // terminating case of path

  if (path[0] == path[1])                                       // if first and second nodes are same in path, remove the second node
  {
    path[1] = -1;
  }
}

void orient(int apr_dir)
{
  do
  {
    obstruction = 0;                                  // no obstruction
    char sd_complete, sd_else;                              // flags for same direction case
    for (int index = 0; path[index + 1] != -1; index++)                 // traverse until generated path is complete
    {
      curr_node = path[index];                            // update current node
      if (index)
        prev_node = path[index - 1];                        // update previous node
      next_node = path[index + 1];                          // update next node

      // target direction
      int tar_dir;

      if (!(apr_dir != -1 && index == 0))
        apr_dir = maze[curr_node][prev_node][1];                  // update approach direction

      tar_dir = maze[curr_node][next_node][1];                    // update target direction

      if (apr_dir - tar_dir == 1 || apr_dir - tar_dir == -3)              // turns right in case of N-W, W-S, S-E, E-N (APPROACH DIRECTION - TARGET DIRECTION)
      {
        R(1);
      }
      else if (apr_dir - tar_dir == -1 || apr_dir - tar_dir == 3)           // turns left in case of N-E, E-S, S-W, W-N (APPROACH DIRECTION - TARGET DIRECTION)
      {
        L(1);
      }
      else if (apr_dir - tar_dir == 2 || apr_dir - tar_dir == -2)           // moves forward in case of N-S, S-N, E-W, W-E (APPROACH DIRECTION - TARGET DIRECTION)
      {
        FF();
      }
      else
      {                                       // case of N-N, S-S, E-E, W-W (APPROACH DIRECTION - TARGET DIRECTION)
        if (apr_dir == N)                             // N-N Case
        {
          sd_complete = 0; sd_else = 1;                     // flag update for same direction // eg if a node to right or left of current facing is present
          for (int j = 0; j < 24; j++)
            if (maze[curr_node][j][1] == W)                   // node present in West             
            {
              sd_complete = 1;
              break;
            }
          if (!sd_complete)                           // no node present in West
          {
            R(1); sd_else = 0;                          // turn right once
          }
          if (sd_complete)
          {
            sd_complete = 0;
            for (int j = 0; j < 24; j++)
              if (maze[curr_node][j][1] == E)                 // node present in East
              {
                sd_complete = 1;
                break;
              }
            if (!sd_complete)                         // no node present in East
            {
              L(1); sd_else = 0;                        // turn left once
            }
          }
          if (sd_else)
            R(2);                               // turn right twice if node present in both East and West 
        }
        else if (apr_dir == E)                            // E-E Case
        {
          sd_complete = 0; sd_else = 1;
          for (int j = 0; j < 24; j++)
            if (maze[curr_node][j][1] == N)                   // node present in North
            {
              sd_complete = 1;
              break;
            }
          if (!sd_complete)
          {
            R(1); sd_else = 0;                          // turn right once
          }
          if (sd_complete)
          {
            sd_complete = 0;
            for (int j = 0; j < 24; j++)
              if (maze[curr_node][j][1] == S)                 // node present in South
              {
                sd_complete = 1;
                break;
              }
            if (!sd_complete)                         // no node present in South
            {
              L(1); sd_else = 0;                        // turn left once
            }
          }
          if (sd_else)
            R(2);                               // turn right twice if node present in both East and West 
        }
        else if (apr_dir == W)                            // W-W Case
        {
          sd_complete = 0; sd_else = 1;
          for (int j = 0; j < 24; j++)
            if (maze[curr_node][j][1] == N)                   // node present in North
            {
              sd_complete = 1;
              break;
            }
          if (!sd_complete)                           // no node present in North
          {
            L(1); sd_else = 0;                          // turn left once
          }
          if (sd_complete)
          {
            sd_complete = 0;
            for (int j = 0; j < 24; j++)
              if (maze[curr_node][j][1] == S)                 // node present in South
              {
                sd_complete = 1;
                break;
              }
            if (!sd_complete)                         // no node present in South
            {
              R(1); sd_else = 0;                        // turn right once
            }
          }
          if (sd_else)
            L(2);                               // turn left twice if node present in both East and West 
        }
        else
        {                                     // S-S Case
          sd_complete = 0; sd_else = 1;
          for (int j = 0; j < 24; j++)
            if (maze[curr_node][j][1] == W)                   // node present in West
            {
              sd_complete = 1;
              break;
            }
          if (!sd_complete)                           // no node present in West
          {
            L(1); sd_else = 0;                          // turn left once
          }
          if (sd_complete)
          {
            sd_complete = 0;
            for (int j = 0; j < 24; j++)
              if (maze[curr_node][j][1] == E)                 // node present in East
              {
                sd_complete = 1;
                break;
              }
            if (!sd_complete)                         // no node present in East
            {
              R(1); sd_else = 0;                        // turn right once
            }
          }
          if (sd_else)
            L(2);                               // turn left twice if node present in both East and West 
        }
      }
      if (obstruction)                                // if obstruction present
      {
        maze[curr_node][next_node][0] = -1; maze[curr_node][next_node][1] = -1;   // block path due to obstruction
        maze[next_node][curr_node][0] = -1; maze[next_node][curr_node][1] = -1;   // block path due to obstruction
        generate_path(curr_node, target_node);                    // generate path
        if (obs_new_dir)
        {
          apr_dir = tar_dir;                            // update approach direction after 180 degree turn
        }
        else
        {
          apr_dir = tar_dir + 2;                          // update approach direction after no turn
          apr_dir %= 4;
        }
        break;                                    // break from the loop 
      }
    }
  } while (obstruction);                                  // repeats until obstruction is present

  if (path[1] != -1)                                    // if end of path, update previous and current
  {
    prev_node = curr_node;
    curr_node = next_node;
  }
}

void loop()
{

  delay(500);
  Stop();                           // Wait for the microcontroller to respond

  int toggle = 0;

  line_track();
  curr_node = 9;
  prev_node = 0;

  for (int i = 17; i <= 23; i++)                // Pick-up zone nodes traversal
  {
    if (i == 20)
      continue;                     // Skip pickup from node 20 due to absence of pickup location
    generate_path(curr_node, i);              // genreates node path from current node to pickup zone node
    if (!toggle)
      orient(-1);                     // traverse to throught he path towards target direction
    else
      orient(N);                      // traverse to throught he path towards north
    toggle = 0;
    nut_color = pick_nut();
    if (nut_color == red)
    {
      if (!occupied[0])
      {
        generate_path(i, 1);              // genreates node path from current node to red pickup zone node
        occupied[0] = 1;                // updates occupancy status
      }
      else
      {
        generate_path(i, 4);              // genreates node path from current node to red pickup zone node
        occupied[1] = 1;                // updates occupancy status
      }
      orient(S);                      // traverse to throught he path towards south
      place_nut();
      toggle = 1;
    }
    else if (nut_color == green)
    {
      if (!occupied[2])
      {
        generate_path(i, 7);              // genreates node path from current node to green pickup zone node
        occupied[2] = 1;                // updates occupancy status
      }
      else
      {
        generate_path(i, 2);              // genreates node path from current node to green pickup zone node
        occupied[3] = 1;                // updates occupancy status
      }
      orient(S);                      // traverse to throught he path towards south
      place_nut();
      toggle = 1;
    }
    else if (nut_color == clear)
    {
      if (i == 23)
        break;                      // break loop if last node
      generate_path(i, i + 1);              // genreates node path from current node to next node in pickup zone
      orient(S);                      // traverse to throught he path towards south
    }
    if (nuts_picked == 4)
      break;
  }

  generate_path(curr_node, 9);                // genreates node path from current node to next 9
  orient(N);                          // traverse to throught he path towards north
  forward();                          // move to start
  delay(1000);
  left(); //  velocity(230, 230);                 // orient as it was at start
  delay(1000);
  Stop();
  delay(3000);
}
