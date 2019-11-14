/************************************************
   Make: Arduino Neural Network Robot
   01/11/2017
   This firmware is comprised of various open source libraries and examples.
   Original elements created by Sean Hodgins
   This firmware is free and open source and can be found here: https://github.com/idlehandsproject/makennbot

   Information on the Neural Network and the code
   can be found here: http://robotics.hobbizine.com/arduinoann.html

   For the OLED screen, the U8G2 library can be found here: https://github.com/olikraus/u8g2

*/

#include <Arduino.h>
#include <math.h>
#define DEBUG
#include <Wire.h>
#define DEBUG

#define EnA A0
#define In1 A1
#define In2 A2
#define EnB A3
#define In3 A4
#define In4 A5
#define trigPin1 2    // Trigger
#define echoPin1 3    // Echo
#define trigPin2  4    // Trigger
#define echoPin2  5    // Echo
#define trigPin3  6    // Trigger
#define echoPin3  7    // Echo
#define trigPin4  8    // Trigger
#define echoPin4 9   // Echo
float duration, cm1, cm2, cm3, cm4;
/******************************************************************
   Network Configuration - customized per network
 ******************************************************************/

const int PatternCount = 16;
const int InputNodes = 4;
const int HiddenNodes = 6;
const int OutputNodes = 4;
const float LearningRate = 0.3;
const float Momentum = 0.9;
const float InitialWeightMax = 0.5;
const float Success = 0.0015;

float Input[PatternCount][InputNodes] = {
  { 0.186046512, 0.029850746, 0.1, 0.217391304 },  // B
  { 0, 0, 0.06, 0.652173913 },  // B
  //  { 0.162790698, 0.074626866, 0.18, 0.108695652 },  // B
  { 0.069767442, 0.179104478, 0.2, 0 },  // B
  { 0.209302326, 0, 0.14, 0.217391304 },  // B
  //  { 0.069767442, 0.089552239, 0.16, 0.043478261 },  // B
  //  { 0.534883721, 0.402985075, 0.54, 0.47826087 },  // F
  //  { 0.720930233, 0.47761194, 0.62, 0.447826087 },  // F
  { 0.76744186, 0.417910448, 0.58, 0.460869565 },  // F
  { 0.418604651, 0.298507463, 0.42, 0.869565217 },  // F
  { 0.88372093, 0.626865672, 0.56, 1 },  // F
  //  { 0.76744186, 0.850746269, 0.9, 0.786956522 },  // F
  { 1, 1, 1, 0.652173913 },  // F
  { 0.186046512, 0.253731343, 0.9, 0.956521739 },  // R
  { 0.069767442, 0.074626866, 0.7, 0.760869565 },  // R
  { 0.023255814, 0.104477612, 0.64, 0.782608696 },  // R
  { 0.23255814, 0, 0.66, 0.739130435 },  // R
  //  { 0.139534884, 0.059701493, 0.7, 0.652173913 },  // R
  { 0.418604651, 0, 0, 0.217391304 },  // L
  { 0.534883721, 0.029850746, 0.06, 0.347826087 },  // L
  { 0.348837209, 0.044776119, 0.1, 0.391304348 },  // L
  { 0.441860465, 0, 0.14, 0.434782609 },  // L
  //  { 0.418604651, 0.029850746, 0.24, 0.304347826 },  // L
  //  { 0.76744186, 0.059701493, 0.3, 0.086956522 },  // L
};

const float Target[PatternCount][OutputNodes] = {
  //  { 0, 1, 0, 0 },   //mundur
  //  { 0, 1, 0, 0 },   //mundur
  { 0, 1, 0, 0 },   //mundur
  { 0, 1, 0, 0 },   //mundur
  { 0, 1, 0, 0 },   //mundur
  { 0, 1, 0, 0 },   //mundur
  //  { 1, 0, 0, 0 },     //Maju
  //  { 1, 0, 0, 0 },     //Maju
  //  { 1, 0, 0, 0 },     //Maju
  { 1, 0, 0, 0 },     //Maju
  { 1, 0, 0, 0 },     //Maju
  { 1, 0, 0, 0 },     //Maju
  { 1, 0, 0, 0 },     //Maju
  //  { 0, 0, 1, 0 },     //kanan
  { 0, 0, 1, 0 },     //kanan
  { 0, 0, 1, 0 },     //kanan
  { 0, 0, 1, 0 },     //kanan
  { 0, 0, 1, 0 },     //kanan
  //  { 0, 0, 0, 1 },     //kiri
  //  { 0, 0, 0, 1 },     //kiri
  { 0, 0, 0, 1 },     //kiri
  { 0, 0, 0, 1 },     //kiri
  { 0, 0, 0, 1 },     //kiri
  { 0, 0, 0, 1 },     //kiri
};

/******************************************************************
   End Network Configuration
 ******************************************************************/

int i, j, p, q, r, mode;
int ReportEvery1000;
int RandomizedIndex[PatternCount];
long  TrainingCycle;
float Rando;
float Error = 2;
float Accum;

float Hidden[HiddenNodes];
float Output[OutputNodes];

float HiddenWeights[InputNodes + 1][HiddenNodes];
float OutputWeights[HiddenNodes + 1][OutputNodes];
float HiddenDelta[HiddenNodes];
float OutputDelta[OutputNodes];
float ChangeHiddenWeights[InputNodes + 1][HiddenNodes];
float ChangeOutputWeights[HiddenNodes + 1][OutputNodes];


void setup() {
  // put your setup code here, to run once:
  //  for (int x = 0; x < 64; x++) {
  //    ErrorGraph[x] = 47;
  //  }
  Serial.begin(115200);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(trigPin4, OUTPUT);
  pinMode(echoPin4, INPUT);

  //fungsi pwm motor driver
  pinMode(EnA, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(EnB, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  randomSeed(analogRead(A6));       //Collect a random ADC sample for Randomization.
  ReportEvery1000 = 1;
  for ( p = 0 ; p < PatternCount ; p++ ) {
    RandomizedIndex[p] = p ;
  }
  Serial.println("Masukkan Perintah:");
  Serial.println("1 = Training");
  Serial.println("2 = Uji");

}


void loop() {

  if (Serial.available() > 0)
  {
    mode = Serial.read();
    Serial.println(mode);
    if (mode == '1') {
      train_nn();
      Serial.println("-------Training Selesai---------");
    }
    else if (mode == '2') {
      Serial.println("-------Mulai Pengujian-------");
      drive_nn();
    }
  }


}


//THIS ROUTINE IS FOR TESTING THE MOTORS

//DRAWS THE MAKE LOGO

//CONTROLS THE MOTORS BASED ON THE BALL LOCATION

//DRIVES MOTOR
void Maju() {
  analogWrite(EnA, 512);
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  analogWrite(EnB, 512);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);

}

void Mundur() {
  analogWrite(EnA, 512);
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  analogWrite(EnB, 512);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
}

void Kanan() {
  analogWrite(EnA, 256);
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  analogWrite(EnB, 512);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
}

void Kiri() {
  analogWrite(EnA, 512);
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  analogWrite(EnB, 256);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
}


//DISPLAYS THE MENU


//TRAINS THE NEURAL NETWORK
void train_nn() {
  /******************************************************************
    Initialize HiddenWeights and ChangeHiddenWeights
  ******************************************************************/
  for ( i = 0 ; i < HiddenNodes ; i++ ) {
    for ( j = 0 ; j <= InputNodes ; j++ ) {
      ChangeHiddenWeights[j][i] = 0.0 ;
      Rando = float(random(100)) / 100;
      HiddenWeights[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;
    }
  }
  /******************************************************************
    Initialize OutputWeights and ChangeOutputWeights
  ******************************************************************/
  for ( i = 0 ; i < OutputNodes ; i ++ ) {
    for ( j = 0 ; j <= HiddenNodes ; j++ ) {
      ChangeOutputWeights[j][i] = 0.0 ;
      Rando = float(random(100)) / 100;
      OutputWeights[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;
    }
  }
  Serial.println("Initial/Untrained Outputs: ");
  toTerminal();
  /******************************************************************
    Begin training
  ******************************************************************/

  for ( TrainingCycle = 1 ; TrainingCycle < 2147483647 ; TrainingCycle++) {

    /******************************************************************
      Randomize order of training patterns
    ******************************************************************/

    for ( p = 0 ; p < PatternCount ; p++) {
      q = random(PatternCount);
      r = RandomizedIndex[p] ;
      RandomizedIndex[p] = RandomizedIndex[q] ;
      RandomizedIndex[q] = r ;
    }
    Error = 0.0 ;
    /******************************************************************
      Cycle through each training pattern in the randomized order
    ******************************************************************/
    for ( q = 0 ; q < PatternCount ; q++ ) {
      p = RandomizedIndex[q];

      /******************************************************************
        Compute hidden layer activations
      ******************************************************************/
      for ( i = 0 ; i < HiddenNodes ; i++ ) {
        Accum = HiddenWeights[InputNodes][i] ;
        for ( j = 0 ; j < InputNodes ; j++ ) {
          Accum += Input[p][j] * HiddenWeights[j][i] ;
        }
        Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ;
      }

      /******************************************************************
        Compute output layer activations and calculate errors
      ******************************************************************/
      for ( i = 0 ; i < OutputNodes ; i++ ) {
        Accum = OutputWeights[HiddenNodes][i] ;
        for ( j = 0 ; j < HiddenNodes ; j++ ) {
          Accum += Hidden[j] * OutputWeights[j][i] ;
        }
        Output[i] = 1.0 / (1.0 + exp(-Accum)) ;
        OutputDelta[i] = (Target[p][i] - Output[i]) * Output[i] * (1.0 - Output[i]) ;
        Error += 0.5 * (Target[p][i] - Output[i]) * (Target[p][i] - Output[i]) ;
      }
      //Serial.println(Output[0]*100);

      /******************************************************************
        Backpropagate errors to hidden layer
      ******************************************************************/
      for ( i = 0 ; i < HiddenNodes ; i++ ) {
        Accum = 0.0 ;
        for ( j = 0 ; j < OutputNodes ; j++ ) {
          Accum += OutputWeights[i][j] * OutputDelta[j] ;
        }
        HiddenDelta[i] = Accum * Hidden[i] * (1.0 - Hidden[i]) ;
      }

      /******************************************************************
        Update Inner-->Hidden Weights
      ******************************************************************/
      for ( i = 0 ; i < HiddenNodes ; i++ ) {
        ChangeHiddenWeights[InputNodes][i] = LearningRate * HiddenDelta[i] + Momentum * ChangeHiddenWeights[InputNodes][i] ;
        HiddenWeights[InputNodes][i] += ChangeHiddenWeights[InputNodes][i] ;
        for ( j = 0 ; j < InputNodes ; j++ ) {
          ChangeHiddenWeights[j][i] = LearningRate * Input[p][j] * HiddenDelta[i] + Momentum * ChangeHiddenWeights[j][i];
          HiddenWeights[j][i] += ChangeHiddenWeights[j][i] ;
        }
      }

      /******************************************************************
        Update Hidden-->Output Weights
      ******************************************************************/
      for ( i = 0 ; i < OutputNodes ; i ++ ) {
        ChangeOutputWeights[HiddenNodes][i] = LearningRate * OutputDelta[i] + Momentum * ChangeOutputWeights[HiddenNodes][i] ;
        OutputWeights[HiddenNodes][i] += ChangeOutputWeights[HiddenNodes][i] ;
        for ( j = 0 ; j < HiddenNodes ; j++ ) {
          ChangeOutputWeights[j][i] = LearningRate * Hidden[j] * OutputDelta[i] + Momentum * ChangeOutputWeights[j][i] ;
          OutputWeights[j][i] += ChangeOutputWeights[j][i] ;
        }
      }
    }

    /******************************************************************
      Every 100 cycles send data to terminal for display and draws the graph on OLED
    ******************************************************************/
    ReportEvery1000 = ReportEvery1000 - 1;
    if (ReportEvery1000 == 0)
    {
      //      int graphNum = TrainingCycle / 100;
      //      int graphE1 = Error * 1000;
      //      int graphE = map(graphE1, 3, 80, 47, 0);
      //      ErrorGraph[graphNum] = graphE;
      //      u8g2.firstPage();
      //      do {
      //        drawGraph();
      //      } while ( u8g2.nextPage());

      Serial.println();
      Serial.println();
      Serial.print ("TrainingCycle: ");
      Serial.print (TrainingCycle);
      Serial.print ("  Error = ");
      Serial.println (Error, 5);
      //      Serial.print ("  Graph Num: ");
      //      Serial.print (graphNum);
      //      Serial.print ("  Graph Error1 = ");
      //      Serial.print (graphE1);
      //      Serial.print ("  Graph Error = ");
      //      Serial.println (graphE);

      toTerminal();

      if (TrainingCycle == 1)
      {
        ReportEvery1000 = 99;
      }
      else
      {
        ReportEvery1000 = 100;
      }
    }


    /******************************************************************
      If error rate is less than pre-determined threshold then end
    ******************************************************************/

    if ( Error < Success ) break ;
  }
}



//USES TRAINED NEURAL NETWORK TO DRIVE ROBOT
void drive_nn()
{
  Serial.println("Running NN Drive Test");
  if (Success < Error) {
    Serial.println("NN not Trained");
  }
  while (Error < Success) {
    float TestInput[] = {0, 0, 0, 0};

    //    //  Sensor ultrasonik 1
    //    digitalWrite(trigPin1, LOW);
    //    delayMicroseconds(5);
    //    digitalWrite(trigPin1, HIGH);
    //    delayMicroseconds(10);
    //    digitalWrite(trigPin1, LOW);
    //    // Read the signal from the sensor: a HIGH pulse whose
    //    // duration is the time (in microseconds) from the sending
    //    // of the ping to the reception of its echo off of an object.
    //    pinMode(echoPin1, INPUT);
    //    duration = pulseIn(echoPin1, HIGH);
    //    // Convert the time into a distance
    //    cm1 = (duration / 2) / 29.1;   // Divide by 29.1 or multiply by 0.0343
    //
    //    //  Sensor ultrasonik 2
    //    digitalWrite(trigPin2, LOW);
    //    delayMicroseconds(5);
    //    digitalWrite(trigPin2, HIGH);
    //    delayMicroseconds(10);
    //    digitalWrite(trigPin2, LOW);
    //    pinMode(echoPin2, INPUT);
    //    duration = pulseIn(echoPin2, HIGH);
    //    // Convert the time into a distance
    //    cm2 = (duration / 2) / 29.1;
    //
    //    //  Sensor ultrasonik 3
    //    digitalWrite(trigPin3, LOW);
    //    delayMicroseconds(5);
    //    digitalWrite(trigPin3, HIGH);
    //    delayMicroseconds(10);
    //    digitalWrite(trigPin3, LOW);
    //    pinMode(echoPin3, INPUT);
    //    duration = pulseIn(echoPin3, HIGH);
    //    // Convert the time into a distance
    //    cm3 = (duration / 2) / 29.1;
    //
    //    //  Sensor ultrasonik 4
    //    digitalWrite(trigPin4, LOW);
    //    delayMicroseconds(5);
    //    digitalWrite(trigPin4, HIGH);
    //    delayMicroseconds(10);
    //    digitalWrite(trigPin4, LOW);
    //    pinMode(echoPin4, INPUT);
    //    duration = pulseIn(echoPin4, HIGH);
    //    // Convert the time into a distance
    //    cm4 = (duration / 2) / 29.1;
    cm1 = 32;
    cm2 = 33;
    cm3 = 35;
    cm4 = 200;

#ifdef DEBUG
    Serial.print("Distance: ");
    Serial.print(cm1);
    Serial.print("\t");
    Serial.print(cm2);
    Serial.print("\t");
    Serial.print(cm3);
    Serial.print("\t");
    Serial.println(cm4);
#endif

    cm1 = peta(cm1, 7, 50, 0, 1);
    cm2 = peta(cm2, 8, 75, 0, 1);
    cm3 = peta(cm3, 5, 55, 0, 1);
    cm4 = peta(cm4, 50, 280, 0, 1);

    Serial.println("Map: ");
    Serial.print("cm1: ");
    Serial.print(cm1);
    Serial.print("cm2: ");
    Serial.print(cm2);
    Serial.print("cm3: ");
    Serial.print(cm3);
    Serial.print("cm4: ");
    Serial.println(cm4);


    cm1 = constrain(cm1, 0, 1);
    cm2 = constrain(cm2, 0, 1);
    cm3 = constrain(cm3, 0, 1);
    cm4 = constrain(cm4, 0, 1);

    Serial.println("Constrain: ");
    Serial.print("cm1: ");
    Serial.print(cm1);
    Serial.print("cm2: ");
    Serial.print(cm2);
    Serial.print("cm3: ");
    Serial.print(cm3);
    Serial.print("cm4: ");
    Serial.println(cm4);

    TestInput[0] = cm1;
    TestInput[1] = cm2;
    TestInput[2] = cm3;
    TestInput[3] = cm4;

#ifdef DEBUG
    Serial.print("Input: ");
    Serial.print(TestInput[0], 2);
    Serial.print("\t");
    Serial.print(TestInput[1], 2);
    Serial.print("\t");
    Serial.print(TestInput[2], 2);
    Serial.print("\t");
    Serial.println(TestInput[3], 2);
#endif

    InputToOutput(TestInput[0], TestInput[1], TestInput[2], TestInput[3]); //INPUT to ANN to obtain OUTPUT

    if (Output[0] > 0.5 ) {
      Maju();
      Serial.println("Maju");
    }
    else if (Output[1] > 0.5) {
      Mundur();
      Serial.println("Mundur");
    }
    else if (Output[2] > 0.5) {
      Kanan();
      Serial.println("Kanan");
    }
    else if (Output[3] > 0.5) {
      Kiri();
      Serial.println("Kiri");
    }
  }
}


//DISPLAYS INFORMATION WHILE TRAINING
void toTerminal()
{

  for ( p = 0 ; p < PatternCount ; p++ ) {
    Serial.println();
    Serial.print ("  Training Pattern: ");
    Serial.println (p);
    Serial.print ("  Input ");
    for ( i = 0 ; i < InputNodes ; i++ ) {
      Serial.print (Input[p][i], DEC);
      Serial.print (" ");
    }
    Serial.print ("  Target ");
    for ( i = 0 ; i < OutputNodes ; i++ ) {
      Serial.print (Target[p][i], DEC);
      Serial.print (" ");
    }
    /******************************************************************
      Compute hidden layer activations
    ******************************************************************/

    for ( i = 0 ; i < HiddenNodes ; i++ ) {
      Accum = HiddenWeights[InputNodes][i] ;
      for ( j = 0 ; j < InputNodes ; j++ ) {
        Accum += Input[p][j] * HiddenWeights[j][i] ;


      }
      Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ;
    }

    /******************************************************************
      Compute output layer activations and calculate errors
    ******************************************************************/

    for ( i = 0 ; i < OutputNodes ; i++ ) {
      Accum = OutputWeights[HiddenNodes][i] ;
      for ( j = 0 ; j < HiddenNodes ; j++ ) {
        Accum += Hidden[j] * OutputWeights[j][i] ;
      }
      Output[i] = 1.0 / (1.0 + exp(-Accum)) ;
    }
    Serial.print ("  Output ");
    for ( i = 0 ; i < OutputNodes ; i++ ) {
      Serial.print (Output[i], 5);
      Serial.print (" ");
    }
  }
}

void InputToOutput(float In1, float In2, float In3, float In4)
{


  float TestInput[] = {0, 0, 0, 0};
  TestInput[0] = In1;
  TestInput[1] = In2;
  TestInput[2] = In3;
  TestInput[3] = In4;

  /******************************************************************
    Compute hidden layer activations
  ******************************************************************/

  for ( i = 0 ; i < HiddenNodes ; i++ ) {
    Accum = HiddenWeights[InputNodes][i] ;
    for ( j = 0 ; j < InputNodes ; j++ ) {
      Accum += TestInput[j] * HiddenWeights[j][i] ;
    }
    Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ;
  }

  /******************************************************************
    Compute output layer activations and calculate errors
  ******************************************************************/

  for ( i = 0 ; i < OutputNodes ; i++ ) {
    Accum = OutputWeights[HiddenNodes][i] ;
    for ( j = 0 ; j < HiddenNodes ; j++ ) {
      Accum += Hidden[j] * OutputWeights[j][i] ;
    }
    Output[i] = 1.0 / (1.0 + exp(-Accum)) ;
  }
#ifdef DEBUG
  Serial.print ("  Output ");
  for ( i = 0 ; i < OutputNodes ; i++ ) {
    Serial.print (Output[i], 5);
    Serial.print (" ");
  }
#endif
}



float peta(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
