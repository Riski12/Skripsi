
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

const int PatternCount = 75;
const int InputNodes = 4;
const int HiddenNodes = 6;
const int OutputNodes = 4;
const float LearningRate = 0.3;
const float Momentum = 0.9;
const float InitialWeightMax = 0.5;
const float Success = 0.0065;

float Input[PatternCount][InputNodes] = {
  {0.005446167, 0.006284039, 0.098367518, 1},  // R
  {0.003351487, 0.006284039, 1, 0.42228739},  // R
  {0.004189359, 0.005865103, 0.036416911, 0.99790532},  // R
  {0.005446167, 0.004189359, 0.056509, 0.097612065},  // R
  {0, 0.001256808, 0.025115111, 0.064097193},  // R
  {0.005446167, 0.004189359, 1, 0.005446167},  // R
  {0.002513615, 0.005027231, 0.098367518, 0.003351487},  // R
  {0.002932551, 0.006284039, 0.066973629, 0},  // R
  {0.003770423, 0.00209468, 0.048137296, 0.005027231},  // R
  {0.000837872, 0.000837872, 0.031393889, 0.005865103},  // R
  {1, 0.006284039, 0.997488489, 0.95810641},  // R
  {0.097612065, 0.004189359, 1, 0.998743192},  // R
  {0.043150398, 0.00209468, 0.056509, 0.097612065},  // R
  {0.070381232, 0.000837872, 0.077438259, 0.12274822},  // R
  {0.12274822, 0.005027231, 0.098367518, 0.055718475},  // R
  {1, 0.006284039, 0.994139807, 0.001256808},  // R
  {0.11855886, 0.004189359, 0.100460444, 0.005446167},  // R
  {0.055718475, 0.002513615, 0.043951444, 0.000418936},  // R
  {0.082949309, 0.005027231, 0.085809962, 0.002932551},  // R
  {0.03477168, 0.001256808, 0.036416911, 0.003351487},  // R
  {0.872643486, 0.006284039, 0.004185852, 0.999162128},  // L
  {0.149979053, 0.004189359, 0.006278778, 0.139505656},  // L
  {0.100125681, 0.002513615, 0.006697363, 0.066191873},  // L
  {0.035609552, 0.006702974, 0.002092926, 0.085043988},  // L
  {0.045245078, 0.001256808, 0, 0.03477168},  // L
  {0.957268538, 0.006284039, 0.002511511, 0.003351487},  // L
  {0.139505656, 0.004189359, 0.006278778, 0.000418936},  // L
  {0.080854629, 0.00209468, 0.005023022, 0.001675744},  // L
  {0.055718475, 0.001256808, 0, 0.003351487},  // L
  {0.038123167, 0.005446167, 0.004185852, 0.005446167},  // L
  {0.005446167, 0.004189359, 0.002511511, 1}, // B
  {0.000418936, 0, 0.003348681, 0.080854629}, // B
  {0.003351487, 0.006284039, 0.006278778, 0.066191873}, // B
  {0.004608295, 0.00209468, 0.001255756, 0.047339757}, // B
  {0.001675744, 0.005446167, 0.005023022, 0.030582321}, // B
  {0.95810641, 0.999581064, 0.140226036, 0.126937579}, // F
  {0.872643486, 0.140343527, 0.99916283, 0.097612065}, // F
  {0.957268538, 0.915374948, 0.831310172, 0.999162128}, // F
  {0.139505656, 0.12777545, 0.081624111, 0.07666527}, // F
  {0.097612065, 0.077503142, 0.066973629, 0.068286552}, // F
  {0.07666527, 0.111018014, 0.140226036, 0.005446167}, // F
  {0.95810641, 0.957268538, 0.830891586, 0.004608295}, // F
  {0.068286552, 0.098449937, 0.085809962, 0.002513615}, // F
  {0.12274822, 0.141181399, 0.071159481, 0.003351487}, // F
  {0.097612065, 0.071219103, 0.108832147, 0.000418936}, // F
  {0.005446167, 0.958944282, 0.004185852, 0.000418936}, // F
  {0.003351487, 0.140343527, 0.006278778, 0.002513615}, // F
  {0, 0.081692501, 0.005441607, 0.005446167}, // F
  {0.001256808, 0.057813155, 0.001255756, 0.003351487}, // F
  {0.004608295, 0.031420193, 0.002511511, 0.004608295}, // F
  {0.005446167, 0.958944282, 0.914608623, 0.999162128}, // F
  {0.003351487, 0.098449937, 0.123482629, 0.087138668}, // F
  {0.004608295, 0.155006284, 0.085809962, 0.149979053}, // F
  {0.001675744, 0.058651026, 0.04604437, 0.055718475}, // F
  {0, 0.03477168, 0.03557974, 0.032677}, // F
  {0.956849602, 1, 0.006697363, 0.958525346}, // F
  {0.139505656, 0.111018014, 0.005441607, 0.068286552}, // F
  {0.070381232, 0.087138668, 0.004185852, 0.129032258}, // F
  {0.089233347, 0.064935065, 0.002092926, 0.03477168}, // F
  {0.035190616, 0.038961039, 0.00083717, 0.085043988}, // F
  {0.004608295, 0.832425639, 0.003348681, 0.999162128}, // F
  {0.003351487, 0.150816925, 0.006278778, 0.110180142}, // F
  {0.005865103, 0.056556347, 0.005441607, 0.037704231}, // F
  {0.000837872, 0.087138668, 0.002092926, 0.078341014}, // F
  {0.002513615, 0.035609552, 0.001255756, 0.043150398}, // F
  {0.005446167, 0.958944282, 0.99916283, 0}, // F
  {0.003351487, 0.071219103, 0.098367518, 0.002513615}, // F
  {0.001675744, 0.090909091, 0.043951444, 0.005446167}, // F
  {0.004608295, 0.031420193, 0.03474257, 0.004608295}, // F
  {0, 0.120234604, 0.128087066, 0.003351487}, // F
  {0.999162128, 0.832006703, 0.006278778, 0.004608295}, // F
  {0.097612065, 0.111018014, 0.00083717, 0.005446167}, // F
  {0.06954336, 0.092165899, 0.003348681, 0}, // F
  {0.031420193, 0.058651026, 0.002092926, 0.00209468}, // F 
  {0.043150398, 0.050272308, 0.005441607, 0.003351487}, // F    
};

const float Target[PatternCount][OutputNodes] = {
  { 0, 0, 1, 0 },   //kanan
  { 0, 0, 1, 0 },   //kanan
  { 0, 0, 1, 0 },   //kanan
  { 0, 0, 1, 0 },   //kanan
  { 0, 0, 1, 0 },   //kanan
  { 0, 0, 1, 0 },   //kanan
  { 0, 0, 1, 0 },   //kanan
  { 0, 0, 1, 0 },   //kanan
  { 0, 0, 1, 0 },   //kanan
  { 0, 0, 1, 0 },   //kanan
  { 0, 0, 1, 0 },   //kanan
  { 0, 0, 1, 0 },   //kanan
  { 0, 0, 1, 0 },   //kanan
  { 0, 0, 1, 0 },   //kanan
  { 0, 0, 1, 0 },   //kanan
  { 0, 0, 1, 0 },   //kanan
  { 0, 0, 1, 0 },   //kanan
  { 0, 0, 1, 0 },   //kanan
  { 0, 0, 1, 0 },   //kanan
  { 0, 0, 1, 0 },   //kanan
  { 0, 0, 0, 1 },   //kiri
  { 0, 0, 0, 1 },   //kiri
  { 0, 0, 0, 1 },   //kiri
  { 0, 0, 0, 1 },   //kiri
  { 0, 0, 0, 1 },   //kiri
  { 0, 0, 0, 1 },   //kiri
  { 0, 0, 0, 1 },   //kiri
  { 0, 0, 0, 1 },   //kiri
  { 0, 0, 0, 1 },   //kiri
  { 0, 0, 0, 1 },   //kiri
  { 0, 1, 0, 0 },   //mundur
  { 0, 1, 0, 0 },   //mundur
  { 0, 1, 0, 0 },   //mundur
  { 0, 1, 0, 0 },   //mundur
  { 0, 1, 0, 0 },   //mundur
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju
  { 1, 0, 0, 0 },   //maju    
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

float HiddenWeights[InputNodes + 1][HiddenNodes]; // array 2 dimensi 
float OutputWeights[HiddenNodes + 1][OutputNodes];
float HiddenDelta[HiddenNodes];
float OutputDelta[OutputNodes];
float ChangeHiddenWeights[InputNodes + 1][HiddenNodes];
float ChangeOutputWeights[HiddenNodes + 1][OutputNodes];


void setup() {
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
      Serial.println("--Mulai Training-");
      train_nn();
      Serial.println("-Training Selesai-");
    }
    else if (mode == '2') {
      Serial.println("--Mulai Pengujian-");
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
  //Serial.println("Initial/Untrained Outputs: ");
  //toTerminal();
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

      //toTerminal();

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
    cm1 = 25;
    cm2 = 30;
    cm3 = 2404;
    cm4 = 1025;

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

    cm1 = peta(cm1, 17, 2404, 0, 1);
    cm2 = peta(cm2, 15, 2402, 0, 1);
    cm3 = peta(cm3, 15, 2404, 0, 1);
    cm4 = peta(cm4, 17, 2404, 0, 1);

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
    Serial.print(TestInput[0], 3);
    Serial.print("\t");
    Serial.print(TestInput[1], 3);
    Serial.print("\t");
    Serial.print(TestInput[2], 3);
    Serial.print("\t");
    Serial.println(TestInput[3], 3);
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
