
//#include <Arduino.h>
#include <math.h>
#define DEBUG
#include <Wire.h>
#define DEBUG
#include <EEPROM.h>

//Motor Kanan
const int inputPin1  = 14;    // Pin 15 of L293D IC
const int inputPin2  = 15;    // Pin 10 of L293D IC
//Motor Kiri
const int inputPin3  = 16;    // Pin  7 of L293D IC
const int inputPin4  = 17;    // Pin  2 of L293D IC
int EN1 = 10;                 // Pin 1 of L293D IC
int EN2 = 11;                 // Pin 9 of L293D IC
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

const int PatternCount = 90;
const int InputNodes = 4;
const int HiddenNodes = 6;
const int OutputNodes = 4;
const float LearningRate = 0.3;
const float Momentum = 0.9;
const float InitialWeightMax = 0.5;
const float Success = 0.0065;

float Input[PatternCount][InputNodes] = {
  {0.033942559, 0.038961039, 0.61038961, 1}, // R
  {0.020887728, 0.038961039, 1, 1},  // R
  {0.026109661, 0.036363636, 0.225974026, 1},  // R
  {0.033942559, 0.025974026, 0.350649351, 0.608355091},  // R
  {0, 0.007792208, 0.155844156, 0.399477807},  // R
  {0.010443864, 0.012987013, 0.215584416, 0.164490862},  // R

  {0.033942559, 0.025974026, 1, 0.033942559},  // R
  {0.015665796, 0.031168831, 0.61038961, 0.020887728},  // R
  {0.018276762, 0.038961039, 0.415584416, 0},  // R
  {0.023498695, 0.012987013, 0.298701299, 0.031331593},  // R
  {0.005221932, 0.005194805, 0.194805195, 0.036553525},  // R
  {0.007832898, 0.041558442, 0.174025974, 0.010443864},  // R

  {1, 0.038961039, 1, 1},  // R
  {0.608355091, 0.025974026, 1, 1},  // R
  {0.268929504, 0.012987013, 0.350649351, 0.608355091},  // R
  {0.438642298, 0.005194805, 0.480519481, 0.765013055},  // R
  {0.765013055, 0.031168831, 0.61038961, 0.347258486},  // R
  {0.154046997, 0.020779221, 0.194805195, 0.18537859},  // R

  {1, 0.038961039, 1, 0.007832898},  // R
  {0.738903394, 0.025974026, 0.623376623, 0.033942559},  // R
  {0.347258486, 0.015584416, 0.272727273, 0.002610966},  // R
  {0.516971279, 0.031168831, 0.532467532, 0.018276762},  // R
  {0.216710183, 0.007792208, 0.225974026, 0.020887728},  // R
  {0.169712794, 0.041558442, 0.215584416, 0.028720627},  // R

  {1, 0.038961039, 0.025974026, 1},  // L
  {0.934725849, 0.025974026, 0.038961039, 0.869451697},  // L
  {0.624020888, 0.015584416, 0.041558442, 0.412532637},  // L
  {0.221932115, 0.041558442, 0.01038961, 0.53002611},  // L
  {0.281984334, 0.007792208, 0, 0.216710183},  // L
  {0.208877285, 0.031168831, 0.018181818, 0.211488251},  // L

  {1, 0.038961039, 0.015584416, 0.020887728},  // L
  {0.869451697, 0.025974026, 0.038961039, 0.002610966},  // L
  {0.503916449, 0.012987013, 0.031168831, 0.010443864},  // L
  {0.347258486, 0.007792208, 0, 0.020887728},  // L
  {0.237597911, 0.033766234, 0.025974026, 0.033942559},  // L
  {0.208877285, 0.020779221, 0.01038961, 0.026109661},  // L

  {0.033942559, 0.025974026, 0.015584416, 1}, // B
  {0.002610966, 0, 0.020779221, 0.503916449}, // B
  {0.020887728, 0.038961039, 0.041558442, 0.412532637}, // B
  {0.028720627, 0.012987013, 0.007792208, 0.295039164}, // B
  {0.010443864, 0.033766234, 0.031168831, 0.190600522}, // B
  {0.005221932, 0.015584416, 0.007792208, 0.138381201}, // B

  {1, 6.197402597, 0.87012987, 0.791122715}, // F
  {1, 0.87012987, 1, 0.608355091}, // F
  {1, 5.675324675, 1, 1}, // F
  {0.869451697, 0.792207792, 0.506493506, 0.477806789}, // F
  {0.608355091, 0.480519481, 0.415584416, 0.425587467}, // F
  {0.219321149, 0.215584416, 0.192207792, 0.216710183}, // F

  {0.477806789, 0.688311688, 0.87012987, 0.033942559}, // F
  {1, 1, 1, 0.028720627}, // F
  {0.425587467, 0.61038961, 0.532467532, 0.015665796}, // F
  {0.765013055, 0.875324675, 0.441558442, 0.020887728}, // F
  {0.608355091, 0.441558442, 0.675324675, 0.002610966}, // F
  {0.18537859, 0.212987013, 0.207792208, 0.007832898}, // F

  {0.033942559, 1, 0.025974026, 0.002610966}, // F
  {0.020887728, 0.87012987, 0.038961039, 0.015665796}, // F
  {0, 0.506493506, 0.033766234, 0.033942559}, // F
  {0.007832898, 0.358441558, 0.007792208, 0.020887728}, // F
  {0.028720627, 0.194805195, 0.015584416, 0.028720627}, // F
  {0.01305483, 0.223376623, 0.020779221, 0.007832898}, // F

  {0.033942559, 1, 1, 1}, // F
  {0.020887728, 0.61038961, 0.766233766, 0.54308094}, // F
  {0.028720627, 0.961038961, 0.532467532, 0.934725849}, // F
  {0.010443864, 0.363636364, 0.285714286, 0.347258486}, // F
  {0, 0.215584416, 0.220779221, 0.203655352}, // F
  {0.036553525, 0.225974026, 0.246753247, 0.214099217}, // F

  {1, 1, 0.041558442, 1}, // F
  {0.869451697, 0.688311688, 0.033766234, 0.425587467}, // F
  {0.438642298, 0.54025974, 0.025974026, 0.804177546}, // F
  {0.55613577, 0.402597403, 0.012987013, 0.216710183}, // F
  {0.219321149, 0.241558442, 0.005194805, 0.53002611}, // F
  {0.673629243, 0.290909091, 0.018181818, 0.671018277}, // F

  {0.028720627, 1, 0.020779221, 1}, // F
  {0.020887728, 0.935064935, 0.038961039, 0.686684073}, // F
  {0.036553525, 0.350649351, 0.033766234, 0.234986945}, // F
  {0.005221932, 0.54025974, 0.012987013, 0.488250653}, // F
  {0.015665796, 0.220779221, 0.007792208, 0.268929504}, // F
  {0.033942559, 0.215584416, 0.028571429, 0.219321149}, // F

  {0.033942559, 1, 1, 0}, // F
  {0.020887728, 0.441558442, 0.61038961, 0.015665796}, // F
  {0.010443864, 0.563636364, 0.272727273, 0.033942559}, // F
  {0.028720627, 0.194805195, 0.215584416, 0.028720627}, // F
  {0, 0.745454545, 0.794805195, 0.020887728}, // F
  {0.026109661, 0.277922078, 0.212987013, 0.007832898}, // F

  {1, 1, 0.038961039, 0.028720627}, // F
  {0.608355091, 0.688311688, 0.005194805, 0.033942559}, // F
  {0.433420366, 0.571428571, 0.020779221, 0}, // F
  {0.195822454, 0.363636364, 0.012987013, 0.01305483}, // F
  {0.268929504, 0.311688312, 0.033766234, 0.020887728}, // F
  {0.219321149, 0.212987013, 0.028571429, 0.007832898}, // F
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
  { 0, 0, 0, 1 },   //kiri
  { 0, 0, 0, 1 },   //kiri

  { 0, 1, 0, 0 },   //mundur
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
int EEAddress ;   //Location we want the data to be put.


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
  pinMode(EN1, OUTPUT);   // where the motor is connected to
  pinMode(EN2, OUTPUT);   // where the motor is connected to
  pinMode(inputPin1, OUTPUT);
  pinMode(inputPin2, OUTPUT);
  pinMode(inputPin3, OUTPUT);
  pinMode(inputPin4, OUTPUT);
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

  //  int sensor = Serial.parseInt();  //Receive Value from serial monitor
  //  Serial.println(sensor);

  if (Serial.available() > 0)
  {
    mode = Serial.read();
    Serial.println(mode);
    if (mode == '1') {
      EEAddress = 0;
      Serial.println("--Mulai Training-");
      //train nn dan menyimpan weight pada eeprom
      train_nn();
      Serial.println("-Training Selesai-");
      EEPROM.put(EEAddress, HiddenWeights);
      EEAddress += sizeof(HiddenWeights);
      EEPROM.put(EEAddress, OutputWeights);
      Serial.println("Data Telah Disimpan di EEPROM");
      drive_nn();
    }

    else if (mode == '2') {
      EEAddress = 0;
      Serial.println("Load Data from EEPROM");
      EEPROM.get(EEAddress, HiddenWeights);
      for (int i = 0; i < HiddenNodes; i++) {
        for (int j = 0; j < InputNodes; j++) {
          Serial.println( HiddenWeights[j][i], 9 );
        }
      };
      EEAddress += sizeof(HiddenWeights);
      EEPROM.get(EEAddress, OutputWeights);
      for (int i = 0; i < OutputNodes; i++) {
        for (int j = 0; j < HiddenNodes; j++) {
          Serial.println( HiddenWeights[j][i], 9 );
        }
      };
      //      Serial.println(OutputWeights);
      drive_nn();
    }
  }


}


//THIS ROUTINE IS FOR TESTING THE MOTORS
//DRIVES MOTOR
void Maju() {
  analogWrite(EN1, 100);      //sets the motors speed
  analogWrite(EN2, 100);      //sets the motors speed
  digitalWrite(inputPin1, HIGH);
  digitalWrite(inputPin2, LOW);
  digitalWrite(inputPin3, HIGH);
  digitalWrite(inputPin4, LOW);
}

void Mundur() {
  analogWrite(EN1, 100);      //sets the motors speed
  analogWrite(EN2, 100);      //sets the motors speed
  digitalWrite(inputPin1, LOW);
  digitalWrite(inputPin2, HIGH);
  digitalWrite(inputPin3, LOW);
  digitalWrite(inputPin4, HIGH);
}

void Kanan() {
  analogWrite(EN1, 50);      //sets the motors speed
  analogWrite(EN2, 70);      //sets the motors speed
  digitalWrite(inputPin1, LOW);
  digitalWrite(inputPin2, HIGH);
  digitalWrite(inputPin3, HIGH);
  digitalWrite(inputPin4, LOW);
}

void Kiri() {
  analogWrite(EN1, 70);      //sets the motors speed
  analogWrite(EN2, 50);      //sets the motors speed
  digitalWrite(inputPin1, HIGH);
  digitalWrite(inputPin2, LOW);
  digitalWrite(inputPin3, LOW);
  digitalWrite(inputPin4, HIGH);
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
//  if (Success < Error) {
//    Serial.println("NN not Trained");
//  }
  while (true) {
    float TestInput[] = {0, 0, 0, 0};

    //  Sensor ultrasonik 1
    digitalWrite(trigPin1, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin1, LOW);
    // Read the signal from the sensor: a HIGH pulse whose
    // duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    pinMode(echoPin1, INPUT);
    duration = pulseIn(echoPin1, HIGH);
    // Convert the time into a distance
    cm1 = (duration / 2) / 29.1; // Divide by 29.1 or multiply by 0.0343
    //pemotongan range sensor menjadi 400 sesuai spesifikasi
    if(cm1 > 400){
      cm1 = 400;
    };

    //  Sensor ultrasonik 2
    digitalWrite(trigPin2, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin2, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin2, LOW);
    pinMode(echoPin2, INPUT);
    duration = pulseIn(echoPin2, HIGH);
    // Convert the time into a distance
    cm2 = (duration / 2) / 29.1;
    if(cm2 > 400){
      cm2 = 400;
    };

    //  Sensor ultrasonik 3
    digitalWrite(trigPin3, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin3, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin3, LOW);
    pinMode(echoPin3, INPUT);
    duration = pulseIn(echoPin3, HIGH);
    // Convert the time into a distance
    cm3 = (duration / 2) / 29.1;
    if(cm3 > 400){
      cm3 = 400;
    };

    //  Sensor ultrasonik 4
    digitalWrite(trigPin4, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin4, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin4, LOW);
    pinMode(echoPin4, INPUT);
    duration = pulseIn(echoPin4, HIGH);
    // Convert the time into a distance
    cm4 = (duration / 2) / 29.1;
    if(cm4 > 400){
      cm4 = 400;
    };
    //    cm1 = 120;
    //    cm2 = 22;
    //    cm3 = 20;
    //    cm4 = 110;

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

    cm1 = peta(cm1, 17, 400, 0, 1);
    cm2 = peta(cm2, 15, 400, 0, 1);
    cm3 = peta(cm3, 15, 400, 0, 1);
    cm4 = peta(cm4, 17, 400, 0, 1);

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
      Accum += TestInput[j] * HiddenWeights[j][i] ;/*HiddenWeights input-hidden*/
    }
    Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ;
  }

  /******************************************************************
    Compute output layer activations and calculate errors
  ******************************************************************/

  for ( i = 0 ; i < OutputNodes ; i++ ) {
    Accum = OutputWeights[HiddenNodes][i] ;
    for ( j = 0 ; j < HiddenNodes ; j++ ) {
      Accum += Hidden[j] * OutputWeights[j][i] ; /*OutputWeights hidden - output*/
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
