 #include <avr/delay.h>
#include <avr/io.h>
#include <math.h>
#include <stdbool.h>
#include<stdlib.h>
#include<stdio.h>
#include<inttypes.h>
#include "Ultarasonic/DIO/LIB/STD_TYPES.h"
#include "Ultarasonic/Ultrasonic_Interface.h"
#include "Ultarasonic/DIO/DIO_Interface.h"
#include "RC_Controling_interface.h"
#include "ServoMotor.h"


u16 Obstacle;
u16 FLAG;
u16 distance;

f32 ALPHA = 0.2;    //LEARNING RATE
f32 GAMMA = 0.5;    //DISCOUNT FACTOR
f32 EPSILON = 0.90; //EXPLORATION PARAMETER
u16 REWARD;           //REWARD FOR PERFORMING AN ACTION
u16 EPISODES  = 50;

u16 Front_distance;
u16 Left_distance;
u16 Right_distance;
#define distance_MAX 100.0

u16 STATE;                        // CURRENT STATE OF THE ROBOT
u16 ACTION = 0;                   //ACTION PERFORMED BY THE ROBOT(0:FORWARD,1:BACKWARD,2: Right,3:LEFT)
f32 PROB;                       //USED FOR EPSILON DECAY
bool ACTION_TAKEN = false;        //THIS VARIABLES TELLS US WHETHER AN ACTION IS TAKEN OR NOT
u16 NEXT_STATE;                   // NEXT STATE OF THE ROBOT
#define STATES 4           //NUMBER OF STATES IN ENVIRONMENT
#define NUMBER_OF_ACTIONS 4


//State0 : Obstacle front
//State1 : Obstacle front left
//State2 : Obstacle front Right
//State3 : Obstacle front Right left

f32 Q[STATES][NUMBER_OF_ACTIONS] =   {{0.0,0.0,0.0,0.0},  //MOST IMPORTANT OF ALL IS THE Q TABLE.
                                      {0.0,0.0,0.0,0.0},  //IT IS FORMED BY STATES AS ITS  ROWS
                                      {0.0,0.0,0.0,0.0},  //AND COLLUMNS AS ITS NUMBER OF ACTIONS
                                      {0.0,0.0,0.0,0.0}};  //INITIALISED TO ZERO IN THE START


/*u16 REWARDS[STATES][NUMBER_OF_ACTIONS] = {{-10,-2,10,8},
                                          {-10,-2,10,-10},
                                          {-10,-2,-10,10},
                                          {-10,10,-10,-10}};*/


////////////////Q LEARNING UPDATE PARAMETERS////////////
f32 Q_OLD;
f32 Q_NEW;
f32 Q_MAX;
//////////////////////////END//////////////////////////


///////////////////////////////ROBOT'S Q LEARNING FUNCTIONS////////////////////////////////////

f32 RANDOM()
{
  /*THIS FUNCTION FINDS RANDOM NUMBER WHICH
  DECIDES WHETHER AN ACTION TO BE TAKEN IS RANDOM
  OR FROM Q_TABLE*/

  f32 RANDOM_VARIABLE;
  f32 PROBABILITY;

  RANDOM_VARIABLE = rand() % 100;
  PROBABILITY = RANDOM_VARIABLE/100;

  return PROBABILITY;
}

f32 DECAY(f32 PARAMETER)
{
  /*THIS FUNCTION IS USED TO REDUCE
  EPSILON(EXPLORATION PARAMETER) WITH
  TIME.FINALLY AT THE END YOU GET RID
  EPSILON AND THE ROBOT LEARNS TO AVOID
  OBSTACLES ON ITS OWN */

  PARAMETER = PARAMETER*0.98; //PARAMETER HERE IS THE EPSILON
  return PARAMETER;
}

u16 GET_STATE()
{
  bool Obstacle_left = false;
  bool Obstacle_right = false;
  
  set_servo_angel(RIGHT);
  _delay_ms(2000);
  distance=ultarasonic_distance();

  if(distance < 20)
  {
    Obstacle_right = true;
  }
  
  set_servo_angel(LEFT);
   _delay_ms(2000);
  distance=ultarasonic_distance();
   
  if(distance < 20)
  {
    Obstacle_left = true;
  }

  set_servo_angel(CENTER);
   _delay_ms(2000);
   
  if(Obstacle_right && Obstacle_left)
  {
	  return 3; 
  }
  else if(Obstacle_right)
  {
    return 2;
  }
  else if(Obstacle_left)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void MAX(f32 Q_Table[][NUMBER_OF_ACTIONS],int NEXT_S, f32 *MAX_VALUE, u16 *MAX_INDEX)
{
  /*THIS FUNCTION FINDS THE BIGGEST NUMBER
  IN Q_TABLE[NEXT_STATE]. THE MAIN ROLE OF
  THIS FUNCTION IS TO FIND Q_MAX PARAMETER*/

  *MAX_VALUE = Q_Table[NEXT_S][0];
  *MAX_INDEX = 0;
  for(u16 i = 1; i < NUMBER_OF_ACTIONS; i++)
  {
      if(Q_Table[NEXT_S][i] > *MAX_VALUE)
      {
          *MAX_INDEX = i;
          *MAX_VALUE = Q_Table[NEXT_S][i];
      }
  }
}

u16 Obstacle_Avoider()
{
  distance = ultarasonic_distance();

  if(distance<20)
  {
    Obstacle = 1;
  }
  else
  {
    Obstacle = 0;
  }

  _delay_ms(10);
  return Obstacle;
}

void getDis(u16 *FD, u16 *LD, u16 *RD)
{
  /*A function to read the distance calculated for the sensor reading 
  from left, right and forward to use it to update the reward function*/

  _delay_ms(1000);
  *FD = ultarasonic_distance();

  set_servo_angel(RIGHT);
  _delay_ms(2000);
  *RD = ultarasonic_distance();

  set_servo_angel(LEFT);
  _delay_ms(2000);
  *LD = ultarasonic_distance();

  set_servo_angel(CENTER);
   _delay_ms(2000);
}

void Update(f32 Q_TABLE[][NUMBER_OF_ACTIONS] , u16 S, u16 NEXT_S, u16 A, u16 ACTIONS[], u16 R, f32 LEARNING_RATE, f32 DISCOUNT_FACTOR)
{
  /*THIS FUNCTION UPDATES THE Q TABLE AND Q VALUES. THIS UPDATE KEEPS ON HAPPENING UNTILL THE
  MAIN LOOP ENDS. AT THE END OF EPISODES THE Q TABLE IS FILLED WITH VARIOUS VALUES. THE GREATER
  THE VALUES THE GREATER IMPORTANCE THE ACTION HAS AT THAT PARTICULAR STATE. "Q_OLD" IS OLD VALUE
  THAT THE Q MATRIX HAS.THIS IS THE VALUE WHICH GETS UPDATED EVENTUALLY. Q_NEW IS THE NEW Q_VALUE
  WHICH IS CALCULATED BY THE Q LEARNING FORMULA. THE Q LEARNING FORMULA USED HERE IS BASED ON
  BELLMAN EQUATION USES TEMPORAL DIFFERENCE LEARNING APPROACH.(MONTE CARLO APPROACH WILL NOT
  WORK IN THIS CASE OF OBSTACLE AVOIDING ROBOT.*/

  Q_OLD = Q_TABLE[S][A];
  MAX(Q_TABLE, NEXT_S, &Q_MAX, &ACTION);
  Q_NEW = (1-LEARNING_RATE)*Q_OLD + LEARNING_RATE*(R + DISCOUNT_FACTOR*Q_MAX);
  //pru16f(" Q VALUE : %f", Q_NEW);
  Q_TABLE[S][A] = Q_NEW;
}
  //=============================================================================================

 bool EEPROM_is_empty()
 {
	 char A;
	 bool empty=true;

	 for(int i=0; i < 4; i++)
	 {
		 A=eeprom_read_byte((uint8_t*)i);
		 if(A != 0xFF)
		 {
			 empty =false;
			 break;
		 }
	 }

	 return empty;
 }
 //=============================================================================================

 void save_q_table()
 {
	 uint16_t loc = 0;

	 for(int i = 0; i < STATES; i++)
	 {
		 for(int k = 0; k < NUMBER_OF_ACTIONS; k++)
		 {
			 eeprom_write_float((float*) loc, Q[i][k]);
			 loc = loc + 4;
		 }
	 }
 }
 //=============================================================================================
 void retrive_q_table()
 {
	 uint16_t loc = 0;

	 for(int i = 0; i < STATES; i++)
	 {
		 for(int k = 0; k < NUMBER_OF_ACTIONS; k++)
		 {
			 Q[i][k] = eeprom_read_float((float*) loc);
			 loc = loc + 4;
		 }
	 }
 }

/////////////////////////////////////////START OF MAIN LOOP/////////////////////////////////////////////////

//===============================TRAINING===============================//
void loop()
{ 
  for(u16 I =0; I<EPISODES  ; I++)
  {
    //pru16f("\n\nSTART :");
    ACTION_TAKEN = false;
    FLAG = 0;
    while(true)
    {

      forward();

	    Obstacle = Obstacle_Avoider();
      if(Obstacle == 1)
      {
        getDis(&Front_distance, &Left_distance, &Right_distance);
        NEXT_STATE =(STATE+1) % STATES;
        //printf("\nSTATE: %d ",  STATE);
        FLAG = 1;
        break;
      }
    }

    if(FLAG ==1)
    {
      PROB = RANDOM();
      if (PROB<=EPSILON)     //EXPLORE THE ACTIONS
      {
        ACTION = rand() % NUMBER_OF_ACTIONS  ;
      }
      else                  //EXPLOIT THE ACTIONS FROM Q TABLE
      {
        MAX(Q,STATE, &Q_MAX, &ACTION);
      }
      FLAG = 2;
      //printf(" Action : %d", ACTION);
    }

    if(FLAG ==2)
    {
      if(ACTION == 0)
      {
        forward();
        _delay_ms(1000);
        stop();
        REWARD = (1 - (Front_distance/distance_MAX)) * (-10);
      }

      if(ACTION == 1)
      {
        backward();
        _delay_ms(1000);
        stop();
        REWARD = -6;
      }

      if(ACTION == 2)
      {
        right();
        _delay_ms(1500);
        stop();
        REWARD = (1 - (Right_distance/distance_MAX)) * (-10);
      }

      if(ACTION == 3)
      {
        left();
        _delay_ms(1500);
        stop();
        REWARD = (1 - (Left_distance/distance_MAX)) * (-10);
      }

      ACTION_TAKEN = true;
      _delay_ms(500);
    }

    if(ACTION_TAKEN == true)
    {

      Update(Q,STATE,NEXT_STATE,ACTION ,ACTIONS,REWARD,ALPHA ,GAMMA);
      STATE = NEXT_STATE;
      EPSILON = DECAY(EPSILON);
    

    DIO_SetPinValue(PORT2,5,1);
	  _delay_ms(2000);
		DIO_SetPinValue(PORT2,5,0);
		_delay_ms(3000);


    }
  }
  save_q_table();
}
//============================================Testing================================//
void Test()
{
  retrive_q_table();

  while(true)
  {

  forward();
  Obstacle = Obstacle_Avoider();
  if(Obstacle == true)
   {
     stop();
     STATE = GET_STATE();
     MAX(Q,STATE, &Q_MAX, &ACTION);

     if(ACTION ==0)
      {
        forward();
        _delay_ms(1000);

      }

     if(ACTION == 1)
       {
         backward();
        _delay_ms(1000);

       }
     if(ACTION == 2)
       {
         reght();
        _delay_ms(1000);
       }

     if(ACTION == 3)
       {
        left();
        _delay_ms(1000);
       }
     }

  }
}
/////////////////////////////////////END OF TRAINING///////////////////////////////////


//////////////////////////////////////EVALUATION//////////////////////////////////////////
 /*USE THIS TO CHECK WHETHER YOUR Q VALUES ARE RIGHT OR WRONG. IF ALL Q VALUES ARE
 COMING RIGHT OR SEEMS RIGHT/ACCURATE COMMENT THIS SECTION */
 void Eval()
 {
 for(u16 y = 0; y< STATES ; y++)
   {
  //  pru16f("\nSET OF Q VALUES  WILL START:");
    for(u16 l = 0; l < NUMBER_OF_ACTIONS; l++)
      {
     //   printf("\nQ VALUE : %f", Q[y][l]);
        //delay(2000);
      }
      //printf("\n");
     //delay(2000);
   }
  //  printf("\nEVALUATION ENDED");
 }
////////////////////////////////END OF EVALUATION/////////////////////////////////////////

/*
////////////////////////////////////////TESTING////////////////////////////////////////////
while(true)
 {
  Forward();
  Obstacle = Obstacle_Avoider();
  if(Obstacle == true)
   {
     STATE = GET_STATE();
     ACTION = ARGMAX(Q,STATE);
     Serial.print("ACTION TAKEN: ");
     Serial.println(ACTION);

     if(ACTION ==0)
      {
        Forward();
        delay(1500);
        Stop();
      }

     if(ACTION == 1)
       {
        Backward();
        delay(1500);
        Stop();
       }
     if(ACTION == 2)
       {
        Stop();
       }

     if(ACTION == 3)
       {
        Left();
        delay(2000);
        Stop();
       }
     }
  }
  //////////////////////////////////////////////////END OF TESTING////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////END OF MAIN LOOP////////////////////////////////////////////////////////
*/


