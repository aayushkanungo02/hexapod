#include <SoftwareSerial.h>      // We use software serial on pins D12 and D8 for HC06
SoftwareSerial Bluetooth(12, 9); // Arduino RX 12 and TX 9 -> HC-06 Bluetooth TX RX
#include <Servo.h>               //We will use servo library

//Variables
int Received = 0;             //here we store the received byte number from Bluetooth
int DELAY = 5;                //Delay in ms for the main loop, the bigger, the slower will the robot move
int MODE = 0;                 //Start mode is 0, so all motors -> home position
bool Impair_start = false;    //In order to add phase rotation between legs, we use this variable
int FM1 = 0;                  //The leg movement loop has different stages
int FM2 = 0;                  //Each of these variables will be used in those stages
int FM3 = 0;                  //We use these as counters to count degrees
int FM4 = 0;                  //...
int FM5 = 0;                  //...
int FM6 = 0;                  //...
int FM7 = 0;                  //...
int FM8 = 0;                  //...


/*/////////////////Initial home position in degrees//////////////////
The first leg is the front left leg as seen from the front
Thew first motor of each leg is the one closer to the robot body*/
int home_Leg1_Mot1 = 75; 
int home_Leg1_Mot2 = 90; 
int home_Leg1_Mot3 = 100;

int home_Leg2_Mot1 = 70;
int home_Leg2_Mot2 = 90;
int home_Leg2_Mot3 = 115;

int home_Leg3_Mot1 = 70;
int home_Leg3_Mot2 = 90;
int home_Leg3_Mot3 = 100;

int home_Leg4_Mot1 = 105;
int home_Leg4_Mot2 = 90;
int home_Leg4_Mot3 = 110;

int home_Leg5_Mot1 = 75;
int home_Leg5_Mot2 = 90;
int home_Leg5_Mot3 = 90;

int home_Leg6_Mot1 = 110;
int home_Leg6_Mot2 = 100;
int home_Leg6_Mot3 = 115;
/*/////////////////////////////////////////////////////////////////*/


/*//////////////////////Declare the Servos/////////////////////////*/
Servo Leg1_Mot1, Leg1_Mot2, Leg1_Mot3;    //Motors 1,2 and 3 of leg 1
Servo Leg2_Mot1, Leg2_Mot2, Leg2_Mot3;    //Motors 1,2 and 3 of leg 2 
Servo Leg3_Mot1, Leg3_Mot2, Leg3_Mot3;    //Motors 1,2 and 3 of leg 3 
Servo Leg4_Mot1, Leg4_Mot2, Leg4_Mot3;    //Motors 1,2 and 3 of leg 4 
Servo Leg5_Mot1, Leg5_Mot2, Leg5_Mot3;    //Motors 1,2 and 3 of leg 5 
Servo Leg6_Mot1, Leg6_Mot2, Leg6_Mot3;    //Motors 1,2 and 3 of leg 6 
int LED = 17;
int bat = A1;
/*/////////////////////////////////////////////////////////////////*/ 


void setup() {
  Serial.begin(9600);     //We use the Arduino monitopr for debug...
  Bluetooth.begin(9600);  // My default baud rate of the HC06, yours can be different
  pinMode(LED, OUTPUT);
  pinMode(bat, INPUT);
  digitalWrite(LED, LOW);
  
  //Select digital pins of the MEGA for each motro
  Leg1_Mot1.attach(35);
  Leg1_Mot2.attach(37);
  Leg1_Mot3.attach(39);

  Leg2_Mot1.attach(29);
  Leg2_Mot2.attach(31);
  Leg2_Mot3.attach(33);

  Leg3_Mot1.attach(34);
  Leg3_Mot2.attach(36);
  Leg3_Mot3.attach(38);

  Leg4_Mot1.attach(26);
  Leg4_Mot2.attach(24);
  Leg4_Mot3.attach(22);

  Leg5_Mot1.attach(32);
  Leg5_Mot2.attach(30);
  Leg5_Mot3.attach(28);

  Leg6_Mot1.attach(27);
  Leg6_Mot2.attach(25);
  Leg6_Mot3.attach(23);
  /*/////////////////////////////////////////////////////////////////*/ 
  
  set_home_pos();       //Home all motors
}

void loop() {
  /*Battery is 12.6V fully charged. We want to enable the LED when battery is below 8.5V.
  Divider is 1k/3K so 0.33 * 8.5 = 2.83 volts -> in 10 bits = 580
  */
  if(analogRead(bat) < 580)
  {
    digitalWrite(LED, HIGH);
  }
  else
  {
    digitalWrite(LED, LOW);
  }

  
  if(Bluetooth.available()>0)           //If we receive somethin, we enter this loop
  {
    Received = Bluetooth.read();        //We store the received byte number
    //Serial.println(Received);         //Print for debug...
    if (Received == 0)                  //If we receive the "0" number, mode = 1
    {
      MODE = 0;               //Change mode to 0 (home position)
      Impair_start = false;   //Reset variables   
      FM1 = 0;                //Reset values...
      FM2 = 0;                //Reset values...
      FM3 = 0;                //Reset values...
      FM4 = 0;                //Reset values...
      FM5 = 0;                //Reset values...
      FM6 = 0;                //Reset values...
      FM7 = 0;                //Reset values...
      FM8 = 0;                //Reset values...
    }
    if (Received == 1)        //If we receive a 1, change mode to 1 and so on...
    {
      MODE = 1;
    }
    if (Received == 2)
    {
      MODE = 2;
    }
    if (Received == 3)
    {
      MODE = 3;
    }
    if (Received == 4)
    {
      MODE = 4;
    }
    if (Received == 5)
    {
      MODE = 5;
    }
    if (Received == 6)
    {
      MODE = 6;
    }
    /*If the received number is higher than 11, then we receive the values from slider
    this will change the speed of movement by changing the delay of the loop*/
    if (Received >= 12)
    {
      DELAY = map(Received,15,100,5,80); //Map delay from 5 to 80 ms
    }
   }

  

  //Home position
  if(MODE == 0){
    set_home_pos();
  }

  //Move Forward
  if(MODE == 1){
    move_frwd();
  }

  //Move Backwards
  if(MODE == 2){
    move_bwd();
  }
  
  //Move Right
  if(MODE == 3){
    move_right();
  }
   
  //Move Left
  if(MODE == 4){
    move_left();
  }

  //Rotate Left
  if(MODE == 5){
    rotate_left();
  }

  //Rotate Right
  if(MODE == 6){
    rotate_right();
  }
 
delay(DELAY);         //Add delay of the loop, this will control the speed
}



/////////////////////////////SET HOME POSITION
void set_home_pos()
{
  Leg1_Mot1.write(home_Leg1_Mot1); 
  Leg1_Mot2.write(home_Leg1_Mot2); 
  Leg1_Mot3.write(home_Leg1_Mot3); 

  Leg2_Mot1.write(home_Leg2_Mot1); 
  Leg2_Mot2.write(home_Leg2_Mot2); 
  Leg2_Mot3.write(home_Leg2_Mot3); 

  Leg3_Mot1.write(home_Leg3_Mot1); 
  Leg3_Mot2.write(home_Leg3_Mot2); 
  Leg3_Mot3.write(home_Leg3_Mot3); 

  Leg4_Mot1.write(home_Leg4_Mot1); 
  Leg4_Mot2.write(home_Leg4_Mot2); 
  Leg4_Mot3.write(home_Leg4_Mot3); 

  Leg5_Mot1.write(home_Leg5_Mot1); 
  Leg5_Mot2.write(home_Leg5_Mot2); 
  Leg5_Mot3.write(home_Leg5_Mot3); 

  Leg6_Mot1.write(home_Leg6_Mot1); 
  Leg6_Mot2.write(home_Leg6_Mot2); 
  Leg6_Mot3.write(home_Leg6_Mot3); 
}




/////////////////////////////MOVE FORWARD
void move_frwd(){
  //Impair Lift 10 deg
  if (FM1 <=10){
    Leg1_Mot2.write(home_Leg1_Mot2 - FM1); 
    Leg1_Mot3.write(home_Leg1_Mot3 - FM1);
    Leg3_Mot2.write(home_Leg3_Mot2 - FM1); 
    Leg3_Mot3.write(home_Leg3_Mot3 - FM1);
    Leg5_Mot2.write(home_Leg5_Mot2 - FM1); 
    Leg5_Mot3.write(home_Leg5_Mot3 - FM1);
    FM1++;
  }  
      
  //Impair rote fwd 30 deg
  if (FM2 <= 30)
  {
    Leg1_Mot1.write(home_Leg1_Mot1 - FM2); 
    Leg3_Mot1.write(home_Leg3_Mot1 - FM2); 
    Leg5_Mot1.write(home_Leg5_Mot1 + FM2); 
    FM2++;
  }
  
  //Impair touch ground -10 deg
  if (FM2 > 20 && FM3 <=10){
    Leg1_Mot2.write(home_Leg1_Mot2 + FM3); 
    Leg1_Mot3.write(home_Leg1_Mot3 + FM3);
    Leg3_Mot2.write(home_Leg3_Mot2 + FM3); 
    Leg3_Mot3.write(home_Leg3_Mot3 + FM3);
    Leg5_Mot2.write(home_Leg5_Mot2 + FM3); 
    Leg5_Mot3.write(home_Leg5_Mot3 + FM3);
    FM3++;
  } 
  
  //Impair rotate bwd -30 deg  
  if (FM2 >= 30)
  {
    Leg1_Mot1.write(home_Leg1_Mot1 + FM4); 
    Leg3_Mot1.write(home_Leg3_Mot1 + FM4); 
    Leg5_Mot1.write(home_Leg5_Mot1 - FM4); 
    FM4++;
    Impair_start = true;      
  }
  if(FM4 >= 30) {
    FM1 = 0;
    FM2 = 0;
    FM3 = 0;
    FM4 = 0;
  }
  
//////////////////////////////////

  if (Impair_start){
    //Pair Lift 10 deg
    if (FM5 <=10){
      Leg2_Mot2.write(home_Leg2_Mot2 - FM5); 
      Leg2_Mot3.write(home_Leg2_Mot3 - FM5);
      Leg4_Mot2.write(home_Leg4_Mot2 - FM5); 
      Leg4_Mot3.write(home_Leg4_Mot3 - FM5);
      Leg6_Mot2.write(home_Leg6_Mot2 - FM5); 
      Leg6_Mot3.write(home_Leg6_Mot3 - FM5);
      FM5++;
    }  
    //Pair rote fwd 30 deg
    if (FM6 <= 30)
    {
      Leg2_Mot1.write(home_Leg2_Mot1 - FM6); 
      Leg4_Mot1.write(home_Leg4_Mot1 + FM6); 
      Leg6_Mot1.write(home_Leg6_Mot1 + FM6); 
      FM6++;
    }
  
    //Pair touch ground -10 deg
    if (FM6 > 20 && FM7 <=10){
      Leg2_Mot2.write(home_Leg2_Mot2 + FM7); 
      Leg2_Mot3.write(home_Leg2_Mot3 + FM7);
      Leg4_Mot2.write(home_Leg4_Mot2 + FM7); 
      Leg4_Mot3.write(home_Leg4_Mot3 + FM7);
      Leg6_Mot2.write(home_Leg6_Mot2 + FM7); 
      Leg6_Mot3.write(home_Leg6_Mot3 + FM7);
      FM7++;
    } 
  
    //Pair rotate bwd -30 deg  
    if (FM6 >= 30)
    {
      Leg2_Mot1.write(home_Leg2_Mot1 + FM8); 
      Leg4_Mot1.write(home_Leg4_Mot1 - FM8); 
      Leg6_Mot1.write(home_Leg6_Mot1 - FM8); 
      FM8++;      
    }
    if(FM8 >= 30) {
      Impair_start = false;
      FM5 = 0;
      FM6 = 0;
      FM7 = 0;
      FM8 = 0;
    }    
  } 
}//End Move Forward


/////////////////////////////MOVE Backwards
void move_bwd(){ 
  //Impair Lift 10 deg
  if (FM1 <=10){
    Leg1_Mot2.write(home_Leg1_Mot2 - FM1); 
    Leg1_Mot3.write(home_Leg1_Mot3 - FM1);
    Leg3_Mot2.write(home_Leg3_Mot2 - FM1); 
    Leg3_Mot3.write(home_Leg3_Mot3 - FM1);
    Leg5_Mot2.write(home_Leg5_Mot2 - FM1); 
    Leg5_Mot3.write(home_Leg5_Mot3 - FM1);
    FM1++;
  }  
    
  //Impair rote fwd 30 deg
  if (FM2 <= 30)
  {
    Leg1_Mot1.write(home_Leg1_Mot1 + FM2); 
    Leg3_Mot1.write(home_Leg3_Mot1 + FM2); 
    Leg5_Mot1.write(home_Leg5_Mot1 - FM2); 
    FM2++;
  }
  
  //Impair touch ground -10 deg
  if (FM2 > 20 && FM3 <=10){
    Leg1_Mot2.write(home_Leg1_Mot2 + FM3); 
    Leg1_Mot3.write(home_Leg1_Mot3 + FM3);
    Leg3_Mot2.write(home_Leg3_Mot2 + FM3); 
    Leg3_Mot3.write(home_Leg3_Mot3 + FM3);
    Leg5_Mot2.write(home_Leg5_Mot2 + FM3); 
    Leg5_Mot3.write(home_Leg5_Mot3 + FM3);
    FM3++;
  } 
  
  //Impair rotate bwd -30 deg  
  if (FM2 >= 30)
  {
    Leg1_Mot1.write(home_Leg1_Mot1 - FM4); 
    Leg3_Mot1.write(home_Leg3_Mot1 - FM4); 
    Leg5_Mot1.write(home_Leg5_Mot1 + FM4); 
    FM4++;
    Impair_start = true;
  }
  if(FM4 >= 30) {
    FM1 = 0;
    FM2 = 0;
    FM3 = 0;
    FM4 = 0;
  }
  
//////////////////////////////////

  if (Impair_start){
    //Pair Lift 10 deg
    if (FM5 <=10){
      Leg2_Mot2.write(home_Leg2_Mot2 - FM5); 
      Leg2_Mot3.write(home_Leg2_Mot3 - FM5);
      Leg4_Mot2.write(home_Leg4_Mot2 - FM5); 
      Leg4_Mot3.write(home_Leg4_Mot3 - FM5);
      Leg6_Mot2.write(home_Leg6_Mot2 - FM5); 
      Leg6_Mot3.write(home_Leg6_Mot3 - FM5);
      FM5++;
    }  
    //Pair rote fwd 30 deg
    if (FM6 <= 30)
    {
      Leg2_Mot1.write(home_Leg2_Mot1 + FM6); 
      Leg4_Mot1.write(home_Leg4_Mot1 - FM6); 
      Leg6_Mot1.write(home_Leg6_Mot1 - FM6); 
      FM6++;
    }
  
    //Pair touch ground -10 deg
    if (FM6 > 20 && FM7 <=10){
      Leg2_Mot2.write(home_Leg2_Mot2 + FM7); 
      Leg2_Mot3.write(home_Leg2_Mot3 + FM7);
      Leg4_Mot2.write(home_Leg4_Mot2 + FM7); 
      Leg4_Mot3.write(home_Leg4_Mot3 + FM7);
      Leg6_Mot2.write(home_Leg6_Mot2 + FM7); 
      Leg6_Mot3.write(home_Leg6_Mot3 + FM7);
      FM7++;
    } 
  
    //Pair rotate bwd -30 deg  
    if (FM6 >= 30)
    {
      Leg2_Mot1.write(home_Leg2_Mot1 - FM8); 
      Leg4_Mot1.write(home_Leg4_Mot1 + FM8); 
      Leg6_Mot1.write(home_Leg6_Mot1 + FM8); 
      FM8++;      
    }
    if(FM8 >= 30) {
      FM5 = 0;
      FM6 = 0;
      FM7 = 0;
      FM8 = 0;
    }    
  } 
}//End Move Backwards



/////////////////////////////Rotate Left
void rotate_left(){
  //Impair Lift 10 deg
  if (FM1 <=10){
    Leg1_Mot2.write(home_Leg1_Mot2 - FM1); 
    Leg1_Mot3.write(home_Leg1_Mot3 - FM1);
    Leg3_Mot2.write(home_Leg3_Mot2 - FM1); 
    Leg3_Mot3.write(home_Leg3_Mot3 - FM1);
    Leg5_Mot2.write(home_Leg5_Mot2 - FM1); 
    Leg5_Mot3.write(home_Leg5_Mot3 - FM1);
    FM1++;
  }  
      
  //Impair rote fwd 30 deg
  if (FM2 <= 30)
  {
    Leg1_Mot1.write(home_Leg1_Mot1 - FM2); 
    Leg3_Mot1.write(home_Leg3_Mot1 - FM2); 
    Leg5_Mot1.write(home_Leg5_Mot1 - FM2); 
    FM2++;
  }
  
  //Impair touch ground -10 deg
  if (FM2 > 20 && FM3 <=10){
    Leg1_Mot2.write(home_Leg1_Mot2 + FM3); 
    Leg1_Mot3.write(home_Leg1_Mot3 + FM3);
    Leg3_Mot2.write(home_Leg3_Mot2 + FM3); 
    Leg3_Mot3.write(home_Leg3_Mot3 + FM3);
    Leg5_Mot2.write(home_Leg5_Mot2 + FM3); 
    Leg5_Mot3.write(home_Leg5_Mot3 + FM3);
    FM3++;
  } 
  
  //Impair rotate bwd -30 deg  
  if (FM2 >= 30)
  {
    Leg1_Mot1.write(home_Leg1_Mot1 + FM4); 
    Leg3_Mot1.write(home_Leg3_Mot1 + FM4); 
    Leg5_Mot1.write(home_Leg5_Mot1 + FM4); 
    FM4++;
    Impair_start = true;      
  }
  if(FM4 >= 30) {
    FM1 = 0;
    FM2 = 0;
    FM3 = 0;
    FM4 = 0;
  }
  
//////////////////////////////////

  if (Impair_start){
    //Pair Lift 10 deg
    if (FM5 <=10){
      Leg2_Mot2.write(home_Leg2_Mot2 - FM5); 
      Leg2_Mot3.write(home_Leg2_Mot3 - FM5);
      Leg4_Mot2.write(home_Leg4_Mot2 - FM5); 
      Leg4_Mot3.write(home_Leg4_Mot3 - FM5);
      Leg6_Mot2.write(home_Leg6_Mot2 - FM5); 
      Leg6_Mot3.write(home_Leg6_Mot3 - FM5);
      FM5++;
    }  
    //Pair rote fwd 30 deg
    if (FM6 <= 30)
    {
      Leg2_Mot1.write(home_Leg2_Mot1 - FM6); 
      Leg4_Mot1.write(home_Leg4_Mot1 - FM6); 
      Leg6_Mot1.write(home_Leg6_Mot1 - FM6); 
      FM6++;
    }
  
    //Pair touch ground -10 deg
    if (FM6 > 20 && FM7 <=10){
      Leg2_Mot2.write(home_Leg2_Mot2 + FM7); 
      Leg2_Mot3.write(home_Leg2_Mot3 + FM7);
      Leg4_Mot2.write(home_Leg4_Mot2 + FM7); 
      Leg4_Mot3.write(home_Leg4_Mot3 + FM7);
      Leg6_Mot2.write(home_Leg6_Mot2 + FM7); 
      Leg6_Mot3.write(home_Leg6_Mot3 + FM7);
      FM7++;
    } 
  
    //Pair rotate bwd -30 deg  
    if (FM6 >= 30)
    {
      Leg2_Mot1.write(home_Leg2_Mot1 + FM8); 
      Leg4_Mot1.write(home_Leg4_Mot1 + FM8); 
      Leg6_Mot1.write(home_Leg6_Mot1 + FM8); 
      FM8++;      
    }
    if(FM8 >= 30) {
      Impair_start = false;
      FM5 = 0;
      FM6 = 0;
      FM7 = 0;
      FM8 = 0;
    }    
  } 
}//End Rotate Left


/////////////////////////////Rotate Right
void rotate_right(){ 
  //Impair Lift 10 deg
  if (FM1 <=10){
    Leg1_Mot2.write(home_Leg1_Mot2 - FM1); 
    Leg1_Mot3.write(home_Leg1_Mot3 - FM1);
    Leg3_Mot2.write(home_Leg3_Mot2 - FM1); 
    Leg3_Mot3.write(home_Leg3_Mot3 - FM1);
    Leg5_Mot2.write(home_Leg5_Mot2 - FM1); 
    Leg5_Mot3.write(home_Leg5_Mot3 - FM1);
    FM1++;
  }  
    
  //Impair rote fwd 30 deg
  if (FM2 <= 30)
  {
    Leg1_Mot1.write(home_Leg1_Mot1 + FM2); 
    Leg3_Mot1.write(home_Leg3_Mot1 + FM2); 
    Leg5_Mot1.write(home_Leg5_Mot1 + FM2); 
    FM2++;
  }
  
  //Impair touch ground -10 deg
  if (FM2 > 20 && FM3 <=10){
    Leg1_Mot2.write(home_Leg1_Mot2 + FM3); 
    Leg1_Mot3.write(home_Leg1_Mot3 + FM3);
    Leg3_Mot2.write(home_Leg3_Mot2 + FM3); 
    Leg3_Mot3.write(home_Leg3_Mot3 + FM3);
    Leg5_Mot2.write(home_Leg5_Mot2 + FM3); 
    Leg5_Mot3.write(home_Leg5_Mot3 + FM3);
    FM3++;
  } 
  
  //Impair rotate bwd -30 deg  
  if (FM2 >= 30)
  {
    Leg1_Mot1.write(home_Leg1_Mot1 - FM4); 
    Leg3_Mot1.write(home_Leg3_Mot1 - FM4); 
    Leg5_Mot1.write(home_Leg5_Mot1 - FM4); 
    FM4++;
    Impair_start = true;
  }
  if(FM4 >= 30) {
    FM1 = 0;
    FM2 = 0;
    FM3 = 0;
    FM4 = 0;
  }
  
//////////////////////////////////

  if (Impair_start){
    //Pair Lift 10 deg
    if (FM5 <=10){
      Leg2_Mot2.write(home_Leg2_Mot2 - FM5); 
      Leg2_Mot3.write(home_Leg2_Mot3 - FM5);
      Leg4_Mot2.write(home_Leg4_Mot2 - FM5); 
      Leg4_Mot3.write(home_Leg4_Mot3 - FM5);
      Leg6_Mot2.write(home_Leg6_Mot2 - FM5); 
      Leg6_Mot3.write(home_Leg6_Mot3 - FM5);
      FM5++;
    }  
    //Pair rote fwd 30 deg
    if (FM6 <= 30)
    {
      Leg2_Mot1.write(home_Leg2_Mot1 + FM6); 
      Leg4_Mot1.write(home_Leg4_Mot1 + FM6); 
      Leg6_Mot1.write(home_Leg6_Mot1 + FM6); 
      FM6++;
    }
  
    //Pair touch ground -10 deg
    if (FM6 > 20 && FM7 <=10){
      Leg2_Mot2.write(home_Leg2_Mot2 + FM7); 
      Leg2_Mot3.write(home_Leg2_Mot3 + FM7);
      Leg4_Mot2.write(home_Leg4_Mot2 + FM7); 
      Leg4_Mot3.write(home_Leg4_Mot3 + FM7);
      Leg6_Mot2.write(home_Leg6_Mot2 + FM7); 
      Leg6_Mot3.write(home_Leg6_Mot3 + FM7);
      FM7++;
    } 
  
    //Pair rotate bwd -30 deg  
    if (FM6 >= 30)
    {
      Leg2_Mot1.write(home_Leg2_Mot1 - FM8); 
      Leg4_Mot1.write(home_Leg4_Mot1 - FM8); 
      Leg6_Mot1.write(home_Leg6_Mot1 - FM8); 
      FM8++;      
    }
    if(FM8 >= 30) {
      FM5 = 0;
      FM6 = 0;
      FM7 = 0;
      FM8 = 0;
    }    
  } 
}//End Rotate Right


/////////////////////////////Move Right
void move_right(){
  //Impair Motor 2 -20 Motor 3 +20
  if (FM1 <= 20){    
    Leg1_Mot2.write(home_Leg1_Mot2 + FM1); 
    Leg1_Mot3.write(home_Leg1_Mot3 + FM1); 

    Leg2_Mot2.write(home_Leg2_Mot2 + FM1); 
    Leg2_Mot3.write(home_Leg2_Mot3 - FM1*2);
        
    Leg3_Mot2.write(home_Leg3_Mot2 + FM1);     
    Leg3_Mot3.write(home_Leg3_Mot3 + FM1); 

    Leg4_Mot2.write(home_Leg4_Mot2 - FM1); 
    Leg4_Mot3.write(home_Leg4_Mot3 + FM1*2);
    
    Leg5_Mot2.write(home_Leg5_Mot2 + FM1);     
    Leg5_Mot3.write(home_Leg5_Mot3 - FM1); 
  
    Leg6_Mot2.write(home_Leg6_Mot2 - FM1); 
    Leg6_Mot3.write(home_Leg6_Mot3 + FM1*2);  
    FM1++;
  }  

  if(FM1 >= 20 && FM2 <= 20){
     Leg2_Mot2.write(home_Leg2_Mot2 + FM1   + FM2); 
     Leg2_Mot3.write(home_Leg2_Mot3 - FM1*2 + FM2*2);

     Leg4_Mot2.write(home_Leg4_Mot2 - FM1   + FM2); 
     Leg4_Mot3.write(home_Leg4_Mot3 + FM1*2 - FM2*2);

     Leg6_Mot2.write(home_Leg6_Mot2 - FM1   + FM2); 
     Leg6_Mot3.write(home_Leg6_Mot3 + FM1*2 - FM2*2);
     FM2++;
  }

   if(FM2 >= 20 && FM3 <= 20){
    Leg1_Mot2.write(home_Leg1_Mot2 + FM1 - FM3); 
    Leg1_Mot3.write(home_Leg1_Mot3 + FM1 - FM1); 
   
            
    Leg3_Mot2.write(home_Leg3_Mot2 + FM1 - FM3);     
    Leg3_Mot3.write(home_Leg3_Mot3 + FM1 - FM3); 

    
    Leg5_Mot2.write(home_Leg5_Mot2 + FM1 - FM3);     
    Leg5_Mot3.write(home_Leg5_Mot3 - FM1 + FM3); 
  
    FM3++;
  }

  if(FM3 >= 20){
    FM1 = 0;
    FM2 = 0;
    FM3 = 0;
  }

}




/////////////////////////////Move Left
void move_left(){
  //Impair Motor 2 -20 Motor 3 +20
  if (FM1 <= 20){    
    Leg4_Mot2.write(home_Leg4_Mot2 + FM1); 
    Leg4_Mot3.write(home_Leg4_Mot3 + FM1); 

    Leg5_Mot2.write(home_Leg5_Mot2 + FM1); 
    Leg5_Mot3.write(home_Leg5_Mot3 - FM1*2);
        
    Leg6_Mot2.write(home_Leg6_Mot2 + FM1);     
    Leg6_Mot3.write(home_Leg6_Mot3 + FM1); 

    Leg3_Mot2.write(home_Leg4_Mot3 - FM1); 
    Leg3_Mot3.write(home_Leg4_Mot3 + FM1*2);
    
    Leg2_Mot2.write(home_Leg2_Mot2 + FM1);     
    Leg2_Mot3.write(home_Leg2_Mot3 - FM1); 
  
    Leg1_Mot2.write(home_Leg1_Mot2 - FM1); 
    Leg1_Mot3.write(home_Leg1_Mot3 + FM1*2);  
    FM1++;
  }  

  if(FM1 >= 20 && FM2 <= 20){
     Leg5_Mot2.write(home_Leg5_Mot2 + FM1   + FM2); 
     Leg5_Mot3.write(home_Leg5_Mot3 - FM1*2 + FM2*2);

     Leg3_Mot2.write(home_Leg3_Mot2 - FM1   + FM2); 
     Leg3_Mot3.write(home_Leg3_Mot3 + FM1*2 - FM2*2);

     Leg1_Mot2.write(home_Leg1_Mot2 - FM1   + FM2); 
     Leg1_Mot3.write(home_Leg1_Mot3 + FM1*2 - FM2*2);
     FM2++;
  }

   if(FM2 >= 20 && FM3 <= 20){
    Leg6_Mot2.write(home_Leg6_Mot2 + FM1 - FM3); 
    Leg6_Mot3.write(home_Leg6_Mot3 + FM1 - FM1); 
   
            
    Leg4_Mot2.write(home_Leg4_Mot2 + FM1 - FM3);     
    Leg4_Mot3.write(home_Leg4_Mot3 + FM1 - FM3); 

    
    Leg2_Mot2.write(home_Leg2_Mot2 + FM1 - FM3);     
    Leg2_Mot3.write(home_Leg2_Mot3 - FM1 + FM3); 
  
    FM3++;
  }
  if(FM3 >= 20){
    FM1 = 0;
    FM2 = 0;
    FM3 = 0;
  }
}
