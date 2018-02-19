/*
 Name:		ultra_with_pid_9.ino
 Created:	2/11/2016 1:15:33 PM
 Author:	Anthony Brown
*/
//LED PIN
int LED = 13;

//LEFT MOTOR PINS.
int enablepinA = 3;
int logic1A = 2;
int logic2A = 4;
//RIGHT MOTOR PINS
int enablepinB = 5;
int logic3A = 6;
int logic4A = 7;
float W = 13.5;
float halfwidth = 0.5*W;
//ULTRA PINS
short int echoright = 8;
short int trigright = 9;
short int echoleft = 10;
short int trigleft = 11;
short int echoforw = 12;
short int trigforw = 13;

//VARIABLES
short int distanceright;
short int distanceleft;
short int distanceforward;
short int distance;
short int difference;
short int absdiff;
float radius;
float R;
short int setspeed = 200;// using 12 bit PWM resolution that runs from 0 to 4096
short int stallspeed = 10;//12 bit resolution for fine grained control
short int safespeed = 200;
bool needjump = true;
char motionstate = 'S';
char pastmotion;
short int maxwheelspeed = 255;
short int minwheelspeed = 50;
short int RWS;
short int LWS;
short int clearance = 35;
short int crashzone = 14;
short int compensate = 0;// compensation for different motor KV on the wheels. 
//PID VARIABLES
float kP = 1.00;
float kI = 0.25;
float kD = 0.5;
float aggretion;
int previouserror = 0;
int integral = 0;
float P, I, D;

void rotateleft()
{
	motionstate = 'l';
	if ((needjump == true) && (pastmotion != motionstate))
	{
		jumpstart();
	}
	motionstate = 'l';
	RWS = setspeed;
	LWS = 0;
	digitalWrite(enablepinA, LOW);
	analogWrite(enablepinB, RWS);
	digitalWrite(logic3A, HIGH);
	digitalWrite(logic4A, LOW);
	pastmotion = motionstate;
	needjump = true;
}

void rotateright()
{
	motionstate = 'r';
	if ((needjump == true) && (pastmotion != motionstate))
	{
		jumpstart();
	}
	motionstate = 'r';
	LWS = setspeed;
	RWS = 0;
	analogWrite(enablepinA, LWS);
	digitalWrite(enablepinB, LOW);
	digitalWrite(logic1A, HIGH);
	digitalWrite(logic2A, LOW);
	pastmotion = motionstate;
	needjump = true;
}

void curveright(float W, float R)
{
	motionstate = 'R';
	if ((needjump == true) && (pastmotion != motionstate))
	{
		jumpstart();
		motionstate = 'R';
	}
	LWS = setspeed;
	RWS = (LWS*((R - halfwidth) / (R + halfwidth)));
	analogWrite(enablepinA, LWS);
	analogWrite(enablepinB, RWS);
	digitalWrite(logic1A, HIGH);
	digitalWrite(logic2A, LOW);
	digitalWrite(logic3A, HIGH);
	digitalWrite(logic4A, LOW);
	if (RWS<stallspeed)
	{
		needjump = true;
	}
	pastmotion = motionstate;
}

void curveleft(float W, float R)
{
	motionstate = 'L';
	if ((needjump == true) && (pastmotion != motionstate))
	{
		jumpstart();
		motionstate = 'L';
	}
	RWS = setspeed;
	LWS = (RWS*((R - halfwidth) / (R + halfwidth)));
	analogWrite(enablepinA, LWS);
	analogWrite(enablepinB, RWS);
	digitalWrite(logic1A, HIGH);
	digitalWrite(logic2A, LOW);
	digitalWrite(logic3A, HIGH);
	digitalWrite(logic4A, LOW);
	if (LWS<stallspeed)
	{
		needjump = true;
	}
	pastmotion = motionstate;
}

void moveforward()
{
	motionstate = 'F';
	if ((needjump == true) && (pastmotion != motionstate))
	{
		jumpstart();
	}
	RWS = setspeed;
	LWS = setspeed;
	analogWrite(enablepinA, LWS);
	analogWrite(enablepinB, RWS);
	digitalWrite(logic1A, HIGH);
	digitalWrite(logic3A, HIGH);
	digitalWrite(logic2A, LOW);
	digitalWrite(logic4A, LOW);
	pastmotion = motionstate;
}

void reverse()
{
	motionstate = 'B';
	if ((pastmotion != motionstate) && (needjump == true))
	{
		revjumpstart();
	}
	RWS = setspeed;
	LWS = setspeed;
	analogWrite(enablepinA, LWS);
	analogWrite(enablepinB, RWS);
	digitalWrite(logic1A, LOW);
	digitalWrite(logic3A, LOW);
	digitalWrite(logic2A, HIGH);
	digitalWrite(logic4A, HIGH);
	pastmotion = motionstate;
}

//stop
void stationary()
{
	motionstate = 'S';
	digitalWrite(enablepinA, LOW);
	digitalWrite(enablepinB, LOW);
	RWS = 0;
	LWS = 0;
	needjump = true;
	pastmotion = motionstate;
}

//accelarate function
void acc_to(int speed)
{
	if (setspeed<speed)
	{
		setspeed += 30;
	}
}

//decelarate function
void decc_to(int speed)
{
	if (setspeed>speed)
	{
		setspeed -= 30;
	}
}

//speed controller
void speedcontroller()
{
	if (distanceforward<30)
	{
		decc_to(safespeed);
	}
}

//rotations to time seconds depending on setspeed and desired rotations
int rot_delay(float rotations)
{
	double k = 0.0007324;
	double T = rotations / (setspeed*k);
	return (T * 1000);
}

void delayrotations(float rots)
{
	delay(rot_delay(rots));
}

//jump start to overcome initial static friction of motors both forward and reverse
void jumpstart()
{
	unsigned long jumptime = 15;
	digitalWrite(enablepinA, HIGH);
	digitalWrite(enablepinB, HIGH);
	digitalWrite(logic1A, HIGH);
	digitalWrite(logic3A, HIGH);
	digitalWrite(logic2A, LOW);
	digitalWrite(logic4A, LOW);
	delay(jumptime);
	needjump = false;
	moveforward();
	Serial.println("jumpstarted");
}

void revjumpstart()
{
	unsigned long jumptime = 15;
	digitalWrite(enablepinA, HIGH);
	digitalWrite(enablepinB, HIGH);
	digitalWrite(logic1A, LOW);
	digitalWrite(logic3A, LOW);
	digitalWrite(logic2A, HIGH);
	digitalWrite(logic4A, HIGH);
	delay(jumptime);
	needjump = false;
	reverse();
	Serial.println("jumpstarted reverse");
}

//manuever to get the rear wheel unhooked from a corener.
void maneuverX(char side)
{
	stationary();
	reverse();
	delayrotations(0.1); // delay for a number of rotations
	stationary();
	if (side == 'R')
	{
		rotateleft();
		delayrotations(0.2);//the speed should affect how long to delay
		moveforward();
		delayrotations(0.1);
		//rotateright();
		//delayrotations(0.3);
		stationary();
	}
	else
	{
		rotateright();
		delayrotations(0.2);
		moveforward();
		delayrotations(0.1);
		//rotateleft();
		//delayrotations(0.3);
		stationary();
	}
}

// edge clip function
void edgeclip()
{
	if (distanceright <= 4)
	{
		maneuverX('R');
	}
	else if (distanceleft <= 4)
	{
		maneuverX('L');
	}
}

// back up a little, bit too close to the wall.
void backup()
{
	reverse();
	delayrotations(0.1);
	stationary();
}

//get distance function declaration and definition
int getdistance(int trigpin, int echopin)
{
	digitalWrite(trigpin, LOW);
	delayMicroseconds(2);
	digitalWrite(trigpin, HIGH);
	delayMicroseconds(10);
	digitalWrite(trigpin, LOW);
	unsigned long duration = pulseIn(echopin, HIGH, 6060);
	distance = round(duration / 58.2);
	if( distance == 0)
		return crashzone+1;
	else
		return distance;
}

//mapping function using floats
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min)*(out_max - out_min) / (in_max - in_min) + out_min;
}
// sign test function
int signtest(int val)
{
	int sign;
	if (val < 0)
	{
		sign = -1;
	}
	else if (val>0)
	{
		sign = 1;
	}
	else
	{
		sign = 0;
	}
	return sign;
}
// control kp 
void dynamic_kp()
{
	kP = (54/ distanceforward);
	if (kP < 0.5)
	{
		kP = 0.5;
	}
}

//pid curver using difference
float pidcurver(int error)
{
	dynamic_kp();
	int integralthresh = 100; //integral threshold to prevent integral wind up
	if (signtest(previouserror) != signtest(error))
	{
		integral = 0; //reset integral if there's a change in motion.
		integral = error;
	}
	else if (abs(integral) >= integralthresh)
	{
		//integral = 0;
		//integral = error;
		integral = integral; //prevent integral from growing larger than the threshold 
	}
	else
	{
		integral += error;
	}
	P = (kP*error);  //Proportional 
	I = (kI*integral);   //Integral
	D = (kD*(previouserror - error));//Derivative
	aggretion = P + I + D;
	if (aggretion == 0)
	{
		R = 100;
	}
	if (abs(aggretion)>50)
	{
		R = 6.75;
	}
	else if ((abs(aggretion)) < 0)
	{
		R = 100;
	}
	else
	{
		R = mapf(abs(aggretion), 0, 50, 100, 6.75);
	}
	previouserror = error;
	return R;
}


void setup()
{
	pinMode(enablepinA, OUTPUT);
	pinMode(enablepinB, OUTPUT);
	pinMode(logic1A, OUTPUT);
	pinMode(logic3A, OUTPUT);
	pinMode(logic2A, OUTPUT);
	pinMode(logic4A, OUTPUT);

	pinMode(trigright, OUTPUT);
	pinMode(trigleft, OUTPUT);
	pinMode(echoleft, INPUT);
	pinMode(echoright, INPUT);
	pinMode(trigforw, OUTPUT);
	pinMode(echoforw, INPUT);
	Serial.begin(9600);
	//analogWriteResolution(12);
	delay(5000);
	jumpstart();
}

void loop()
{
LOOP:
	unsigned long loopB = millis();
	//fetch ultrasound distances
	distanceleft = getdistance(trigleft, echoleft);
	distanceforward = getdistance(trigforw, echoforw);
	distanceright = getdistance(trigright, echoright);
	difference = (distanceleft - distanceright) + compensate; //compensation factor if needed
	absdiff = abs(difference);

	if (distanceforward <= crashzone)//impossible navigation space , special manuvers required
	{
		stationary();
		if (absdiff > 3) // not at finish line
		{
			backup();
		}
	}
	/*else if ((distanceforward <= clearance) && (distanceforward > crashzone)) // small navigation space either dead end or rotate needed
	{
		if (absdiff > 2)
		{
			if (distanceleft > distanceright)
			{
				rotateleft();
				delayrotations(rottime);
			}
			else //if (distanceright > distanceleft)
			{
				rotateright();
				delayrotations(rottime); //hold the rotate abit to get through some of the turn 
			}
		}
		else
		{
			stationary();
		}
	}*/
	else //if (distanceforward > clearance) // fair distance to use usuall radius
	{
		edgeclip(); // check if stuck on wall edge
		if (difference >= 2)
		{
			pidcurver(difference);
			curveleft(W, R);
		}
		else if (difference <= -2)
		{
			pidcurver(difference);
			curveright(W, R);
		}
		else //if (absdiff<2)
		{
			moveforward();
		}
	}
	Serial.print("distances:");
	Serial.print(distanceleft);
	Serial.print("/");
	Serial.print(distanceright);
	Serial.print("\tf");
	Serial.print(distanceforward);
	Serial.print("\tdifference");
	Serial.print(difference);
	Serial.print("\t");
	Serial.print(motionstate);
	Serial.print("\tpid=");
	Serial.print(P);
	Serial.print("/");
	Serial.print(I);
	Serial.print("/");
	Serial.print(D);
	Serial.print("\taggretion");
	Serial.print(aggretion);
	Serial.print("\tradius");
	Serial.print(R);
	Serial.print("\t");
	Serial.print(LWS);
	Serial.print("/");
	Serial.print(RWS);
	Serial.print("\tloop time=");
	unsigned long loopE = millis();
	unsigned long loopt = (loopE - loopB);
	Serial.println(loopt);
}

