#include <Wire.h>
#include <HMC5883L.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

/*
Rev: ver 8.0 - Bluetooth Connection.
Rev: ver 9.0 - GPS coordinates transmitted during movement
*/

boolean magnetic_calibrated, GPS_init, FINISHED = false;
HMC5883L compass;
int error = 0;
int i, X0, Y0;
unsigned long t;
double a, b;
int minX, minY, maxX, maxY;
int offsetX, offsetY;
float headingN;
unsigned long hareket_suresi;
int PATH_COUNT;


double headingGPS; //******************************
double Ncur, Ecur; // Current GPS Coordinates
double Ntar, Etar; // Target GPS coordinates



int PWMA = 3;
int PWMB = 11;
int BRAKEA = 9;
int BRAKEB = 8;
int DIRA = 12;
int DIRB = 13;

// GPS Koordinatlari
//TERAS TESTI
float Epath[4] = {28.831895021, 28.832006672, 28.832084907, 28.831974501}  ;
float Npath[4] = {40.991090506, 40.991155691, 40.991101564, 40.991025399};
int numPaths=4;

// OTOPARK PATH_1
//float Epath[2] = {28.831474028, 28.831829263};
//float Npath[2] = {40.990020532, 40.989937291};
//int numPaths = 2;

//OTOPARK PATH_2
//float Epath[5] = {28.832761877,28.832947833,28.832900368,28.832708105,28.832731514};
//float Npath[5] = {40.989519816,40.989477027,40.989336370,40.989378567,40.989460199 };
//int numPaths=5;

struct TargetPair {
  double heading;
  float distance;
};

struct TargetPair GlobalPair;

float longave ;
float latave;

static const int GPS_RXPin = 0, GPS_TXPin = 1;
static const int GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial ss(GPS_RXPin, GPS_TXPin);
LiquidCrystal lcd(10, 7, 6, 5, 4, 2);
MagnetometerRaw raw;



void setup() {
  PATH_COUNT = 0;
  delay(5000);
  Wire.begin();
  Serial.begin(9600);
  compass = HMC5883L();
  error = compass.SetScale(1.3);
  error = compass.SetMeasurementMode(Measurement_Continuous);
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);
  pinMode(BRAKEA, OUTPUT);
  pinMode(BRAKEB, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  ss.begin(GPSBaud);

  longave = 0;
  latave = 0;
  minX = 1000;
  minY = 1000;
  maxX = -1000;
  maxY = -1000;
  Serial.println("GUVE ver. 9.0");
  lcdBasla();
  Serial.println("Starting Device...");
}



void loop() {
  t = millis();
  // ---------------------------START OF MAGNETIC CALIBRATION ------------------------
  if (magnetic_calibrated == false)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MAGNETOMETER");
    lcd.setCursor(0, 1);
    lcd.print("CALIBRATION");

    Serial.println("Magnetometer Calibration Started...");

    int count_offset = 0;
    a = 0;
    b = 0;
    digitalWrite(DIRA, LOW);
    digitalWrite(DIRB, LOW);
    digitalWrite(BRAKEA, LOW);
    digitalWrite(BRAKEB, LOW);
    analogWrite(PWMA, 255);
    analogWrite(PWMB, 255);
    while ((millis() - t ) < 10000) // 10 saniye saga donus
    {
      raw = compass.ReadRawAxis();
      a = a + raw.XAxis;
      b = b + raw.YAxis;
      if ((raw.XAxis) < minX & (abs(raw.XAxis) < 1002))
        minX = raw.XAxis;
      if ((raw.XAxis > maxX) & (abs(raw.XAxis) < 1002))
        maxX = raw.XAxis;
      if ((raw.YAxis < minY)  & (abs(raw.YAxis) < 1002))
        minY = raw.YAxis;
      if ((raw.YAxis > maxY ) & (abs(raw.YAxis) < 1002))
        maxY = raw.YAxis;
      count_offset++;
      delay(10);
    }
    magnetic_calibrated = true;

    Serial.println("Finished...");

    lcd.print("..OK.");
    a = a / (float) count_offset;
    b = b / (float) count_offset;
    offsetX = (floor)((minX + maxX) / (float) 2);
    offsetY = (floor)((minY + maxY) / (float) 2);
    Y0 = -(raw.YAxis - offsetY);
    X0 = raw.XAxis - offsetX;
    headingN = atan(absFloat(Y0) / (float) absFloat(X0)) * 180 / ( (float) PI);

    if (X0 > 0)
    {
      if (Y0 > 0)
      {
        headingN = headingN;
      }
      else
      {
        //Serial.println("N>0, E<0");
        headingN = 360 - headingN;
      }
    }
    else
    {
      if (Y0 > 0)
      {
        headingN = 180 - headingN;
      }
      else
      {
        headingN = 180 + headingN;

      }

    }
    headingN = headingN + 15;
    if (headingN < 0)
      headingN += 360;
    if (headingN > 360)
      headingN -= 360;

    digitalWrite(BRAKEA, HIGH);
    digitalWrite(BRAKEB, HIGH);
    delay(2000);
    lcd.clear();
    lcd.setCursor(0, 0);

    lcd.print("OffsetX: ");
    lcd.print(offsetX);
    lcd.setCursor(0, 1);
    lcd.print("OffsetY: ");
    lcd.print(offsetY);
    delay(2000);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("X0: ");
    lcd.print(X0);
    lcd.setCursor(0, 1);
    lcd.print("Y0: ");
    lcd.print(Y0);

    Serial.print("Offset X: ");
    Serial.print(X0);
    Serial.print(",OFFSET Y: ");
    Serial.println(Y0);

    delay(2000);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Heading:");
    lcd.print(headingN);
    Serial.print("North Heading: ");
    Serial.println(headingN, 2);

    delay(2000);
  } // ---------------------------ENF OF MAGNETIC CALIBRATION ------------------------

  //               ---------------- INITIAL COORDINATES ------------------
  t = millis();
  if (GPS_init == false) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Getting GPS");
    lcd.setCursor(0, 1);
    lcd.print("Coordinates...");

    Serial.println("Getting GPS Coordinates...");
    Serial.println("Longitude, Latitude");
    while ((millis() - t ) < 60000) {

      while (ss.available() > 0)
        if (gps.encode(ss.read()))
        {

          longave += gps.location.lng();
          latave += gps.location.lat();

          Serial.print(gps.location.lng(), 8);
          Serial.print(",");
          Serial.println(gps.location.lat(), 8);
          i++;


        }
      if (millis() > 5000 && gps.charsProcessed() < 10)
      {
        //Serial.println(F("No GPS detected: check wiring."));
        while (true);
      }
    }

    GPS_init = true;
    Ecur = (double) (longave / (float) i);
    Ncur = (double) (latave / (float) i);
  }
  GPSinfo(Ncur, Ecur);
  delay(3000);

  //               ----------------END OF INITIAL COORDINATES ------------------



  if (!FINISHED)
  {

    for (PATH_COUNT = 0; PATH_COUNT < numPaths ; PATH_COUNT++)
    {
      digitalWrite(DIRA, LOW);
      digitalWrite(DIRB, LOW);
      analogWrite(PWMA, 200);
      analogWrite(PWMB, 200);
      digitalWrite(BRAKEA, LOW);
      digitalWrite(BRAKEB, LOW);
      if (PATH_COUNT == 0)
        GlobalPair = calPair(Ncur, Ecur, Npath[0], Epath[0]);
      else
        GlobalPair = calPair(Npath[PATH_COUNT - 1], Epath[PATH_COUNT - 1], Npath[PATH_COUNT], Epath[PATH_COUNT]);

      Serial.print("Target Heading: ");
      Serial.println(GlobalPair.heading);
      Serial.print("Target Distance: ");
      Serial.print(GlobalPair.distance);

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Target Heading: ");
      lcd.setCursor(0, 1);
      lcd.print(GlobalPair.heading);
      delay(2000);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Target Distance: ");
      lcd.setCursor(0, 1);
      lcd.print(GlobalPair.distance);
      delay(3000);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("ROTATING...");
      lcd.setCursor(0, 1);
      Serial.println("Heading Adjustment...");

      delay(1000);


      while (abs(headingN - GlobalPair.heading) > 2)

      {
        raw = compass.ReadRawAxis();
        //headingN = 180 - (atan2((raw.YAxis - offsetY), (raw.XAxis - offsetX)) * 180 / PI);
        Y0 = -(raw.YAxis - offsetY);
        X0 = raw.XAxis - offsetX;
        delay(10);
        headingN = atan(absFloat(Y0) / (float) absFloat(X0)) * 180 / ( (float) PI);

        if (X0 > 0)
        {
          if (Y0 > 0)
          {
            headingN = headingN;
          }
          else
          {
            //Serial.println("N>0, E<0");
            headingN = 360 - headingN;
          }
        }
        else
        {
          if (Y0 > 0)
          {
            headingN = 180 - headingN;
          }
          else
          {
            headingN = 180 + headingN;

          }
        }
        headingN = headingN + 15;
        if (headingN < 0)
          headingN = 360 + headingN;
        else if (headingN > 360)
          headingN = headingN - 360;
        //        lcd.print(headingN); //print heading during rotation
        //        lcd.setCursor(0, 1);
        //        lcd.print("      ");
        //        lcd.setCursor(0, 1);


        Serial.println(headingN);

        delay(10);
      }
      Serial.println("Finished...");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("ROTATING...");
      lcd.setCursor(7, 1);
      lcd.print("OK");
      digitalWrite(BRAKEA, HIGH);
      digitalWrite(BRAKEB, HIGH);
      delay(5000);
      // ------------ ENF OF ROTATION, START OF FORWARD MOVEMENT -------------------

      Serial.println("Starting Forward Movement...");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("FORWARD MOVEMENT");
      lcd.setCursor(7, 1);
      lcd.print("...");
      delay(2000);
      digitalWrite(BRAKEA, LOW);
      digitalWrite(BRAKEB, LOW);
      digitalWrite(DIRA, HIGH);
      digitalWrite(DIRB, LOW);
      analogWrite(PWMA, 255);
      analogWrite(PWMB, 255);


      hareket_suresi = GlobalPair.distance * 2500;
      Serial.print("Duration: ");
      Serial.print(hareket_suresi);
      Serial.println(" seconds.");

      Serial.println("Longitude, Latitude");
      t = millis();
      while (millis() - t < hareket_suresi)
      {
        while (ss.available() > 0)
          if (gps.encode(ss.read()))
          {
            Serial.print(gps.location.lng(), 8);
            Serial.print(",");
            Serial.println(gps.location.lat(), 8);
          }


        if (millis() > 5000 && gps.charsProcessed() < 10)
        {
          Serial.println(F("No GPS detected: check wiring."));
          while (true);
        }
      }


      digitalWrite(BRAKEA, HIGH);
      digitalWrite(BRAKEB, HIGH);
      Serial.print("End of path: ");
      Serial.println(PATH_COUNT + 1);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("END OF PATH: ");

      lcd.setCursor(7, 1);
      lcd.print(PATH_COUNT + 1);
      if (PATH_COUNT == numPaths - 1)
        FINISHED = true;
      delay(5000);
    }
  }
} //end of loop ----------------------------------------------------------------------------------












int MinofDeg(double input)
{
  int deg1, minute1;
  deg1 = floor(input);
  minute1 = floor((input - deg1) * 60);
  return minute1;
}
float SecofDeg(double input)
{
  int deg1, minute1;
  float sec;
  deg1 = floor(input);
  minute1 = floor((input - deg1) * 60);
  sec = ((input - deg1) * 60 - minute1) * 60;
  return sec;
}


double calheading(float N_1, float E_1 , float N_2 , float E_2 )
{
  float N, E;
  double teta;
  double heading;
  N = N_2 - N_1;
  E = E_2 - E_1;
  teta = (atan(absFloat(E / (float) N)) * 180) / (float) PI;
  //heading = (atan2((double) N, (double) E) * 180) / (float) PI;
  if (N > 0)
  {
    if (E > 0)
    {
      heading = teta;
    }
    else
    {
      //Serial.println("N > 0, E < 0");
      heading = 360 - teta;
    }
  }
  else
  {
    if (E > 0)
    {
      heading = 180 - teta;
    }
    else
    {
      heading = 180 + teta;
      //Serial.println("N < 0, E < 0");
    }
  }
  // if (heading < 0)
  // heading = heading + 360;
  return heading;
}



float distance(float N_1, float E_1 , float N_2 , float E_2 ) // bu sabit değerler için,değişkenler?
{
  float N, E;
  float sub = 0;
  N = SecofDeg(absFloat(N_2 - N_1));
  E = SecofDeg(absFloat(E_2 - E_1));
  sub = sqrt((pow((N * 32), 2) + pow((E * 23), 2)));
  return  sub;
}

struct TargetPair calPair(float N_1, float E_1 , float N_2 , float E_2 ) {
  struct TargetPair subT;
  subT.heading = calheading(N_1, E_1 , N_2 , E_2 );
  subT.distance = distance(N_1, E_1 , N_2 , E_2);
  return subT;
}

float absFloat(float x)
{
  if (x < 0)
    return -x;
  else
    return x;
}


void lcdBasla()
{
  lcd.begin(16, 2);
  lcd.setCursor(4, 0);
  lcd.print("TUBiTAK");
  lcd.setCursor(5, 1);
  lcd.print("2209A");
  delay(2000);

  lcd.clear();

  lcd.setCursor(4, 0);
  lcd.print("BiTiRME");
  lcd.setCursor(4, 1);
  lcd.print("PROJESi");
  delay(2000);

  lcd.clear();

  lcd.setCursor(4, 0);
  lcd.print("GPS GUIDED");
  lcd.setCursor(4, 1);
  lcd.print("VEHICLE");
  delay(2000);

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print(" HASAN FATIH ");
  lcd.setCursor(3, 1);
  lcd.print(" ACI ");
  delay(2000);

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print(" MUHAMMED ");
  lcd.setCursor(5, 1);
  lcd.print(" SENYURT ");
  delay(2000);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print(" STARTING... ");
  delay(2000);
  //lcd.clear();
}


void GPSinfo(double LAT, double LON)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("N: ");
  lcd.setCursor(3, 0);
  lcd.print(floor(LAT));
  lcd.setCursor(6, 0);
  lcd.print(MinofDeg(LAT));
  lcd.setCursor(9, 0);
  lcd.print(SecofDeg(LAT));
  delay(10);
  lcd.setCursor(0, 1);
  delay(10);
  lcd.print("E: ");
  lcd.setCursor(3, 1);
  lcd.print(floor(LON));
  lcd.setCursor(6, 1);
  lcd.print(MinofDeg(LON));
  lcd.setCursor(9, 1);
  lcd.print(SecofDeg(LON));

  Serial.print("N: ");
  Serial.print(floor(LAT));
  Serial.print("deg, ");
  Serial.print(MinofDeg(LAT));
  Serial.print("min, ");
  Serial.print(SecofDeg(LAT));
  Serial.println("sec");

  Serial.print("E: ");
  Serial.print(floor(LON));
  Serial.print("deg, ");
  Serial.print(MinofDeg(LON));
  Serial.print("min, ");
  Serial.print(SecofDeg(LON));
  Serial.println("sec");
}



