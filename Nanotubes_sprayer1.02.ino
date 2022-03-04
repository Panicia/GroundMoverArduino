#include <math.h>
#include <Servo.h>

class StepMotor
{
protected:
    int enablePin; // пин включения моторов
    int stepPin; // пин шага моторов
    int dirPin; // пин направления вращения моторов
    double speed; // миллиметры в минуту
    double milisecPerStep; // милисекунд за шаг
    double microsecPerStep;
    int ImpulseTime; // время импульса
    bool MotorGoingState = 0; // переменная состояния мотора 1 - мотор вращается 0 - мотор стоит
    long StepsLeft = 0; // количество оставшихся шагов
    bool GoingDir = 0; // направление вращения 0 - вперед, 1 - назад
    unsigned long lastStepTime; // время с последнего шага
    bool stepState = 0;
public:
    StepMotor(int EnP, int StP, int DirP, int _speed)
    {
        enablePin = EnP;
        stepPin = StP;
        dirPin = DirP;
        setSpeed(_speed);
    }
    void setSpeed(double mmPerMin)
    {
        speed = mmPerMin;
        microsecPerStep = (60.0 * 1000000) / (80.0 * mmPerMin);
        milisecPerStep = microsecPerStep / 1000;
        ImpulseTime = microsecPerStep / 2;
    }
    void setMicsecPerStep(int _microsecPerstep)
    {
        microsecPerStep = _microsecPerstep;
        milisecPerStep = 1000 * microsecPerStep;
    }
    void motorON()
    {
        digitalWrite(enablePin, 0);
    }
    void motorOFF()
    {
        digitalWrite(enablePin, 1);
    }
    bool IsMotorEn() { return(MotorGoingState); }
    void StartGoing(double mm, bool dir) // 1 -; 0 +;
    {
        StepsLeft = mm * 80.0;
        GoingDir = dir;
        MotorGoingState = 1;
        setDir(dir);
        digitalWrite(stepPin, HIGH);
        lastStepTime = micros();
        stepState = 1;
    }
    void StartGoingSteps(long steps, bool dir) // 1 -; 0 +;
    {
        StepsLeft = steps;
        GoingDir = dir;
        MotorGoingState = 1;
        setDir(dir);
        digitalWrite(stepPin, HIGH);
        lastStepTime = micros();
        stepState = 1;
    }
    bool KeepGoing()
    {
        bool ret = 0;
        if (StepsLeft > 0)
        {
            if (stepState && micros() - lastStepTime > ImpulseTime)
            {
                digitalWrite(stepPin, LOW);
                stepState = 0;
                lastStepTime = micros();
                StepsLeft--;
                ret = 1;
                return 1;
            }
            else
            {
                if (!stepState && micros() - lastStepTime > ImpulseTime)
                {
                    digitalWrite(stepPin, HIGH);
                    stepState = 1;
                    lastStepTime = micros();
                }
            }
            if (!ret) return 0;
        }
        else
        {
            MotorGoingState = 0;
            return 0;
        }
    }
    long getLeftSteps() { return StepsLeft; }
    void doStepsSlow(int steps, bool dir)
    {
        int impulseTime = milisecPerStep / 2;
        setDir(dir);
        for (int i = 0; i < steps; i++)
        {
            digitalWrite(stepPin, HIGH);
            delay(impulseTime);
            digitalWrite(stepPin, LOW);
            delay(impulseTime);
        }
    }
    void doStepsFast(int steps, bool dir)
    {
        int impulseTime = microsecPerStep / 2;
        setDir(dir);
        for (int i = 0; i < steps; i++)
        {
            digitalWrite(stepPin, HIGH);
            delayMicroseconds(impulseTime);
            digitalWrite(stepPin, LOW);
            delayMicroseconds(impulseTime);
        }
    }
    void go(float mm, bool dir)
    {
        if (microsecPerStep > 14000)
            doStepsSlow(mm * 80, dir);
        else
            doStepsFast(mm * 80, dir);
    }
    void setDir(bool dir)
    {
        digitalWrite(dirPin, dir);
    }
    void initPins()
    {
        pinMode(enablePin, OUTPUT);
        pinMode(stepPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
    }
    int getSpeed() { return speed; }
    int getDelay() { return microsecPerStep; }
};
class Extruder
{
protected:
    int servoPin;
    int sprayPin;
    Servo* serv;
    int servDef = 40;
    int servOn = 110;
public:
    Extruder(int _sprayPin, int _servoPin)
    {
        sprayPin = _sprayPin;
        servoPin = _servoPin;
        serv = new Servo();
    }
    void sprayOn()
    {
        //digitalWrite(sprayPin, 0);
        serv->write(servOn);
    }
    void sprayOff()
    {
        //digitalWrite(sprayPin, 1);
        serv->write(servDef);
    }
    void CompressorOn()
    {
        digitalWrite(sprayPin, 0);
    }
    void CompressorOff()
    {
        digitalWrite(sprayPin, 1);
    }
    void initPins()
    {
        digitalWrite(sprayPin, 1);
        pinMode(sprayPin, OUTPUT);
        serv->attach(servoPin);
        serv->write(servDef);
    }
};
class Dispergator
{
protected:
    int timeUntillONOFF = 10;
    int dispOnPin;
    int dispOffPin;
    double dispTime;
    double delayTime;
    bool isWorking = 0;
    bool Disperg = 1;
    unsigned long nowTime;
    bool dispOnGoing = 0;
    bool dispOffGoing = 0;
    unsigned long dispOnStartTime;
    unsigned long dispOffStartTime;

public:
    Dispergator(int disppin, int dispOff)
    {
        dispOnPin = disppin;
        dispOffPin = dispOff;
    }
    void dispOn()
    {
        digitalWrite(dispOnPin, 0);
        dispOnGoing = 1;
        dispOnStartTime = millis();
    }
    void dispOff()
    {
        digitalWrite(dispOffPin, 0);
        dispOffGoing = 1;
        dispOffStartTime = millis();
    }
    void initPins()
    {
        digitalWrite(dispOnPin, 1);
        digitalWrite(dispOffPin, 1);
        pinMode(dispOnPin, OUTPUT);
        pinMode(dispOffPin, OUTPUT);
    }
    void GoDisper(double _dispTime, double _delayTime)
    {
        dispTime = _dispTime;
        delayTime = _delayTime;
        isWorking = true;
        nowTime = millis();
        dispOn();
    }
    void KeepDisper()
    {
        if (dispOnGoing)
        {
            if (millis() > dispOnStartTime + timeUntillONOFF)
            {
                digitalWrite(dispOnPin, 1);
                dispOnGoing = 0;
            }
        }
        if (dispOffGoing)
        {
            if (millis() > dispOffStartTime + timeUntillONOFF)
            {
                digitalWrite(dispOffPin, 1);
                dispOffGoing = 0;
            }
        }
        if (isWorking)
        {
            if (delayTime != 0)
            {
                if (Disperg && millis() - dispTime > nowTime)
                {
                    dispOff();
                    Disperg = false;
                    nowTime = millis();
                }
                if (!Disperg && millis() - delayTime > nowTime)
                {
                    dispOn();
                    Disperg = true;
                    nowTime = millis();
                }
            }
        }
    }
    void StopDisper()
    {
        dispOff();
        isWorking = 0;
        Disperg = 1;
    }
    bool IsWorking()
    {
        if (dispOffGoing)
            return true;
        else
            return isWorking;
    }
};
class Switch
{
protected:
    int bPin;
    bool pol;
public:
    Switch(int pin, bool p)
    {
        pol = p;
        bPin = pin;
    }
    bool getState()
    {
        if (!pol)
            return !digitalRead(bPin);
        else
            return digitalRead(bPin);
    }
    void initPins()
    {
        pinMode(bPin, INPUT_PULLUP);
    }
};
class Termistor
{
protected:
    int tPin;
    float R;
    float T;
    float B = 3950; // B-коэффициент
    float SERIAL_R = 102000; // сопротивление последовательного резистора, 102 кОм
    float THERMISTOR_R = 100000; // номинальное сопротивления термистора, 100 кОм
    float NOMINAL_T = 25; // номинальная температура (при которой TR = 100 кОм)

public:
    Termistor(int pin)
    {
        tPin = pin;
    }
    float getTemp()
    {
        T = R / THERMISTOR_R; // (R/Ro)
        T = log(T); // ln(R/Ro)
        T /= B; // 1/B * ln(R/Ro)
        T += 1.0 / (NOMINAL_T + 273.15); // + (1/To)
        T = 1.0 / T; // Invert
        T -= 273.15;
        return T;
    }
    float getRez()
    {
        float t = analogRead(tPin);
        R = 1023.0 / t - 1;
        R = SERIAL_R / R;
        return R;
    }
    void initPins()
    {
        pinMode(tPin, INPUT);
    }
};
class GCODE
{
protected:
    Extruder* Ex;
    Dispergator* Disp;
    StepMotor* MX;
    StepMotor* MY;
    Switch* SwX;
    Switch* SwY;
    double X = 0;
    double Y = 0;
    double defaultSpeed = 1000;
    double speed;
    double lastSpeed;
    bool G1Going = 0;
    bool G00Going = 0;
    unsigned long delayTime;
    unsigned long delaySetTime;
    bool delayGoing = 0;
public:
    GCODE(StepMotor* mx, StepMotor* my, Switch* swx, Switch* swy, Extruder* ex, Dispergator* disp)
    {
        MX = mx;
        MY = my;
        SwX = swx;
        SwY = swy;
        Ex = ex;
        Disp = disp;
        lastSpeed = speed;
    }
    bool IsGoing()
    {
        if (G1Going || G00Going || delayGoing)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    void KeepGoing()
    {
        if (G1Going)
        {
            if (MX->getLeftSteps() > 0 || MY->getLeftSteps() > 0)
            {
                if (MX->IsMotorEn())
                    MX->KeepGoing();
                if (MY->IsMotorEn())
                    MY->KeepGoing();
            }
            else
            {
                G1Going = false;
            }
        }
        else if (G00Going)
        {
            if (!SwX->getState() || !SwY->getState())
            {
                if (!SwX->getState())
                {
                    if (MX->IsMotorEn())
                    {
                        MX->KeepGoing();
                    }
                    else
                    {
                        MX->StartGoingSteps(1, 1);
                    }
                }
                if (!SwY->getState())
                {
                    if (MY->IsMotorEn())
                    {
                        MY->KeepGoing();
                    }
                    else
                    {
                        MY->StartGoingSteps(1, 1);
                    }
                }
            }
            else
            {
                G00Going = false;
                X = 0;
                Y = 0;
            }
        }
        else if (delayGoing)
        {
            if (millis() - delayTime > delaySetTime)
                delayGoing = false;
        }
        if (Disp->IsWorking())
        {
            Disp->KeepDisper();
        }

    }
    void G01(double x, double y, double f)//ход Х У
    {
        if (X == x && Y == y)
            return;
        if (f == 0)
            speed = lastSpeed;
        else
        {
            speed = f;
            lastSpeed = f;
        }
        double dx = x - X;
        double dy = y - Y;
        X = x;
        Y = y;
        double speedX = speed * (abs(dx) / sqrt(dx * dx + dy * dy));
        double speedY = speed * (abs(dy) / sqrt(dx * dx + dy * dy));
        MX->setSpeed(speedX);
        MY->setSpeed(speedY);
        if (dx != 0)
        {
            if (dx > 0)
                MX->StartGoing(dx, 0);
            else
                MX->StartGoing(abs(dx), 1);
        }
        if (dy != 0)
        {
            if (dy > 0)
                MY->StartGoing(dy, 0);
            else
                MY->StartGoing(abs(dy), 1);
        }
        G1Going = true;
    }
    void G03(double x, double y, double i, double j, double f)//круговая интерполяция против часовой стрелки X Y I J
    {

    }
    void G02()//круговая интерполяция по часовой стрелке X Y I J
    {

    }
    void M00()//выключение двигателей
    {
        MX->motorOFF();
        MY->motorOFF();
    }
    void M01()//включение двигателей
    {
        MX->motorON();
        MY->motorON();
    }
    void M03() //включение распыления
    {
        Ex->sprayOn();
        Ex->CompressorOn();
    }
    void M04()
    {
        
    }
    void M05()//остановка распыления
    {
        Ex->sprayOff();
        Ex->CompressorOff();
    }
    void M06(double intDisp, double intDelay)// включение диспергатора
    {
        Disp->GoDisper(intDisp, intDelay);
    }
    void M07()// выключение диспергатора
    {
        Disp->StopDisper();
    }
    void M08() // включение компрессора
    {
        Ex->CompressorOn();
    }
    void M09() // выключение компрессора
    {
        Ex->CompressorOff();
    }
     void M11() // включение компрессора
    {
        Ex->sprayOn();
    }
    void M12() // выключение компрессора
    {
        Ex->sprayOff();
    }
    void M25(double P)// остановка на время P 
    {
        delayTime = P;
        delaySetTime = millis();
        delayGoing = true;
    }
    void M30(){} // конец операции
    void M31() // обнулить координаты
    {
      X = 0;
      Y = 0;
    }
    void G00() //go homu
    {
        MX->setSpeed(defaultSpeed);
        MY->setSpeed(defaultSpeed);
        MX->StartGoingSteps(1, 1);
        MY->StartGoingSteps(1, 1);
        G00Going = true;
    }
};



bool IsGoing = 0;
String buffer = "";

int defSpeed = 1000;
int M1en = A0;
int M1step = 2;
int M1dir = 3;
int M2en = A0;
int M2step = 4;
int M2dir = 5;

int ExPin = 11;
int DispPinOn = A4;
int DispPinOff = A5;
int servPin = 12;

Extruder* Ex = new Extruder(ExPin, servPin);
Dispergator* Disp = new Dispergator(DispPinOn, DispPinOff);

StepMotor* M1 = new StepMotor(M1en, M1step, M1dir, defSpeed);
StepMotor* M2 = new StepMotor(M2en, M2step, M2dir, defSpeed);

int swx = 7;
int swy = 6;
Switch* SwX = new Switch(swx, 1);
Switch* SwY = new Switch(swy, 1);

GCODE* G = new GCODE(M1, M2, SwX, SwY, Ex, Disp);

bool WaitingForDone = false;

void setup()
{
    Serial.begin(250000);
    Serial.setTimeout(10);
    M1->initPins();
    M2->initPins();
    Ex->initPins();
    Disp->initPins();
    G->M00();  // выключение двиателей
    G->M05();  // остановка распыления
    SwX->initPins();
    SwY->initPins();
}
void loop()
{
    if (WaitingForDone)
    {
        if (!G->IsGoing())
        {
            WaitingForDone = false;
            Serial.print(1);
        }
    }
    else
    {
        ReadCom();
    }
    G->KeepGoing();
}
void ReadCom()
{
    if (Serial.available() != 0)
    {
        if (!IsGoing)
        {
            char a = Serial.read();
            {
                if (a == '%')
                    IsGoing = true;
                Serial.print(1);
            }
        }
        else
        {
            buffer = Serial.readString();
            for (int i = 0; i < buffer.length(); i++)
            {
                if (buffer[i] == 'G')
                {
                    if (buffer[i + 2] == '0')
                    {
                        G->G00();
                        WaitingForDone = true;
                        return;
                    }
                    else if (buffer[i + 2] == '1')
                    {
                        String readX = "";
                        String readY = "";
                        String readF = "";
                        for (int j = i + 3; j < buffer.length(); j++)
                        {
                            if (buffer[j] == 'X')
                            {
                                for (int k = j; k < buffer.length(); k++)
                                {
                                    if (buffer[k] == ' ')
                                    {
                                        for (int l = j + 1; l < k; l++)
                                        {
                                            readX += buffer[l];
                                        }
                                        j = k;
                                        break;
                                    }
                                }
                            }
                            if (buffer[j] == 'Y')
                            {
                                for (int k = j; k < buffer.length(); k++)
                                {
                                    if (buffer[k + 1] == ' ' || k + 1 == buffer.length())
                                    {
                                        for (int l = j + 1; l <= k; l++)
                                        {
                                            readY += buffer[l];
                                        }
                                        j = k;
                                        break;
                                    }
                                }
                            }
                            if (buffer[j] == 'F')
                            {
                                for (int k = j; k < buffer.length(); k++)
                                {
                                    if (buffer[k + 1] == ' ' || k + 1 == buffer.length())
                                    {
                                        for (int l = j + 1; l <= k; l++)
                                        {
                                            readF += buffer[l];
                                        }
                                        j = k;
                                        break;
                                    }
                                }
                            }
                        }
                        G->G01(readX.toFloat(), readY.toFloat(), readF.toFloat());
                        WaitingForDone = true;
                        return;
                    }
                }
                if (buffer[i] == 'M')
                {
                    String readM = "";
                    readM = readM + buffer[i + 1] + buffer[i + 2];
                    if (readM == "00")
                    {
                        G->M00();
                        Serial.print(1);
                        return;
                    }
                    else if (readM == "01")
                    {
                        G->M01();
                        Serial.print(1);
                        return;
                    }
                    else if (readM == "03")
                    {
                        G->M03();
                        Serial.print(1);
                        return;
                    }
                    else if (readM == "05")
                    {
                        G->M05();
                        Serial.print(1);
                        return;
                    }
                    else if (readM == "08")
                    {
                        G->M08();
                        Serial.print(1);
                        return;
                    }
                    else if (readM == "09")
                    {
                        G->M09();
                        Serial.print(1);
                        return;
                    }
                    else if (readM == "11")
                    {
                        G->M11();
                        Serial.print(1);
                        return;
                    }
                    else if (readM == "12")
                    {
                        G->M12();
                        Serial.print(1);
                        return;
                    }
                    else if (readM == "30")
                    {
                        IsGoing = false;
                        Serial.print(1);
                        return;
                    }
                    else if (readM == "31")
                    {
                        G->M31();
                        Serial.print(1);
                        return;
                    }
                    else if (readM == "25")
                    {
                        String readP = "";
                        for (int j = i + 3; j < buffer.length(); j++)
                        {
                            if (buffer[j] == 'P')
                            {
                                for (int k = j; k < buffer.length(); k++)
                                {
                                    if (buffer[k] == ' ' || k + 1 == buffer.length())
                                    {
                                        for (int l = j + 1; l <= k; l++)
                                        {
                                            readP += buffer[l];
                                        }
                                        j = k;
                                        break;
                                    }
                                }
                            }
                        }
                        G->M25(readP.toFloat());
                        WaitingForDone = true;
                        return;
                    }
                    else if (readM == "06")
                    {
                        String readP = "";
                        String readN = "";
                        for (int j = i + 3; j < buffer.length(); j++)
                        {
                            if (buffer[j] == 'P')
                            {
                                for (int k = j; k < buffer.length(); k++)
                                {
                                    if (buffer[k] == ' ' || k + 1 == buffer.length())
                                    {
                                        for (int l = j + 1; l <= k; l++)
                                        {
                                            readP += buffer[l];
                                        }
                                        j = k;
                                        break;
                                    }
                                }
                            }
                            if (buffer[j] == 'N')
                            {
                                for (int k = j; k < buffer.length(); k++)
                                {
                                    if (buffer[k] == ' ' || k + 1 == buffer.length())
                                    {
                                        for (int l = j + 1; l <= k; l++)
                                        {
                                            readN += buffer[l];
                                        }
                                        j = k;
                                        break;
                                    }
                                }
                            }
                        }
                        G->M06(readP.toFloat(), readN.toFloat());
                        Serial.print(1);
                        return;
                    }
                    else if (readM == "07")
                    {
                        G->M07();
                        Serial.print(1);
                        return;
                    }
                }
            }
        }
    }
}
