
//Data input

//Data opslag

//Data uitlezen

//Data versturen (serial of wireless)

#include <time.h>

struct sensordata
{
    time_t time;
    float datavalue;
} ;

class Sensorvalues
{
private:
    time_t datatime;
    float datavalue;
public:
    Sensorvalues(sensordata sensor1);
    sensordata get_sensorvalues();
};

Sensorvalues::Sensorvalues(sensordata sensor)
{
    datavalue = sensor.datavalue;
    datatime = sensor.time;
};

sensordata Sensorvalues::get_sensorvalues()
{
    sensordata sensor;
    sensor.datavalue = datavalue;
    sensor.time = datatime;
    return sensor;
};


int main ()
{
  return 0;
}
