#include "Lidar.h"

int main()
{
    LidarDescription ls;
    Lidar lidar(ls);
    printf("11111111111111\n");
    SensorData *sensorData = lidar.makeData();
    printf("22222222222222\n");
}