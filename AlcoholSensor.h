#ifndef AlcoholSensor_h
#define AlcoholSensor_h
#include "BiQuadLPF.h"
#include "ARX.h"



class AlcoholSensor
{
  public:
    AlcoholSensor();
    void addData(double _data);
    void setReferenceValue(double _refValue);

    double getData();
    double getLPFData();
    double getARXData();
    double getRawData();


    bool detected();
    void setThreshold(double _threshold);
    double getThreshold();
    ARX arx = ARX();
  private:
    double data,refValue = 0,threshold=-1;
    BiQuadLPF lpfValue = BiQuadLPF(100,10,0.707);
    
};



#endif
