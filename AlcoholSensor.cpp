#include "Arduino.h"
#include "AlcoholSensor.h"


AlcoholSensor::AlcoholSensor(){
}


void AlcoholSensor::addData(double _data){
    data = _data;
  
  //LPF通過
  lpfValue.addData(_data);
  
  //ARXモデル通過
  arx.addData(lpfValue.getData());
}
double AlcoholSensor::getData(){  
  return data;  
}
double AlcoholSensor::getLPFData(){  
  return lpfValue.getData();  
}
double AlcoholSensor::getARXData(){  
  return arx.getData();  
}
//フィルタ掛ける前の生値
double AlcoholSensor::getRawData(){
  return lpfValue.getRawData();
}
void AlcoholSensor::setThreshold(double _threshold){  
  threshold = _threshold;  
}
double AlcoholSensor::getThreshold(){  
  return threshold;  
}
void AlcoholSensor::setReferenceValue(double _refValue){  
  refValue = _refValue;  
}

bool AlcoholSensor::detected(){  
  if(arx.getData() >= threshold)
    return true;
  else 
    return false;
}
