#include "Arduino.h"
#include "BiQuadLPF.h"

BiQuadLPF::BiQuadLPF(double _fs,double _fc,double _Q){
  fs = _fs;
  fc = _fc;
  Q = _Q;

  double omega = 2 * 3.1415 * fc / fs;
  double alpha = sin(omega) / (2 * Q);

  a0 =  1 + alpha;
  a1 = -2 * cos(omega);
  a2 =  1 - alpha;
  b0 = (1 - cos(omega)) / 2;
  b1 =  1 - cos(omega);
  b2 = (1 - cos(omega)) / 2;

  int i=0;
  for(i=0;i<7;i++)
    input[i] = 0;
  for(i=0;i<3;i++)
    output[i] = 0;

}


void BiQuadLPF::addData(double data){

    int e,i;
  for(e=6;e>0;e--)
    input[e] = input[e-1]; 
  input[0] = data;

  //移動平均
  for(i = 1; i >= 0; i--)
    ma_input[i+1] = ma_input[i];
  ma_input[0] = (input[0]+input[1]+input[2]+input[3]+input[4]+input[5]+input[6])/7.0;

  //微分
  for(i = 1; i >= 0; i--)
    dt_input[i] = (ma_input[i] - ma_input[i+1])/(0.02);
  updateFilter();
}

double BiQuadLPF::getData(){  
  return filteredValue;  
}
double BiQuadLPF::getRawData(){  
  return input[0];  
}


void BiQuadLPF::updateFilter(){
    
  output[0] = b0/a0 * dt_input[0] + b1/a0 * dt_input[1] +b2/a0 * dt_input[2] - a1/a0 * output[1] - a2/a0 * output[2];
  int e;
  for(e=2;e>0;e--)
    output[e] = output[e-1]; 
  filteredValue = output[0];
}
