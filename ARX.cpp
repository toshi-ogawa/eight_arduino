#include "Arduino.h"
#include "ARX.h"

ARX::ARX(){
  int i=0;
  for(i=0;i<2;i++)
    input[i] = 0;
  for(i=0;i<5;i++)
    output[i] = 0;
}


void ARX::addData(double data){
  int e;
  for(e=1;e>0;e--)
    input[e] = input[e-1]; 
  input[0] = data;

  updateARX();
}
double ARX::getData(){  
  return estimatedValue;  
}

double ARX::getRawData(){  
  return input[0];  
}


void ARX::updateARX(){
  int i;
  for(i = 5; i >= 0; i--)
    output[i+1]=output[i];
  
 //モデル通す
  output[0] = b0*input[0] + b1*input[1] -a1*output[1] -a2*output[2];

//移動平均
  for(i=3;i>=0;i--)
   ma_output[i+1]= ma_output[i];
  ma_output[0]=(output[0]+output[1]+output[2]+output[3]+output[4])/5.0;
  estimatedValue = ma_output[0];

}


void ARX::set_constant(double x1,double x2,double x3,double x4){  
  b0 =x1;
  b1 = x2;
  a1= x3;
  a2= x4;
}
