#ifndef ARX_h
#define ARX_h


class ARX
{
  public:
    ARX();
    void addData(double data);
    double getData();
    double getRawData();
    void set_constant(double x1,double x2,double x3,double x4);
  private:
    void updateARX();
    double input[2];
    double output[5];
    double ma_output[5];
    double estimatedValue;


double b0 = 0.0035;
double b1 = 0.0008;
double a1 = -0.9015;
double a2 = -0.0833;
double b2 = 0;
};



#endif
