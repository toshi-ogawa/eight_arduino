#ifndef BiQuadLPF_h
#define BiQuadLPF_h

//双二次フィルタ
class BiQuadLPF
{
  public:
    BiQuadLPF(double _fs,double _fc,double _Q);
    void addData(double data);
    double getData();
    double getRawData();
  private:
    void updateFilter();
    double input[7];
    double output[3];
    double ma_input[2];
    double dt_input[2];
    double filteredValue;

    //サンプリング周波数, カットオフ周波数, Q値
    double fs,fc,Q;
    double a0,a1,a2,b0,b1,b2;
    
};



#endif
