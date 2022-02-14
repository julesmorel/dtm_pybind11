#ifndef SQUAREBF_H
#define SQUAREBF_H

class squareBF
{
public:
    static double getValue(double x, double y, double z)
    {
      return B(1.5*x)*B(1.5*y)*B(1.5*z);
    }

private:
    static double B(double x)
    {
      if(x>=0)
      {
        if(x<=0.5){
          return 1-x;
        }else if(x>=0.5 && x <=1.5){
          return 0.5*x*x-1.5*x+1.125;
        }else if(x >=1.5){
          return 0.;
        }
      }else{
        return B(-x);
      }
    }
};

#endif // SQUAREBF_H
