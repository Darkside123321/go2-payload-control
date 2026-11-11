#include <cmath>
extern "C" {
int centroidal_B(double mass,const double* invI,const double* arms,double dt,double* B) {
 if(!(mass>0 && dt>0)) return -1;
 for(int k=0;k<144;k++) B[k]=0;
 for(int foot=0;foot<4;foot++){
  const double* r=arms+3*foot;
  double S[9]={0,-r[2],r[1],r[2],0,-r[0],-r[1],r[0],0};
  for(int i=0;i<3;i++) for(int j=0;j<3;j++){
   int col=3*foot+j;
   if(i==j){ B[(3+i)*12+col]=dt/mass; B[i*12+col]=.5*dt*dt/mass; }
   double a=0;for(int k=0;k<3;k++) a+=invI[3*i+k]*S[3*k+j];
   B[(9+i)*12+col]=dt*a;B[(6+i)*12+col]=.5*dt*dt*a;
  }
 }
 return 0;
}
void gait(double time,double period,double duty,int* contact,double* swing){
 double offsets[4]={0,.5,.5,0};
 for(int i=0;i<4;i++){
  if(time<0){contact[i]=1;swing[i]=0;continue;}
  double phase=std::fmod(time/period+offsets[i],1.);
  contact[i]=phase<duty; swing[i]=contact[i]?0:(phase-duty)/(1-duty);
 }
}
void swing_ref(const double* p0,const double* p1,double s,double T,double h,double* p,double* v,double* a){
 s=std::fmax(0.,std::fmin(1.,s));
 double s2=s*s,s3=s2*s,s4=s3*s,s5=s4*s;
 double f=10*s3-15*s4+6*s5;
 double df=(30*s2-60*s3+30*s4)/T;
 double ddf=(60*s-180*s2+120*s3)/(T*T);
 for(int i=0;i<3;i++){ double d=p1[i]-p0[i];p[i]=p0[i]+f*d;v[i]=df*d;a[i]=ddf*d; }
 p[2]+=16*h*(s2-2*s3+s4);
 v[2]+=16*h*(2*s-6*s2+4*s3)/T;
 a[2]+=16*h*(2-12*s+12*s2)/(T*T);
}
}
