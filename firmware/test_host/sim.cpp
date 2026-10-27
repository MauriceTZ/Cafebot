// Prueba en el PC: firmware real + planta simulada (motores, encoders, servos).
// Compilar y correr:  ./correr.sh
#include "Arduino.h"
#include <chrono>
#include <thread>
#include <unistd.h>
#include <fcntl.h>
uint32_t sim_us=0; int pin_val[128]; int pin_pwm[128]; int analog_val[128]; isr_t isr_tab[128]; int servo_cmd[128];
SerialMock Serial;
#include "EEPROM.h"
EEPROMMock EEPROM;
#include "../mega_cafebot/mega_cafebot.ino"

// ---------------- Planta ----------------
static const float POS[4][2]={{ESQUINA_X[0]*CINEMATICA_L/2,ESQUINA_Y[0]*CINEMATICA_W/2},{ESQUINA_X[1]*CINEMATICA_L/2,ESQUINA_Y[1]*CINEMATICA_W/2},{ESQUINA_X[2]*CINEMATICA_L/2,ESQUINA_Y[2]*CINEMATICA_W/2},{ESQUINA_X[3]*CINEMATICA_L/2,ESQUINA_Y[3]*CINEMATICA_W/2}};
static const uint8_t ENC_A[4]={M1_ENC_A,M2_ENC_A,M3_ENC_A,M4_ENC_A}, ENC_B[4]={M1_ENC_B,M2_ENC_B,M3_ENC_B,M4_ENC_B}, ENC_INV[4]={M1_INV_ENC,M2_INV_ENC,M3_INV_ENC,M4_INV_ENC};
float ganancia[4]={1.05f,0.95f,1.0f,0.92f};  // motores distintos a propósito
float vrueda[4]={0}, dist_acum[4]={0}; long cuentas_gen[4]={0};
double X=0,Y=0,TH=0;  // pose real
void planta(float h){
  float vw[4][2];
  for(int i=0;i<4;i++){
    int d1=pin_val[PIN_DIR1[i]], d2=pin_val[PIN_DIR2[i]], pwm=pin_pwm[PIN_PWM[i]];
    int sg=(d2&&!d1)?1:((d1&&!d2)?-1:0); if(INV_MOTOR[i]) sg=-sg;
    bool freno = d1&&d2;
    float obj = sg*std::max(0,pwm-15)*2.27f*ganancia[i];
    float tau = freno?0.03f:0.08f;
    vrueda[i]+= (obj-vrueda[i])*h/tau;
    float phi = radians(((servo_cmd[PIN_SERVO[i]]-544)/10.311f-OFFSET_SERVO[i])*SIGNO_SERVO[i]);
    vw[i][0]=vrueda[i]*cos(phi); vw[i][1]=vrueda[i]*sin(phi);
    dist_acum[i]+=vrueda[i]*h;
    long objetivo=lround(dist_acum[i]/(CIRCUNFERENCIA_RUEDA/(PULSOS_POR_REVOLUCION*2.0f)));  // cada flanco físico de A
    while(cuentas_gen[i]!=objetivo){
      bool fwd = objetivo>cuentas_gen[i];
      int a=!pin_val[ENC_A[i]]; pin_val[ENC_A[i]]=a;
      bool quiero_igual = fwd ? !ENC_INV[i] : ENC_INV[i];
      pin_val[ENC_B[i]] = quiero_igual ? a : !a;
      isr_tab[ENC_A[i]]();
      cuentas_gen[i]+= fwd?1:-1;
    }
  }
  // Movimiento rígido por mínimos cuadrados: v_i = (vx - w*y_i, vy + w*x_i)
  double sx=0,sy=0,sw=0,den=0; for(int i=0;i<4;i++){sx+=vw[i][0]; sy+=vw[i][1];}
  double vx=sx/4, vy=sy/4; double num=0; for(int i=0;i<4;i++){num+= -POS[i][1]*vw[i][0]+POS[i][0]*vw[i][1]; den+=POS[i][0]*POS[i][0]+POS[i][1]*POS[i][1];}
  double w=num/den;
  X+= (vx*cos(TH)-vy*sin(TH))*h; Y+=(vx*sin(TH)+vy*cos(TH))*h; TH+=w*h;
}
std::string log_out;
bool correr_hasta(const std::string& marca, float max_s){
  uint32_t fin=sim_us+(uint32_t)(max_s*1e6);
  while(sim_us<fin){ sim_us+=100; planta(1e-4f); loop();
    if(!Serial.out.empty()){ log_out+=Serial.out; Serial.out.clear(); }
    if(log_out.find(marca)!=std::string::npos) return true; }
  return false;
}
void enviar(const char*c){ Serial.in+=c; Serial.in+="\n"; }
int fallos=0;
void prueba(const char* nombre,const char* cmd,const char* marca,double ex,double ey,double eth_deg,float tol_mm=15,float tol_deg=3){
  log_out.clear(); uint32_t t0=sim_us; enviar(cmd);
  bool ok=correr_hasta(marca,60);
  correr_hasta("@@nunca@@",0.3);
  double e_fw = hypot(pose.x-ex,pose.y-ey), e_real=hypot(X-ex,Y-ey);
  double eth_fw=fabs(degrees(wrapAngle(pose.th-radians(eth_deg)))), eth_real=fabs(degrees(wrapAngle(TH-radians(eth_deg))));
  bool pasa = ok && e_fw<tol_mm && e_real<tol_mm && eth_fw<tol_deg && eth_real<tol_deg;
  if(!pasa) fallos++;
  printf("%-34s %s  t=%.1fs  fw=(%.0f,%.0f,%.1f°) real=(%.0f,%.0f,%.1f°)\n",nombre,pasa?"OK  ":"FALLA",(sim_us-t0)/1e6,pose.x,pose.y,degrees(pose.th),X,Y,degrees(TH));
  if(!pasa) printf("   salida: %s\n",log_out.c_str());
}
// Modo interactivo: el firmware simulado habla por stdin/stdout en tiempo real.
// Lo usa pi/tests (y sirve para probar la Pi sin el robot): ./cafebot_sim --interactivo
int interactivo(){
  fcntl(0,F_SETFL,O_NONBLOCK); setvbuf(stdout,nullptr,_IONBF,0);
  setup();
  auto t0=std::chrono::steady_clock::now();
  char b[256];
  while(true){
    ssize_t n=::read(0,b,sizeof b);
    if(n>0) Serial.in.append(b,n); else if(n==0) return 0;
    uint32_t real_us=(uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-t0).count();
    while(sim_us<real_us){ sim_us+=100; planta(1e-4f); loop(); }
    if(!Serial.out.empty()){ fwrite(Serial.out.data(),1,Serial.out.size(),stdout); Serial.out.clear(); }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}
int main(int argc,char**argv){
  if(argc>1 && std::string(argv[1])=="--interactivo") return interactivo();
  setup(); correr_hasta("READY",1);
  prueba("A 500 (avanzar 0,5 m)","A 500","DONE #1",500,0,0);
  prueba("W 90 (girar 90° izq)","W 90","DONE #2",500,0,90);
  prueba("A -200 (retroceder)","A -200","DONE #3",500,-200,90);
  prueba("L 90 300 (cangrejo a la izq)","L 90 300","DONE #4",200,-200,90);
  prueba("W -180","W -180","DONE #5",200,-200,-90);
  prueba("G 0 0 0 (waypoint al origen)","G 0 0 0","DONE #6",0,0,0);
  // Watchdog
  log_out.clear(); enviar("V 100 0"); bool wd=correr_hasta("WATCHDOG",2.0);
  printf("%-34s %s\n","Watchdog V sin repetir",wd&&log_out.find("WATCHDOG")!=std::string::npos?"OK":"FALLA"); if(!wd) fallos++;
  // IR borde con seguridad activa
  correr_hasta("@@",0.5); analog_val[A8]=200; analog_val[A9]=200; enviar("IS 1"); enviar("A 1000"); correr_hasta("OK #",0.2);
  correr_hasta("@@",1.5); pin_val[IR_DI_PIN]=1; double x0=X; log_out.clear(); bool br=correr_hasta("BORDE",0.1);
  correr_hasta("@@",0.5);
  printf("%-34s %s  (recorrido desde que ve la linea hasta quedar quieto: %.1f mm)\n","IR: frena al ver borde",br?"OK":"FALLA",X-x0); if(!br) fallos++;
  pin_val[IR_DI_PIN]=0;
  // Diagnóstico de encoders: en cuadratura sana ambos flancos suman con el mismo signo
  enviar("S"); correr_hasta("@@",0.3); log_out.clear(); enviar("ED"); correr_hasta("ENCD",0.2);
  log_out.clear(); enviar("A 300"); correr_hasta("DONE",10); log_out.clear(); enviar("ED"); correr_hasta("ENCD",0.2);
  { int s1=0,b1=0; sscanf(log_out.c_str()+log_out.find("M1=")+3,"%d/%d",&s1,&b1);
    bool ok = s1>400 && b1>400 && abs(s1-b1)<20;
    printf("%-34s %s  (%s)\n","ED: flancos con el mismo signo", ok?"OK":"FALLA", log_out.substr(0,log_out.find('\n')).c_str()); if(!ok) fallos++; }
  // Cambio a x2 en vivo: la distancia recorrida debe seguir siendo la correcta
  log_out.clear(); enviar("EM 2"); correr_hasta("OK EM",0.2);
  { double x0=X, p0=pose.x; log_out.clear(); enviar("A 300"); bool ok=correr_hasta("DONE",10); correr_hasta("@@",0.3);
    ok = ok && fabs((X-x0)-300)<8 && fabs((pose.x-p0)-300)<8;
    printf("%-34s %s  (real %.0f mm, odometria %.0f mm)\n","EM 2 + A 300", ok?"OK":"FALLA", X-x0, pose.x-p0); if(!ok) fallos++; }
  enviar("EM 1"); correr_hasta("OK EM",0.2);
  // Ruta T
  enviar("S"); enviar("R"); X=Y=TH=0; correr_hasta("@@",0.3);
  prueba("T (cuadrado 500 mm)","T","DONE #13",0,0,0,25,4);
  printf("\n%s (%d fallos)\n", fallos?"HAY FALLOS":"TODO OK", fallos);
  return fallos?1:0;
}
