// Shim mínimo de Arduino para compilar el firmware en el PC y probarlo con una planta simulada.
#pragma once
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <string>
#include <algorithm>
using std::isnan;
template<class A,class B> inline auto max(A a,B b)->decltype(a+b){return a>b?a:b;}
template<class A,class B> inline auto min(A a,B b)->decltype(a+b){return a<b?a:b;}
#ifndef PI
#define PI 3.14159265358979f
#endif
#define TWO_PI 6.28318530717958f
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define RISING 3
#define CHANGE 1
#define A0 54
#define A1 55
#define A2 56
#define A3 57
#define A4 58
#define A5 59
#define A6 60
#define A7 61
#define A8 62
#define A9 63
#define A10 64
#define A11 65
#define A12 66
#define A13 67
#define A14 68
#define A15 69
#define F(x) (reinterpret_cast<const __FlashStringHelper*>(x))
class __FlashStringHelper;
inline float radians(float d){return d*PI/180.0f;}
inline float degrees(float r){return r*180.0f/PI;}
template<class T,class L,class H> inline auto constrain(T x,L l,H h)->decltype(x+l+h){return x<l?l:(x>h?h:x);}
extern uint32_t sim_us;
inline uint32_t micros(){return sim_us;}
inline uint32_t millis(){return sim_us/1000;}
extern int pin_val[128];
extern int pin_pwm[128];
inline void pinMode(int,int){}
inline void digitalWrite(int p,int v){pin_val[p]=v;}
inline int digitalRead(int p){return pin_val[p];}
inline void analogWrite(int p,int v){pin_pwm[p]=v;}
extern int analog_val[128];
inline int analogRead(int p){return analog_val[p];}
inline void noInterrupts(){} inline void interrupts(){}
inline int digitalPinToInterrupt(int p){return p;}
typedef void(*isr_t)();
extern isr_t isr_tab[128];
inline void attachInterrupt(int p,isr_t f,int){isr_tab[p]=f;}
struct SerialMock{
  std::string in; std::string out;
  void begin(long){}
  int available(){return (int)in.size();}
  char read(){char c=in[0]; in.erase(0,1); return c;}
  void print(const char*s){out+=s;}
  void print(const __FlashStringHelper*s){out+=reinterpret_cast<const char*>(s);}
  void print(char c){out+=c;}
  void print(long v){out+=std::to_string(v);}
  void print(int v){out+=std::to_string(v);}
  void print(unsigned v){out+=std::to_string(v);}
  void print(unsigned long v){out+=std::to_string(v);}
  void print(uint16_t v){out+=std::to_string(v);}
  void print(uint8_t v){out+=std::to_string(v);}
  void print(int16_t v){out+=std::to_string(v);}
  void print(double v,int d=2){char b[32];snprintf(b,32,"%.*f",d,v);out+=b;}
  template<class T> void println(T v){print(v);out+="\n";}
  template<class T> void println(T v,int d){print(v,d);out+="\n";}
  void println(){out+="\n";}
};
extern SerialMock Serial;
