#pragma once
extern int servo_cmd[128];  // pulso en µs por pin
struct Servo{int p=0; void attach(int pin,int=0,int=0){p=pin;} void write(int deg){servo_cmd[p]=(int)(544+deg*10.311f);} void writeMicroseconds(int us){servo_cmd[p]=us;}};
