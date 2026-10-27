#pragma once
extern int servo_cmd[128];
struct Servo{int p=0; void attach(int pin){p=pin;} void write(int v){servo_cmd[p]=v;}};
