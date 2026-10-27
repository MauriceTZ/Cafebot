#pragma once
#include <cstring>
struct EEPROMMock{ unsigned char m[4096]={0}; template<class T> void get(int a,T&t){memcpy(&t,m+a,sizeof(T));} template<class T> void put(int a,const T&t){memcpy(m+a,&t,sizeof(T));} };
extern EEPROMMock EEPROM;
