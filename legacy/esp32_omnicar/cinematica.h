#pragma once
#include "config.h"
#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>


// Paper de donde saqué las matrices
// https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf

/*
  Cinemática directa para un omnidireccional
  r: cm - Radio de las ruedas
  lx: cm - Largo del omnidireccional
  ly: cm - Ancho del omnidireccional
*/
BLA::Matrix<4, 3, double> mat_directa(
  double r,
  double lx,
  double ly) {
    BLA::Matrix<4, 3, double> result = {
        1, -1, -(lx + ly),
        1, 1, (lx + ly),
        1, 1, -(lx + ly),
        1, -1, (lx + ly)
    };
    return (1 / r) * result;
}

/*
  Cinemática inversa para un omnidireccional
  r: cm - Radio de las ruedas
  lx: cm - Largo del omnidireccional
  ly: cm - Ancho del omnidireccional
*/
BLA::Matrix<3, 4, double> mat_inversa(
  double r,
  double lx,
  double ly) {
    BLA::Matrix<3, 4, double> result = {
        1, 1, 1, 1,
        -1, 1, 1, -1,
        -1 / (lx + ly), 1 / (lx + ly), -1 / (lx + ly), 1 / (lx + ly)
    };
    return (r / 4) * result;
}

BLA::Matrix<4, 3, double> mat_cinematica_directa = mat_directa(RADIO_RUEDA, LARGO_ROBOT, ANCHO_ROBOT);
BLA::Matrix<3, 4, double> mat_cinematica_inversa = mat_inversa(RADIO_RUEDA, LARGO_ROBOT, ANCHO_ROBOT);
