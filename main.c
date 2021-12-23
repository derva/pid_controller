#include <stdio.h>
#include "pid.h"


int main()
{
  PIDController a;
  /* PIDInit(&a, 5.2, 3.3, 2.2); */
  printPID(&a);
  return 0;
}
