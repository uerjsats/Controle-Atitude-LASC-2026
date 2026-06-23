#include <string.h>

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "platform.h"
#include "motors.h"
#define DEBUG_MODULE "PWR_DIST"
#include "debug_cf.h"

static bool motorSetEnable = false;

// 1. add o m5 na estrutura de potência principal
static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
  uint32_t m5; // add
} motorPower;

// 2. add o m5 na estrutura de potência de teste (manual)
static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
  uint16_t m5; // add
} motorPowerSet;

#ifndef DEFAULT_IDLE_THRUST
#define DEFAULT_IDLE_THRUST 0
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

void powerDistributionInit(void)
{
  motorsInit(platformConfigGetMotorMapping());
}

bool powerDistributionTest(void)
{
  bool pass = true;

  pass &= motorsTest();

  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerStop()
{
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
  motorsSetRatio(MOTOR_M5, 0); // add (parar se houver emergência)
} 

void powerDistribution(const control_t *control)
{
  #ifdef CONFIG_MOTORS_REQUIRE_ARMING
  if (!motorsIsArmed()) {
    motorSetEnable = false;
  }
  #endif

  // 1. O ESTADO DESARMADO  (Trava de Segurança)
  if (!motorSetEnable)
  {
    motorPower.m1 = 0;
    motorPower.m2 = 0;
    motorPower.m3 = 0;
    motorPower.m4 = 0;
    motorPower.m5 = 0;  
  }
  // 2. O ESTADO ARMADO
  else
  {
    // O Motor 5 (Central) assume o empuxo principal
    motorPower.m5 = limitThrust(control->thrust);

    // Os motores 1 a 4 recebem uma fração do empuxo
    uint32_t base_thrust_m1_m4 = control->thrust / 5;

    #ifdef QUAD_FORMATION_X
      int16_t r = control->roll / 2.0f;
      int16_t p = control->pitch / 2.0f;
      motorPower.m1 = limitThrust(base_thrust_m1_m4 - r + p + control->yaw);
      motorPower.m2 = limitThrust(base_thrust_m1_m4 - r - p - control->yaw);
      motorPower.m3 = limitThrust(base_thrust_m1_m4 + r - p + control->yaw);
      motorPower.m4 = limitThrust(base_thrust_m1_m4 + r + p - control->yaw);
    #else // QUAD_FORMATION_NORMAL
      motorPower.m1 = limitThrust(base_thrust_m1_m4 + control->pitch + control->yaw);
      motorPower.m2 = limitThrust(base_thrust_m1_m4 - control->roll  - control->yaw);
      motorPower.m3 = limitThrust(base_thrust_m1_m4 - control->pitch + control->yaw);
      motorPower.m4 = limitThrust(base_thrust_m1_m4 + control->roll  - control->yaw);
    #endif

    // Impede que as hélices travem completamente no ar durante o voo
    if (motorPower.m1 < idleThrust) { motorPower.m1 = idleThrust; }
    if (motorPower.m2 < idleThrust) { motorPower.m2 = idleThrust; }
    if (motorPower.m3 < idleThrust) { motorPower.m3 = idleThrust; }
    if (motorPower.m4 < idleThrust) { motorPower.m4 = idleThrust; }
    if (motorPower.m5 < idleThrust) { motorPower.m5 = idleThrust; }
  }

  // 3. ENVIA OS VALORES CALCULADOS PARA OS MOTORES FÍSICOS
  motorsSetRatio(MOTOR_M1, motorPower.m1);
  motorsSetRatio(MOTOR_M2, motorPower.m2);
  motorsSetRatio(MOTOR_M3, motorPower.m3);
  motorsSetRatio(MOTOR_M4, motorPower.m4);
  motorsSetRatio(MOTOR_M5, motorPower.m5);
}
// 4. parâmetros e logs: add o m5 para ver a telemetria dele no pc
PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_ADD(PARAM_UINT16, m5, &motorPowerSet.m5) // add - controle manual do motor 5
PARAM_GROUP_STOP(motorPowerSet)

PARAM_GROUP_START(powerDist)
PARAM_ADD(PARAM_UINT32, idleThrust, &idleThrust)
PARAM_GROUP_STOP(powerDist)

LOG_GROUP_START(motor)
LOG_ADD(LOG_UINT32, m1, &motorPower.m1)
LOG_ADD(LOG_UINT32, m2, &motorPower.m2)
LOG_ADD(LOG_UINT32, m3, &motorPower.m3)
LOG_ADD(LOG_UINT32, m4, &motorPower.m4)
LOG_ADD(LOG_UINT32, m5, &motorPower.m5) // add - log do motor 5
LOG_GROUP_STOP(motor)