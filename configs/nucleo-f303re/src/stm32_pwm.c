/****************************************************************************
 * configs/nucleo-f303re/src/stm32_pwm.c
 *
 *   Copyright (C) 2012, 2015 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Paul Alexander Patience <paul-a.patience@polymtl.ca>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/drivers/pwm.h>

#include "stm32_pwm.h"
#include "nucleo-f303re.h"

#ifdef CONFIG_PWM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if 1 /* FIXME : DEBUG : HACK GOLDO */
/* FIXME : TODO : create true robot driver */

struct pwm_lowerhalf_s *g_pwm0;
struct pwm_lowerhalf_s *g_pwm1;

int goldo_pwm_update_duty(FAR struct pwm_lowerhalf_s *dev, ub16_t duty);

void goldo_maxon2_dir_p(void)
{
  (void)stm32_configgpio(GPIO_MAXON2_DIR_P);
}

void goldo_maxon2_dir_n(void)
{
  (void)stm32_configgpio(GPIO_MAXON2_DIR_N);
}

void goldo_maxon2_en(void)
{
  (void)stm32_configgpio(GPIO_MAXON2_EN);
}

void goldo_maxon2_dis(void)
{
  (void)stm32_configgpio(GPIO_MAXON2_DIS);
}

void goldo_maxon2_speed(int32_t s)
{
  ub16_t duty;
  int32_t abs_s;

  if (s>=0) {
    goldo_maxon2_dir_p();
    abs_s = s;
  } else {
    goldo_maxon2_dir_n();
    abs_s = -s;
  }
  if (abs_s>0xffff) abs_s=0xffff;
  duty = b16frac(abs_s);
  (void)goldo_pwm_update_duty(g_pwm1, duty);
}

void goldo_maxon1_dir_p(void)
{
  (void)stm32_configgpio(GPIO_MAXON1_DIR_P);
}

void goldo_maxon1_dir_n(void)
{
  (void)stm32_configgpio(GPIO_MAXON1_DIR_N);
}

void goldo_maxon1_en(void)
{
  (void)stm32_configgpio(GPIO_MAXON1_EN);
}

void goldo_maxon1_dis(void)
{
  (void)stm32_configgpio(GPIO_MAXON1_DIS);
}

void goldo_maxon1_speed(int32_t s)
{
  ub16_t duty;
  int32_t abs_s;

  if (s>=0) {
    goldo_maxon1_dir_n();
    abs_s = s;
  } else {
    goldo_maxon1_dir_p();
    abs_s = -s;
  }
  if (abs_s>0xffff) abs_s=0xffff;
  duty = b16frac(abs_s);
  (void)goldo_pwm_update_duty(g_pwm0, (int)duty);
}
#endif

/************************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ************************************************************************************/

int stm32_pwm_setup(void)
{
  static bool initialized = false;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call stm32_pwminitialize() to get an instance of the PWM interface */

      /* GOLDOBOT : PWM pour moteur DROIT (MAXON1) */
      g_pwm0 = stm32_pwminitialize(1);
      if (g_pwm0 == NULL)
        {
          pwmerr("ERROR: Failed to get the STM32 PWM lower half\n");
          return -ENODEV;
        }

      /* Register the PWM driver at "/dev/pwm0" */

      ret = pwm_register("/dev/pwm0", g_pwm0);
      if (ret < 0)
        {
          pwmerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }

      /* GOLDOBOT : PWM pour moteur GAUCHE (MAXON2) */
      g_pwm1 = stm32_pwminitialize(2);
      if (g_pwm1 == NULL)
        {
          pwmerr("ERROR: Failed to get the STM32 PWM lower half\n");
          return -ENODEV;
        }

      /* Register the PWM driver at "/dev/pwm1" */

      ret = pwm_register("/dev/pwm1", g_pwm1);
      if (ret < 0)
        {
          pwmerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_PWM */
