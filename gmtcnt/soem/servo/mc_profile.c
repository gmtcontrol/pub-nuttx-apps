/****************************************************************************
 * apps/gmtcnt/soem/servo/mc_profile.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <servo.h>


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: MC_SetModeOfOperation
 ****************************************************************************/

static int MC_SetModeOfOperation(uint16_t slave, int8_t value)
{
  return ec_SDOwrite(slave, 0x6060, 0, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
}

/****************************************************************************
 * Name: MC_Power
 ****************************************************************************/

int MC_Power(void *arg)
{
	sdxec_data_s *sdxec = (sdxec_data_s*)arg;
	uint32_t wait;

	DEBUGASSERT(sdxec != NULL);

	/* check the slave id validity */

	if (sdxec->slc == EC_INVSLAVE)
		{
			return ERROR;
		}

	/* update the state machine */

	sdxec->mc.state = FSM_MC_POWER;

	/* Wait for the state machine done */

	wait = 0;
	while (sdxec->mc.state != FSM_MC_NONE)
		{
			osal_usleep(1000);
	
			/* security timer */

			if (++wait >= EC_TIMEOUTPWR)
				{
					return ERROR;
				}
		}

	return OK;
}

/****************************************************************************
 * Name: MC_MoveVelocity
 ****************************************************************************/

int MC_MoveVelocity(void *arg, uint32_t velocity,
													 		 uint32_t accel,
													 		 uint32_t decel,
													 		 uint32_t jerk,
													 		 uint32_t direction)
{
	sdxec_data_s *sdxec = (sdxec_data_s*)arg;
	uint32_t wait;

	DEBUGASSERT(sdxec != NULL);

	UNUSED(jerk);
	UNUSED(direction);

	/* check the slave id validity */

	if (sdxec->slc == EC_INVSLAVE)
		{
			return ERROR;
		}

	/* check the mode of operation */

	if (sdxec->txpdo->operation_mode != OPMOD_VELOCITY)
		{
			/* set the mode of operation */

			if (MC_SetModeOfOperation(sdxec->slc, OPMOD_VELOCITY))
				{
					/* Wait for mode of operation change */

					wait = 0;
					while (sdxec->txpdo->operation_mode != OPMOD_VELOCITY)
						{
							osal_usleep(1000);

							/* security timer */

							if (++wait >= EC_TIMEOUTCMD)
								{
									return ERROR;
								}
						}
				}
			else
				{
					return ERROR;
				}
		}

	/* update the values (for start) */

	sdxec->rxpdo->target_velocity = velocity;
	sdxec->rxpdo->profile_accel = accel;
	sdxec->rxpdo->profile_decel = decel;

	/* update the state machine */

	sdxec->mc.state = FSM_MC_ACCEL;

	/* Wait for the state machine to reach target */

	while (sdxec->mc.state != FSM_MC_NONE)
		{
			osal_usleep(1000);
		}

	/* update the target velocity (for stop) */

	sdxec->rxpdo->target_velocity = 0;

	/* update the state machine */

	sdxec->mc.state = FSM_MC_DECEL;

	/* Wait for the state machine to reach target */

	while (sdxec->mc.state != FSM_MC_NONE)
		{
			osal_usleep(1000);
		}

	return OK;
}

/****************************************************************************
 * Name: MC_MovePosition
 ****************************************************************************/

int MC_MovePosition(void *arg, uint32_t position,
													 		 uint32_t velocity,
													 		 uint32_t accel,
													 		 uint32_t decel,
													 		 uint32_t jerk,
													 		 bool relative)
{
	sdxec_data_s *sdxec = (sdxec_data_s*)arg;
	uint32_t wait;

	DEBUGASSERT(sdxec != NULL);

	UNUSED(jerk);

	/* check the slave id validity */

	if (sdxec->slc == EC_INVSLAVE)
		{
			return ERROR;
		}

	/* check the actual position */

	if ((sdxec->txpdo->actual_position == position) && (relative == false))
		{
			return OK;
		}

	/* check the mode of operation */

	if (sdxec->txpdo->operation_mode != OPMOD_POSITION)
		{
			/* set the mode of operation */

			if (MC_SetModeOfOperation(sdxec->slc, OPMOD_POSITION))
				{
					/* Wait for mode of operation change */

					wait = 0;
					while (sdxec->txpdo->operation_mode != OPMOD_POSITION)
						{
							osal_usleep(1000);

							/* security timer */

							if (++wait >= EC_TIMEOUTCMD)
								{
									return ERROR;
								}
						}
				}
			else
				{
					return ERROR;
				}
		}

	/* configure the control word */

	sdxec->rxpdo->control_word &= ~(CBITS_PAUSE_HALT | CBITS_PP_START_MOVE);
	sdxec->rxpdo->control_word |=  (CBITS_PP_START_IMMEDIATE | CBITS_PP_BLEND_MOVE);

	/* selected position mode */

	if (relative)
		{
			sdxec->rxpdo->control_word |= CBITS_PP_RELATIVE_MOVE;
		}
	else
		{
			sdxec->rxpdo->control_word &= ~CBITS_PP_RELATIVE_MOVE;
		}

	/* 
		* Previous setpoint still in process. 
		* Setpoint overwriting will be accepted.
		* We are waiting for it.
		*/

	wait = 0;
	do
	{
		osal_usleep(1000);

		/* security timer */

		if (++wait >= EC_TIMEOUTCMD)
			{
				/* timeout has been reached */

				sdxec->rxpdo->target_velocity = 0;
				sdxec->mc.state = FSM_MC_NONE;
				return ERROR;
			}
	} while ((sdxec->txpdo->status_word & SBITS_PP_SETPOINT_ACK) == SBITS_PP_SETPOINT_ACK);
	
	/* update the values */

	sdxec->rxpdo->profile_position = position;
	sdxec->rxpdo->profile_velocity = velocity;
	sdxec->rxpdo->profile_accel = accel;
	sdxec->rxpdo->profile_decel = decel;

	/* start the move */

	sdxec->rxpdo->control_word |= CBITS_PP_START_MOVE;

	/* update the state machine */

	sdxec->mc.state = FSM_MC_ACCEL;

	/* Wait for the state machine to reach target */

	while (sdxec->mc.state != FSM_MC_NONE)
		{
			osal_usleep(1000);
		}

	return OK;
}

/****************************************************************************
 * Name: MC_MoveAbsolute
 ****************************************************************************/

int MC_MoveAbsolute(void *arg, uint32_t position,
													 		 uint32_t velocity,
													 		 uint32_t accel,
													 		 uint32_t decel,
													 		 uint32_t jerk)
{
	return MC_MovePosition(arg, position,
													 		velocity,
													 		accel,
													 		decel,
													 		jerk,
													 		false);
}

/****************************************************************************
 * Name: MC_MoveRelative
 ****************************************************************************/

int MC_MoveRelative(void *arg, uint32_t position,
													 		 uint32_t velocity,
													 		 uint32_t accel,
													 		 uint32_t decel,
													 		 uint32_t jerk)
{
	return MC_MovePosition(arg, position,
													 		velocity,
													 		accel,
													 		decel,
													 		jerk,
													 		true);
}
