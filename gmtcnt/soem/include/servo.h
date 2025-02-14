/****************************************************************************
 * apps/gmtcnt/soem/include/servo.h
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

#ifndef __GMTCNT_SOEM_INCLUDE_SERVO_H
#define __GMTCNT_SOEM_INCLUDE_SERVO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <ethercat.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Servo ID */

#define SDxEC_MAN 								(0x00004321)
#define SD1EC_ID  								(0x000010D3)
#define SD3EC_ID  								(0x000010D4)

/* CANopen DS402 0x6060 Control word */

#define CBITS_SWITCH_ON						(1U << 0)
#define CBITS_DISABLE_VOLTAGE			(1U << 1)
#define CBITS_QUICK_STOP					(1U << 2)
#define CBITS_ENABLE_OPER					(1U << 3)

#define CBITS_HM_STARTOP					(1U << 4)

#define CBITS_PP_START_MOVE				(1U << 4)
#define CBITS_PP_START_IMMEDIATE	(1U << 5)
#define CBITS_PP_RELATIVE_MOVE		(1U << 6)
#define CBITS_PP_BLEND_MOVE				(1U << 9)

#define CBITS_FAULT_RESET					(1U << 7)
#define CBITS_PAUSE_HALT					(1U << 8)

#define CWORD_SHUTDOWN						(CBITS_DISABLE_VOLTAGE | CBITS_QUICK_STOP)
#define CWORD_SWITCH_ON						(CBITS_SWITCH_ON | CWORD_SHUTDOWN)
#define CWORD_DISABLE_VOLTAGE			(0)
#define CWORD_QUICK_STOP					(CBITS_QUICK_STOP)
#define CWORD_DISABLE_OPER				(CWORD_SWITCH_ON)
#define CWORD_ENABLE_OPER					(CWORD_SWITCH_ON | CBITS_ENABLE_OPER)
#define CWORD_FAULT_RESET					(CBITS_FAULT_RESET)
#define CWORD_HALT_OPER						(CBITS_PAUSE_HALT)

/* CANopen DS402 0x6041 Status word */

#define SBITS_READY_TO_SWITCH_ON	(1U << 0)
#define SBITS_SWITCHED_ON					(1U << 1)
#define SBITS_OPERATION_ENABLE		(1U << 2)
#define SBITS_FAULT_MODE					(1U << 3)
#define SBITS_VOLTAGE_ENABLED			(1U << 4)
#define SBITS_QUICK_STOP					(1U << 5)
#define SBITS_SWITCH_ON_DISABLED	(1U << 6)
#define SBITS_WARNING							(1U << 7)
#define SBITS_REMOTE							(1U << 9)
#define SBITS_TARGET_REACHED			(1U << 10)
#define SBITS_INT_LIMIT_ACTIVE		(1U << 11)

#define SBITS_CSP_FOLLOW_SETPOINT	(1U << 12)
#define SBITS_CSV_FOLLOW_SETPOINT	(1U << 12)

#define SBITS_HM_ATTAINED					(1U << 12)
#define SBITS_HM_ERROR						(1U << 13)

#define SBITS_PP_SETPOINT_ACK			(1U << 12)
#define SBITS_PP_ERROR						(1U << 13)

#define SBITS_PV_MOTOR_STOPPED		(1U << 12)

/* States */

#define STATE_OFF				(	SBITS_SWITCH_ON_DISABLED )
#define STATE_READY			(	SBITS_VOLTAGE_ENABLED | \
													SBITS_READY_TO_SWITCH_ON )
#define STATE_ON 				(	SBITS_QUICK_STOP | \
													SBITS_VOLTAGE_ENABLED | \
													SBITS_OPERATION_ENABLE | \
													SBITS_SWITCHED_ON | \
													SBITS_READY_TO_SWITCH_ON )
#define STATE_HOMING		( SBITS_QUICK_STOP | \
													SBITS_VOLTAGE_ENABLED | \
													SBITS_OPERATION_ENABLE | \
													SBITS_SWITCHED_ON | \
													SBITS_READY_TO_SWITCH_ON | \
													SBITS_TARGET_REACHED | \
													SBITS_HM_ATTAINED )
#define STATE_DISABLED	( SBITS_QUICK_STOP | \
													SBITS_VOLTAGE_ENABLED | \
													SBITS_SWITCHED_ON | \
													SBITS_READY_TO_SWITCH_ON)
#define STATE_FAULT 		( SBITS_FAULT_MODE )

/* CANopen Mode of Operation */

#define OPMOD_NO_MODE					(-1)
#define OPMOD_POSITION				( 1)
#define OPMOD_VELOCITY				( 3)
#define OPMOD_TORQUE					( 4)
#define OPMOD_HOME						( 6)
#define OPMOD_POSITION_CYC		( 8)

/* Timeout values */

#define EC_TIMEOUTRST					(10000)
#define EC_TIMEOUTMON					(500)
#define EC_TIMEOUTCMD					(500)
#define EC_TIMEOUTPWR					(1000)

/* Cycle time */

#define EC_SYNC0TIME					(1000 * 1000)
#define EC_CYCLETIME					(1000 * 1000)

/* Invalid slave ID */

#define EC_INVSLAVE						(0xFFFF)

/* CANOpen Finite State Machine states */

enum
{
	FSM_MC_NONE,
	FSM_MC_POWER,
	FSM_MC_STOP,
	FSM_MC_RESET,
	FSM_MC_PREPARE,
	FSM_MC_SWITCH_ON,
	FSM_MC_ON,
	FSM_MC_POSITION,
	FSM_MC_VELOCITY,
	FSM_MC_ACCEL,
	FSM_MC_DECEL,
	FSM_MC_MOVE,
	FSM_MC_FAULT
};

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* RxPDO Data Type */

typedef struct PACKED sdxec_rxpdo
{
	/* RxPDO 1 */

	uint16_t control_word;   		// 0x6040
	int16_t  target_torque; 		// 0x6071
	int32_t  target_velocity; 	// 0x60FF
	int32_t  profile_position; 	// 0x607A
	uint32_t profile_velocity; 	// 0x6081
	uint32_t profile_accel; 		// 0x6083
	uint32_t profile_decel; 		// 0x6084

	/* RxPDO 2 */

	int8_t 	 homing_method;			// 0x6098
	uint32_t homing_speed_fast;	// 0x6099, Sub:1
	uint32_t homing_speed_slow;	// 0x6099, Sub:2
	uint32_t homing_accel;			// 0x609A
	int32_t  homing_offset;			// 0x607C
} sdxec_rxpdo_s;

/* TxPDO Data Type */

typedef struct PACKED sdxec_txpdo
{
	/* TxPDO 1 */

	uint16_t error_code;     		// 0x603F
	uint16_t status_word;    		// 0x6041
	int8_t 	 operation_mode; 		// 0x6061
	int32_t  actual_position; 	// 0x6064
	int32_t  actual_velocity; 	// 0x606C
	int16_t  actual_torque; 		// 0x6077
	uint32_t digital_inputs; 		// 0x60FD
} sdxec_txpdo_s;

/* SDxEC driver data structure */

typedef struct sdxec_data
{
	uint8_t  IOmap[256];
	uint32_t cycle;
	uint32_t cycle_time;
	uint16_t slc;
	uint16_t run;
	uint16_t group;
	uint32_t expectedWKC;
	uint32_t wkc;

	struct sdxec_txpdo *txpdo;
	struct sdxec_rxpdo *rxpdo;

	struct
	{
		uint32_t state;
		uint64_t start;
	} mc;

} sdxec_data_s;

/* EtherCAT PDO Map type */

typedef struct pdo_map
{
	const uint16_t index;
	const uint16_t size;
	const void *data;
} pdo_map_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: MC_Power
 ****************************************************************************/

int MC_Power(void *arg);

/****************************************************************************
 * Name: MC_Stop
 ****************************************************************************/

int MC_Stop(void *arg, uint32_t decel,
											 uint32_t jerk);

/****************************************************************************
 * Name: MC_Halt
 ****************************************************************************/

int MC_Halt(void *arg, uint32_t decel,
											 uint32_t jerk);

/****************************************************************************
 * Name: MC_MoveVelocity
 ****************************************************************************/

int MC_MoveVelocity(void *arg, uint32_t velocity,
													 		 uint32_t accel,
													 		 uint32_t decel,
													 		 uint32_t jerk,
													 		 uint32_t direction);

/****************************************************************************
 * Name: MC_MoveAbsolute
 ****************************************************************************/

int MC_MoveAbsolute(void *arg, uint32_t position,
													 		 uint32_t velocity,
													 		 uint32_t accel,
													 		 uint32_t decel,
													 		 uint32_t jerk);

/****************************************************************************
 * Name: MC_MoveRelative
 ****************************************************************************/

int MC_MoveRelative(void *arg, uint32_t position,
													 		 uint32_t velocity,
													 		 uint32_t accel,
													 		 uint32_t decel,
													 		 uint32_t jerk);

#endif /* __GMTCNT_SOEM_INCLUDE_SERVO_H */
