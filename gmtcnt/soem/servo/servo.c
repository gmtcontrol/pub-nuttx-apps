/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : ebox [ifname] [cycletime]
 * ifname is NIC interface, f.e. eth0
 * cycletime in us, f.e. 500
 *
 * This test is specifically build for the E/BOX.
 *
 * (c)Arthur Ketels 2011
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <math.h>

#include "ethercat.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Servo ID */

#define SDxEC_MAN 	(0x00004321)
#define SDxEC_ID  	(0x000010D3)

/* CANopen DS402 0x6060 Control word */

#define CTRLW_SHUTDOWN				0x0006U
#define CTRLW_SWITCH_ON				0x0007U
#define CTRLW_ENABLE_OPER			0x000FU
#define CTRLW_FAULT_RESET			0x0080U
#define CTRLW_DISABLE_VOLTAGE	0x0000U
#define CTRLW_QUICK_STOP			0x0002U
#define CTRLW_SET_ABS_POINT		0x001FU

/* CANopen DS402 0x6041 Status word */

#define STATW_MASK								(0x7F)
#define STATW_READY_TO_SWITCH_ON	(1U << 0)
#define STATW_SWITCHED_ON					(1U << 1)
#define STATW_OPERATION_ENABLE		(1U << 2)
#define STATW_FAULT_MODE					(1U << 3)
#define STATW_VOLTAGE_ENABLED			(1U << 4)
#define STATW_QUICK_STOP					(1U << 5)
#define STATW_SWITCH_ON_DISABLED	(1U << 6)
#define STATW_WARNING							(1U << 7)
#define STATW_REMOTE							(1U << 9)
#define STATW_TARGET_REACHED			(1U << 10)
#define STATW_INT_LIMIT_ACTIVE		(1U << 11)
#define STATW_SETPOINT_ACK				(1U << 12)

/* CANopen Mode of Operation */

#define OPMOD_NO_MODE					(-1)
#define OPMOD_POSITION				( 1)
#define OPMOD_VELOCITY				( 3)
#define OPMOD_TORQUE					( 4)
#define OPMOD_HOME						( 6)
#define OPMOD_POSITION_CYC		( 8)

/* States */

#define STATE_OFF				(	STATW_SWITCH_ON_DISABLED )
#define STATE_READY			(	STATW_VOLTAGE_ENABLED | \
													STATW_READY_TO_SWITCH_ON )
#define STATE_ON 				(	STATW_QUICK_STOP | \
													STATW_VOLTAGE_ENABLED | \
													STATW_OPERATION_ENABLE | \
													STATW_SWITCHED_ON | \
													STATW_READY_TO_SWITCH_ON )
#define STATE_HOMING		( STATW_QUICK_STOP | \
													STATW_VOLTAGE_ENABLED | \
													STATW_OPERATION_ENABLE | \
													STATW_SWITCHED_ON | \
													STATW_READY_TO_SWITCH_ON | \
													STATW_TARGET_REACHED | \
													STATW_SETPOINT_ACK )
#define STATE_DISABLED	( STATW_QUICK_STOP | \
													STATW_VOLTAGE_ENABLED | \
													STATW_SWITCHED_ON | \
													STATW_READY_TO_SWITCH_ON)
#define STATE_FAULT 		( STATW_FAULT_MODE )

#define EC_TIMEOUTRST		(10000)
#define EC_TIMEOUTMON		(500)

/* sample interval in ns, here 1000us -> 1kHz */

#define SYNC0TIME				(1000 * 1000)

/* CANOpen Finite State Machine states */

enum
{
	FSM_NONE,
	FSM_START,
	FSM_RESET,
	FSM_PREPARE,
	FSM_SWITCH_ON,
	FSM_ON,
	FSM_SETMODE,
	FSM_SETPOS,
	FSM_MOVE,
	FSM_FAULT
};

/****************************************************************************
 * Private Type
 ****************************************************************************/

/* EtherCAT PDO Map type */

typedef struct PACKED
{
	uint16_t index;
	void 		*data;
	uint16_t size;
} pdo_map_t;

/* RxPDO 1 Data Type */

typedef struct PACKED
{
	uint16_t control_word;   		// 0x6040
	int32_t  target_position;  	// 0x607A
	uint16_t touch_probe_func;	// 0x60B8
} sdxec_rxpdo1_t;

/* RxPDO 1 Mapping */

static const uint32_t g_rxpdo1_map[] = 
{
	0x60400010,
	0x607A0020,
	0x60B80010,
};
static const uint16_t g_rxpdo1_map_cnt = sizeof(g_rxpdo1_map) / sizeof(g_rxpdo1_map[0]);

/* RxPDO 2 Data Type */

typedef struct PACKED
{
	uint16_t control_word;   		// 0x6040
	uint8_t  operation_mode;		// 0x6060
	int16_t  torque_offset;			// 0x60B2
	int32_t  target_velocity; 	// 0x60FF
} sdxec_rxpdo2_t;

/* RxPDO 2 Mapping */

static const uint32_t g_rxpdo2_map[] = 
{
	0x60400010,
	0x60600008,
	0x60B20010,
	0x60FF0020,
};
static const uint16_t g_rxpdo2_map_cnt = sizeof(g_rxpdo2_map) / sizeof(g_rxpdo2_map[0]);

/* RxPDO 3 Data Type */

typedef struct PACKED
{
	uint16_t control_word;   		// 0x6040
	uint8_t  operation_mode;		// 0x6060
	int16_t  target_torque;  		// 0x6071
	uint32_t torque_slope;			// 0x6087
} sdxec_rxpdo3_t;

/* RxPDO 3 Mapping */

static const uint32_t g_rxpdo3_map[] = 
{
	0x60400010,
	0x60600008,
	0x60710010,
	0x60870020,
};
static const uint16_t g_rxpdo3_map_cnt = sizeof(g_rxpdo3_map) / sizeof(g_rxpdo3_map[0]);

/* RxPDO 4 Data Type */

typedef struct PACKED
{
	uint16_t control_word;   		// 0x6040
	uint8_t  operation_mode;		// 0x6060
	int32_t  home_offset;				// 0x607C
	int8_t 	 home_mode;					// 0x6098
	uint32_t home_fast_vel;			// 0x6099, Sub:1
	uint32_t home_slow_vel;			// 0x6099, Sub:2
	uint32_t home_accel;				// 0x609A
} sdxec_rxpdo4_t;

/* RxPDO 4 Mapping */

static const uint32_t g_rxpdo4_map[] = 
{
	0x60400010,
	0x60600008,
	0x607C0020,
	0x60980008,
	0x60990120,
	0x60990220,
	0x609A0020,
};
static const uint16_t g_rxpdo4_map_cnt = sizeof(g_rxpdo4_map) / sizeof(g_rxpdo4_map[0]);

/* TxPDO 1 Data Type */

typedef struct PACKED
{
	uint16_t error_code;     		// 0x603F
	uint16_t status_word;    		// 0x6041
	int8_t 	 operation_mode; 		// 0x6061
	int32_t  actual_position; 	// 0x6064
	uint16_t touch_probe_stat; 	// 0x60B9
	int32_t  touch_probe_pos; 	// 0x60BA
	uint32_t digital_inputs; 		// 0x60FD
} sdxec_txpdo1_t;

/* TxPDO 1 Mapping */

static const uint32_t g_txpdo1_map[] = 
{
	0x603F0010, 
	0x60410010, 
	0x60610008,
	0x60640020,
	0x60B90010, 
	0x60BA0020, 
	0x60FD0020
};
static const uint16_t g_txpdo1_map_cnt = sizeof(g_txpdo1_map) / sizeof(g_txpdo1_map[0]);

/* Device PDO Maps */

static const pdo_map_t g_pdo_map[]=
{
	{ 0x1600, (void*)g_rxpdo1_map, g_rxpdo1_map_cnt },
	{ 0x1601, (void*)g_rxpdo2_map, g_rxpdo2_map_cnt },
	{ 0x1602, (void*)g_rxpdo3_map, g_rxpdo3_map_cnt },
	{ 0x1604, (void*)g_rxpdo4_map, g_rxpdo4_map_cnt },
	{ 0x1A00, (void*)g_txpdo1_map, g_txpdo1_map_cnt },
};
static const uint16_t g_pdo_map_cnt = sizeof(g_pdo_map) / sizeof(g_pdo_map[0]);

/* Sync Manager Mapping */

static const uint16_t g_sm2_map[] = { 0x1600, 0x1601 };
static const uint16_t g_sm2_map_cnt = sizeof(g_sm2_map) / sizeof(g_sm2_map[0]);

static const uint16_t g_sm3_map[] = { 0x1A00 };
static const uint16_t g_sm3_map_cnt = sizeof(g_sm3_map) / sizeof(g_sm3_map[0]);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pthread_t 			 g_thread_check;
static pthread_t 			 g_thread_rt;
static pthread_cond_t  g_cond  = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;

struct 
{
	uint8_t  IOmap[256];
	uint32_t cycle;
	uint16_t run;
	uint16_t group;
	uint32_t expectedWKC;
	uint32_t wkc;

	sdxec_txpdo1_t *pTxPDO1;
	sdxec_rxpdo1_t *pRxPDO1;
	sdxec_rxpdo2_t *pRxPDO2;
	sdxec_rxpdo3_t *pRxPDO3;
	sdxec_rxpdo4_t *pRxPDO4;
	bool needlf;
} sdxec_data;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getMonotonicTimestampUSec
 *
 * Description:
 *   Returns the current monotonic time in microseconds. This function uses
 *   the CLOCK_MONOTONIC clock to obtain the time, which is not affected by
 *   discontinuous jumps in the system time (e.g., if the system administrator
 *   manually changes the clock or if the system is synchronized with an
 *   external time reference). The time is returned as a 64-bit unsigned
 *   integer representing the number of microseconds since an unspecified
 *   starting point.
 ****************************************************************************/

uint64_t getMonotonicTimestampUSec(void)
{
  struct timespec ts;

  memset(&ts, 0, sizeof(ts));
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
    {
      abort();
    }

  return ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}

/****************************************************************************
 * Name: ecat_power
 ****************************************************************************/

static uint32_t g_ec_power_state = FSM_NONE;
static uint64_t g_start_motor_ts;

static void ecat_power(void)
{
	switch (g_ec_power_state)
		{
			case FSM_NONE:
				break;

			case FSM_START:
				g_start_motor_ts = getMonotonicTimestampUSec();
				sdxec_data.pRxPDO1->control_word = CTRLW_FAULT_RESET;
				sdxec_data.pRxPDO2->control_word = CTRLW_FAULT_RESET;
				g_ec_power_state = FSM_RESET;
				break;

			case FSM_RESET:
				sdxec_data.pRxPDO1->control_word = CTRLW_SHUTDOWN;
				sdxec_data.pRxPDO2->control_word = CTRLW_SHUTDOWN;

				if ((getMonotonicTimestampUSec() - g_start_motor_ts) > EC_TIMEOUTRST)
					{
						g_ec_power_state = FSM_PREPARE;
					}
				break;

			case FSM_PREPARE:
				sdxec_data.pRxPDO1->control_word = CTRLW_SWITCH_ON;
				sdxec_data.pRxPDO2->control_word = CTRLW_SWITCH_ON;

				if ((sdxec_data.pTxPDO1->status_word & STATE_READY) == STATE_READY)
					{
						g_ec_power_state = FSM_SWITCH_ON;
					}
				break;

			case FSM_SWITCH_ON:
				sdxec_data.pRxPDO1->control_word = CTRLW_ENABLE_OPER;
				sdxec_data.pRxPDO2->control_word = CTRLW_ENABLE_OPER;

				if ((sdxec_data.pTxPDO1->status_word & STATE_ON) == STATE_ON)
					{
						g_ec_power_state = FSM_ON;
					}
				break;

			case FSM_ON:
				g_ec_power_state = FSM_NONE;
				break;

			case FSM_SETMODE:
				sdxec_data.pRxPDO1->target_position = 0;
				sdxec_data.pRxPDO2->operation_mode  = 8;
				g_ec_power_state = FSM_SETPOS;
				break;

			case FSM_SETPOS:
				sdxec_data.pRxPDO1->target_position += 20000;
				if ((sdxec_data.pTxPDO1->status_word & STATW_SETPOINT_ACK) == STATW_SETPOINT_ACK)
					{
						g_ec_power_state = FSM_MOVE;
					}
				break;

			case FSM_MOVE:
				sdxec_data.pRxPDO1->control_word = CTRLW_SET_ABS_POINT;
				sdxec_data.pRxPDO2->control_word = CTRLW_SET_ABS_POINT;
				
				if ((sdxec_data.pTxPDO1->status_word & STATW_TARGET_REACHED) == STATW_TARGET_REACHED)
					{
						sdxec_data.pRxPDO1->control_word = CTRLW_ENABLE_OPER;
						sdxec_data.pRxPDO2->control_word = CTRLW_ENABLE_OPER;
						g_ec_power_state = FSM_SETPOS;
					}
				break;
		}
}

/****************************************************************************
 * Name: mapPDO
 ****************************************************************************/

static int mapPDO(const uint16_t  slave,
									const uint16_t  index, 
									const uint32_t *data, 
									const uint8_t   dataSize)
{
	uint8_t subIndex;
	int wkc = 0;

	/* Unmap previous registers, setting 0 in PDO_MAP subindex 0 */

	uint32_t zeroU32 = 0;
	wkc += ec_SDOwrite(slave, index, 0, FALSE, sizeof(zeroU32), &zeroU32, EC_TIMEOUTRXM);
	
	/* Modify mapping, setting register address in PDO's subindexes from 0x1A00:01 */

	for (uint8_t i = 0; i < dataSize; i++)
	{
			subIndex = i + 1;
			wkc += ec_SDOwrite(slave, index, subIndex, FALSE, sizeof(data[i]), (void*)&data[i], EC_TIMEOUTRXM);
	}

	/* Enable mapping by setting number of registers in PDO_MAP subindex 0 */

	uint32_t pdoMapSize = dataSize;
	wkc += ec_SDOwrite(slave, index, 0, FALSE, sizeof(pdoMapSize), (void*)&pdoMapSize, EC_TIMEOUTRXM);
	
	return wkc;
}

/****************************************************************************
 * Name: mapSM
 ****************************************************************************/

static int mapSM(const uint16_t  slave,
								 const uint16_t  index, 
								 const uint16_t *data, 
								 const uint8_t   dataSize)
{
	uint8_t subIndex;
	int wkc = 0;

	/* Unmap previous mappings, setting 0 in SM_MAP subindex 0 */

	uint8_t zeroU8 = 0;
	wkc += ec_SDOwrite(slave, index, 0, FALSE, sizeof(zeroU8), &zeroU8, EC_TIMEOUTRXM);
	
	/* Modify mapping, setting register address in SM's subindexes from 0x1C1x:01 */

	for (uint8_t i = 0; i < dataSize; i++)
	{
			subIndex = i + 1;
			wkc += ec_SDOwrite(slave, index, subIndex, FALSE, sizeof(data[i]), (void*)&data[i], EC_TIMEOUTRXM);
	}

	/* Save mapping count in SM (here only one PDO_MAP) */

	uint8_t pdoMapSize = dataSize;
	wkc += ec_SDOwrite(slave, index, 0, FALSE, sizeof(pdoMapSize), (void*)&pdoMapSize, EC_TIMEOUTRXM);
	
	return wkc;
}

/****************************************************************************
 * Name: SDxEC_setup
 ****************************************************************************/

static int SDxEC_setup(uint16_t 				slave, 
											 const pdo_map_t *pdo_map, uint16_t pdo_cnt,
											 const uint16_t  *sm2_map, uint16_t sm2_cnt,
											 const uint16_t  *sm3_map, uint16_t sm3_cnt)
{
	int wkc = 0;
	int i, j;

	/* Map the PDO(s)*/

	for (i = 0; i < pdo_cnt; i++)
		{
			/* Scan the SM2 map */

			for (j = 0; j < sm2_cnt; j++)
				{
					/* Is the PDO index in the SM2 list*/

					if (pdo_map[i].index == sm2_map[j])
						{
							wkc += mapPDO(slave, pdo_map[i].index, pdo_map[i].data, pdo_map[i].size);	
						}
				}
			
			/* Scan the SM3 map */

			for (j = 0; j < sm3_cnt; j++)
				{
					/* Is the PDO index in the SM3 list*/

					if (pdo_map[i].index == sm3_map[j])
						{
							wkc += mapPDO(slave, pdo_map[i].index, pdo_map[i].data, pdo_map[i].size);	
						}
				}
		}

	/* Map the SM(s)*/

	wkc += mapSM(slave, 0x1C12, sm2_map, sm2_cnt);
	wkc += mapSM(slave, 0x1C13, sm3_map, sm3_cnt);

	return wkc;
}

/****************************************************************************
 * Name: SDxEC_config
 ****************************************************************************/

static int SDxEC_config(struct ecx_context *context, uint16 slave)
{
	return SDxEC_setup(slave, 
										 g_pdo_map, g_pdo_map_cnt,
										 g_sm2_map, g_sm2_map_cnt,
										 g_sm3_map, g_sm3_map_cnt);
}

/****************************************************************************
 * Name: servo_config
 ****************************************************************************/

static void servo_config(char *ifname)
{
	int oloop, iloop;
	int slc;

	printf("Starting Servo Control\n");

	/* initialise SOEM, bind socket to ifname */

	if (ec_init(ifname))
  	{
    	printf("ec_init on %s succeeded.\n", ifname);
      
			/* find and auto-config slaves */

      if (ec_config_init(FALSE) > 0)
      	{
         	printf("%d slaves found and configured.\n", ec_slavecount);

					/* check if first slave is an valid */

					if (ec_slavecount > 0)
						{
							for (slc = 1; slc <= ec_slavecount; slc++)
								{
									/* GMT SDxEC, using ec_slave[].name is not very reliable */

									if ((ec_slave[slc].eep_man == SDxEC_MAN) && (ec_slave[slc].eep_id == SDxEC_ID))
										{
												printf("Found %s at position %d\n", ec_slave[slc].name, slc);

												/* link slave specific setup to preop->safeop hook */

												ec_slave[slc].PO2SOconfigx = &SDxEC_config;
												break;
										}
								}
						}

					/* Configure the mapping */

					ec_config_map(&sdxec_data.IOmap);
					ec_configdc();
					printf("Slaves mapped, state to SAFE_OP.\n");

					/* wait for all slaves to reach SAFE_OP state */

					ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

					oloop = ec_slave[0].Obytes;
					if ((oloop == 0) && (ec_slave[0].Obits > 0)) 
						{ 
							oloop = 1;
						}

					iloop = ec_slave[0].Ibytes;
					if ((iloop == 0) && (ec_slave[0].Ibits > 0)) 
						{ 
							iloop = 1;
						}

					printf("segments : %d : %d %d %d %d\n", (int)ec_group[0].nsegments ,
																									(int)ec_group[0].IOsegment[0],
																									(int)ec_group[0].IOsegment[1],
																									(int)ec_group[0].IOsegment[2],
																									(int)ec_group[0].IOsegment[3]);
					printf("Request operational state for all slaves\n");

         	sdxec_data.expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
					printf("Calculated workcounter %d\n", (int)sdxec_data.expectedWKC);

					/* send one processdata cycle to init SM in slaves */
				
					ec_send_processdata();
					ec_receive_processdata(EC_TIMEOUTRET);
					
					/* request OP state for all slaves */
					
					ec_slave[0].state = EC_STATE_OPERATIONAL;
					ec_writestate(0);
            	
					/* wait for all slaves to reach OP state */

					ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
          if (ec_slave[0].state == EC_STATE_OPERATIONAL)
						{
							printf("Operational state reached for all slaves.\n");
							sdxec_data.run = 1;

							/* wait for nuttx to sync on DC */

							osal_usleep(100000);

							/* SYNC0 on slave */

							ec_dcsync0(slc, TRUE, SYNC0TIME, 0);

							/* assign the PDO data */

							sdxec_data.pTxPDO1 = (sdxec_txpdo1_t *)(ec_slave[0].inputs );
							sdxec_data.pRxPDO1 = (sdxec_rxpdo1_t *)(ec_slave[0].outputs);
							sdxec_data.pRxPDO2 = (sdxec_rxpdo2_t *)(ec_slave[0].outputs + sizeof(sdxec_rxpdo1_t));

							/* start the state machine */

							g_ec_power_state = FSM_START;
						
							/* cyclic loop, reads data from RT thread */

							for (int i = 1; i <= 500; i++)
								{
									if (sdxec_data.wkc >= sdxec_data.expectedWKC)
										{
											#if 0
											printf("Processdata cycle %4d, WKC %d , O:", (int)sdxec_data.cycle, 
																																	 (int)sdxec_data.wkc);
											for (int j = 0 ; j < oloop; j++)
												{
													printf(" %2.2x", *(ec_slave[0].outputs + j));
												}

											printf(" I:");
											for (int j = 0 ; j < iloop; j++)
												{
													printf(" %2.2x", *(ec_slave[0].inputs + j));
												}
											printf(" T:%lld\r", ec_DCtime);
											#else
											printf("Error : %04X ", sdxec_data.pTxPDO1->error_code);
											printf("Status: %04X ", sdxec_data.pTxPDO1->status_word);
											printf("OpMode: %02X ", sdxec_data.pTxPDO1->operation_mode);
											printf("ActPos: %08d ", (int)sdxec_data.pTxPDO1->actual_position);
											printf(" T:%lld\r", ec_DCtime);
											#endif
											sdxec_data.needlf = TRUE;
										}
									osal_usleep(50000);
								}
						
							sdxec_data.run = 0;
						}
					else
						{
							printf("Not all slaves reached operational state.\n");

							ec_readstate();
          		for (int i = 1; i <= ec_slavecount; i++)
                {
                  if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                      printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n", i, 
																ec_slave[i].state, 
																ec_slave[i].ALstatuscode, 
																ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }					
						}
        }
         
			/* SYNC0 off */

			ec_dcsync0(slc, FALSE, SYNC0TIME, 0);
			printf("\nRequest safe operational state for all slaves\n");
			
			/* request SAFE_OP state for all slaves */
			
			ec_slave[0].state = EC_STATE_SAFE_OP;
			ec_writestate(0);
			
			/* wait for all slaves to reach state */
			
			ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
			
			/* request INIT state for all slaves */

			ec_slave[0].state = EC_STATE_INIT;
			ec_writestate(0);
      
			/* stop SOEM, close socket */
      
			printf("End servo control, close socket\n");
			ec_close();
		}
	else
    {
      printf("No socket connection on %s\nExcecute as root\n", ifname);
    }
}

/****************************************************************************
 * Name: add_timespec
 * 
 * Description: add ns to timespec
 * 
 ****************************************************************************/

void add_timespec(struct timespec *ts, int64 addtime)
{
	int64 sec, nsec;

	nsec = addtime % NSEC_PER_SEC;
	sec = (addtime - nsec) / NSEC_PER_SEC;
	ts->tv_sec += sec;
	ts->tv_nsec += nsec;
	if (ts->tv_nsec > NSEC_PER_SEC)
		{
			nsec = ts->tv_nsec % NSEC_PER_SEC;
			ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
			ts->tv_nsec = nsec;
		}
}

/****************************************************************************
 * Name: ec_sync
 * 
 * Description: PI calculation to get nuttx time synced to DC time
 * 
 ****************************************************************************/

void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime)
{
	static int64 integral = 0;
	int64 delta;
	
	/* set nuttx sync point 50us later than DC sync, just as example */

	delta = (reftime - 50000) % cycletime;
	
	if (delta > (cycletime/2)) 
		{ 
			delta = delta - cycletime; 
		}
	
	if (delta > 0)
		{ 
			integral++; 
		}
	
	if (delta < 0)
		{ 
			integral--; 
		}

	*offsettime = -(delta / 100) - (integral / 20);
}

/****************************************************************************
 * Name: ecat_thread
 * 
 * Description: EtherCAT real time thread
 * 
 ****************************************************************************/

static void ecat_thread(void *ptr)
{
	struct timespec ts;
	struct timeval tp;
	int64 cycletime;
	int64 toff = 0;
	int ht;

	pthread_mutex_lock(&g_mutex);
	gettimeofday(&tp, NULL);

	/* Convert from timeval to timespec */

	ts.tv_sec  = tp.tv_sec;

	/* round to nearest ms */

	ht = (tp.tv_usec / 1000) + 1;
	ts.tv_nsec = ht * 1000000;

	/* cycletime in ns */

	cycletime = *((int*)ptr) * 1000;
	sdxec_data.run = 0;

	while (1)
  	{
    	/* calculate next cycle start */
      
			add_timespec(&ts, cycletime + toff);
      
			/* wait to cycle start */
      
			pthread_cond_timedwait(&g_cond, &g_mutex, &ts);
      if (sdxec_data.run > 0)
      	{
					/* Communication state machine  */

					ecat_power();

					/* update the time */

         	gettimeofday(&tp, NULL);

					/* Send/Receive data */

         	ec_send_processdata();
         	sdxec_data.wkc = ec_receive_processdata(EC_TIMEOUTRET);
         	sdxec_data.cycle++;

         	/* calulate toff to get nuttx time and DC synced */
         	
					ec_sync(ec_DCtime, cycletime, &toff);
      	}
   	}
}

/****************************************************************************
 * Name: ecat_check
 * 
 * Description: EtherCAT error status check thread
 * 
 ****************************************************************************/

static void ecat_check(void *ptr)
{
	int slave;
	UNUSED(ptr);

	while (1)
		{
			if (sdxec_data.run && ((sdxec_data.wkc < sdxec_data.expectedWKC) || ec_group[sdxec_data.group].docheckstate))
				{
					if (sdxec_data.needlf)
						{
							sdxec_data.needlf = FALSE;
							printf("\n");
						}
					
					/* one ore more slaves are not responding */
					
					ec_group[sdxec_data.group].docheckstate = FALSE;
					ec_readstate();
					
					for (slave = 1; slave <= ec_slavecount; slave++)
						{
							if ((ec_slave[slave].group == sdxec_data.group) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
								{
									ec_group[sdxec_data.group].docheckstate = TRUE;
									if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
										{
											printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
											ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
											ec_writestate(slave);
										}
									else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
										{
											printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
											ec_slave[slave].state = EC_STATE_OPERATIONAL;
											ec_writestate(slave);
										}
									else if(ec_slave[slave].state > EC_STATE_NONE)
										{
											if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
												{
													ec_slave[slave].islost = FALSE;
													printf("MESSAGE : slave %d reconfigured\n",slave);
												}
										}
									else if(!ec_slave[slave].islost)
										{
											/* re-check state */

											ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
											if (ec_slave[slave].state == EC_STATE_NONE)
												{
													ec_slave[slave].islost = TRUE;
													printf("ERROR : slave %d lost\n",slave);
												}
										}
								}

							if (ec_slave[slave].islost)
								{
									if (ec_slave[slave].state == EC_STATE_NONE)
										{
											if (ec_recover_slave(slave, EC_TIMEOUTMON))
												{
													ec_slave[slave].islost = FALSE;
													printf("MESSAGE : slave %d recovered\n",slave);
												}
										}
									else
										{
											ec_slave[slave].islost = FALSE;
											printf("MESSAGE : slave %d found\n",slave);
										}
								}
						}

					if (!ec_group[sdxec_data.group].docheckstate)
						{
							printf("OK : all slaves resumed OPERATIONAL.\n");
						}
				}
			osal_usleep(10000);
		}
}

/****************************************************************************
 * Name: soem_servo_main
 ****************************************************************************/

int main(int argc, char *argv[])
{
	struct sched_param schedp;
	struct sched_param param;
	int policy = SCHED_OTHER;

	/* 1ms cycle time */

	int ctime = 1000;

	printf("SOEM (Simple Open EtherCAT Master)\nServo control\n");

	memset(&sdxec_data, 0, sizeof(sdxec_data));
	memset(&schedp, 0, sizeof(schedp));
	memset(&param, 0, sizeof(param));

  /* create thread to handle slave error handling in OP */
	
	osal_thread_create(&g_thread_check, 
										 8192,
										 (void*) &ecat_check, 
										 (void*) &ctime);

	/* do not set priority above SCHED_HPWORKPRIORITY, otherwise sockets are starved */

	schedp.sched_priority = CONFIG_GMTCNT_SOEM_PRIORITY;
	sched_setscheduler(0, SCHED_FIFO, &schedp);
	usleep(1000);

	/* create RT thread */

	osal_thread_create(&g_thread_rt, 
								 		 8192, 
								 		 (void*) &ecat_thread, 
								 		 (void*) &ctime);

	/* give it higher priority */

	param.sched_priority = CONFIG_GMTCNT_SOEM_PRIORITY + 1;
	pthread_setschedparam(g_thread_rt, policy, &param);

	/* start acyclic part */

	servo_config(argv[1]);

	schedp.sched_priority = 0;
	sched_setscheduler(0, SCHED_OTHER, &schedp);

	printf("End program\n");

	return EXIT_SUCCESS;
}
