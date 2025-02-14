/****************************************************************************
 * apps/gmtcnt/soem/servo/servo.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <sched.h>

#include <servo.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_GMTCNT_REALTIME_PRIORITY
#define CONFIG_GMTCNT_REALTIME_PRIORITY		(250)
#endif

/****************************************************************************
 * Private Type
 ****************************************************************************/

/* RxPDO 1 Mapping (0x1600) */

static const uint32_t g_rxpdo1_map[] = 
{
	0x60400010,
	0x60710010,
	0x60FF0020,
	0x607A0020,
	0x60810020,
	0x60830020,
	0x60840020,
};
static const uint16_t g_rxpdo1_map_cnt = sizeof(g_rxpdo1_map) / sizeof(g_rxpdo1_map[0]);

/* RxPDO 2 Mapping (0x1601) */

static const uint32_t g_rxpdo2_map[] = 
{
	0x60980008,
	0x60990120,
	0x60990220,
	0x609A0020,
	0x607C0020,
};
static const uint16_t g_rxpdo2_map_cnt = sizeof(g_rxpdo2_map) / sizeof(g_rxpdo2_map[0]);

/* TxPDO 1 Mapping (0x1A00) */

static const uint32_t g_txpdo1_map[] = 
{
	0x603F0010, 
	0x60410010, 
	0x60610008,
	0x60640020,
	0x606C0020,
	0x60770010,
	0x60FD0020,
};
static const uint16_t g_txpdo1_map_cnt = sizeof(g_txpdo1_map) / sizeof(g_txpdo1_map[0]);

/* Device PDO Maps */

static const pdo_map_s g_pdo_map[]=
{
	{ 0x1600, g_rxpdo1_map_cnt, g_rxpdo1_map },
	{ 0x1601, g_rxpdo2_map_cnt, g_rxpdo2_map },
	{ 0x1A00, g_txpdo1_map_cnt, g_txpdo1_map },
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

static sdxec_data_s		 g_sdxec_data;
static pthread_t 			 g_thread_check;
static pthread_t 			 g_thread_rt;
static pthread_cond_t  g_cond  = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;

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
 * Name: mc_fsm_proc
 ****************************************************************************/

static void mc_fsm_proc(void *arg)
{
	sdxec_data_s *sdxec = (sdxec_data_s*)arg;

	/* check the slave id validity */

	if (sdxec->slc == EC_INVSLAVE)
		{
			sdxec->mc.state = FSM_MC_NONE;
		}

	/* MC states */

	switch (sdxec->mc.state)
		{
			case FSM_MC_NONE:
			{
				break;
			}

			case FSM_MC_POWER:
			{
				sdxec->rxpdo->control_word = CWORD_FAULT_RESET;

				sdxec->mc.start = getMonotonicTimestampUSec();
				sdxec->mc.state = FSM_MC_RESET;
				break;
			}

			case FSM_MC_RESET:
			{
				sdxec->rxpdo->control_word = CWORD_SHUTDOWN;

				if ((getMonotonicTimestampUSec() - sdxec->mc.start) > EC_TIMEOUTRST)
					{
						sdxec->mc.state = FSM_MC_PREPARE;
					}
				break;
			}

			case FSM_MC_PREPARE:
			{
				sdxec->rxpdo->control_word = CWORD_SWITCH_ON;

				if ((sdxec->txpdo->status_word & STATE_READY) == STATE_READY)
					{
						sdxec->mc.state = FSM_MC_SWITCH_ON;
					}
				break;
			}

			case FSM_MC_SWITCH_ON:
			{
				sdxec->rxpdo->control_word = CWORD_ENABLE_OPER;

				if ((sdxec->txpdo->status_word & STATE_ON) == STATE_ON)
					{
						sdxec->mc.state = FSM_MC_ON;
					}
				break;
			}

			case FSM_MC_ON:
			{
				sdxec->mc.state = FSM_MC_NONE;
				break;
			}

			case FSM_MC_POSITION:
			{
				sdxec->mc.state = FSM_MC_NONE;
				break;
			}

			case FSM_MC_ACCEL:
			{
				switch (sdxec->txpdo->operation_mode)
					{
						case OPMOD_VELOCITY:
						{
							/* Is the moving started ? */

							if ((sdxec->txpdo->status_word & SBITS_PV_MOTOR_STOPPED) != SBITS_PV_MOTOR_STOPPED)
								{
									sdxec->mc.state = FSM_MC_MOVE;
								}
							break;
						}

						case OPMOD_POSITION:
						{
							/* Is the setpoint accepted ? */

							if ((sdxec->txpdo->status_word & SBITS_PP_SETPOINT_ACK) == SBITS_PP_SETPOINT_ACK)
								{
									sdxec->mc.state = FSM_MC_MOVE;
								}
							break;
						}
					}
				break;
			}

			case FSM_MC_DECEL:
			{
				switch (sdxec->txpdo->operation_mode)
					{
						case OPMOD_VELOCITY:
						{
							/* Is the moving stoped ? */

							if ((sdxec->txpdo->status_word & SBITS_PV_MOTOR_STOPPED) == SBITS_PV_MOTOR_STOPPED)
								{
									sdxec->mc.state = FSM_MC_MOVE;
								}
							break;
						}

						case OPMOD_POSITION:
						{
							break;
						}
					}
				break;
			}

			case FSM_MC_MOVE:
			{
				/* wait till to target reached */

				if ((sdxec->txpdo->status_word & SBITS_TARGET_REACHED) == SBITS_TARGET_REACHED)
					{
						sdxec->mc.state = FSM_MC_NONE;
					}
				break;
			}
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
	int bits = 0;
	int wkc = 0;

	/* Unmap previous registers, setting 0 in PDO_MAP subindex 0 */

	uint32_t zeroU32 = 0;
	wkc += ec_SDOwrite(slave, index, 0, FALSE, sizeof(zeroU32), &zeroU32, EC_TIMEOUTRXM);
	
	/* Modify mapping, setting register address in PDO's subindexes from 0x1A00:01 */

	for (uint8_t i = 0; i < dataSize; i++)
	{
			subIndex = i + 1;

			/* write the SDO */

			wkc += ec_SDOwrite(slave, index, subIndex, FALSE, sizeof(data[i]), (void*)&data[i], EC_TIMEOUTRXM);

			/* get the index bit counts */

			bits += (index & 0xff);
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

static int SDxEC_setup(uint16_t slave, 
											 const pdo_map_s *pdo_map, uint16_t pdo_cnt,
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
	return SDxEC_setup(slave, g_pdo_map, g_pdo_map_cnt,
										 				g_sm2_map, g_sm2_map_cnt,
										 				g_sm3_map, g_sm3_map_cnt);
}

/****************************************************************************
 * Name: servo_config
 ****************************************************************************/

static void servo_config(sdxec_data_s *sdxec, const char *ifname)
{
	int oloop, iloop;
	int slc = 0;

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

									if ((ec_slave[slc].eep_man == SDxEC_MAN) && (ec_slave[slc].eep_id == SD3EC_ID))
										{
												printf("Found %s at position %d\n", ec_slave[slc].name, slc);

												/* link slave specific setup to preop->safeop hook */

												ec_slave[slc].PO2SOconfigx = &SDxEC_config;

												/* update the slave id */

												sdxec->slc = slc;
												break;
										}
								}
						}
					else
						{
							printf("No valid slaves found.\n");
							goto errout;
						}

					/* Configure the mapping */

					ec_config_map(&sdxec->IOmap);
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

         	sdxec->expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
					printf("Calculated workcounter %d\n", (int)sdxec->expectedWKC);

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
							sdxec->run = 1;

							/* wait for nuttx to sync on DC */

							osal_usleep(100000);

							/* SYNC0 on slave */
							#if EC_SYNC0TIME > 0
							ec_dcsync0(slc, TRUE, EC_SYNC0TIME, 0);
							#endif
							
							/* assign the PDO data */

							sdxec->txpdo = (sdxec_txpdo_s *)(ec_slave[0].inputs );
							sdxec->rxpdo = (sdxec_rxpdo_s *)(ec_slave[0].outputs);

							/* Power ON the motor drive */

							if (MC_Power(sdxec) == OK)
								{
									/* Endless movement */

									//MC_MoveVelocity(sdxec, 500000, 100000, 100000, 0, 0);

									MC_MoveRelative(sdxec, 500000, 200000, 50000, 50000, 0);
									MC_MoveRelative(sdxec, 500000, 300000, 50000, 50000, 0);
									MC_MoveRelative(sdxec, 500000, 400000, 50000, 50000, 0);
								}
						
							/* stop RT thread process  */

							sdxec->run = 0;
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

			#if EC_SYNC0TIME > 0
			ec_dcsync0(slc, FALSE, EC_SYNC0TIME, 0);
			#endif
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
errout:      
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

static void add_timespec(struct timespec *ts, int64 addtime)
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

static void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime)
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

static void ecat_thread(void *arg)
{
	sdxec_data_s *sdxec = (sdxec_data_s*)arg;
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

	cycletime = sdxec->cycle_time;
	sdxec->run = 0;

	while (1)
  	{
    	/* calculate next cycle start */
      
			add_timespec(&ts, cycletime + toff);
      
			/* wait to cycle start */
      
			pthread_cond_timedwait(&g_cond, &g_mutex, &ts);
      if (sdxec->run > 0)
      	{
					/* Communication state machine  */

					mc_fsm_proc(arg);

					/* update the time */

         	gettimeofday(&tp, NULL);

					/* Send/Receive data */

         	ec_send_processdata();
         	sdxec->wkc = ec_receive_processdata(EC_TIMEOUTRET);
         	sdxec->cycle++;

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

static void ecat_check(void *arg)
{
	sdxec_data_s *sdxec = (sdxec_data_s*)arg;
	int slave;

	while (1)
		{
			if (sdxec->run && ((sdxec->wkc < sdxec->expectedWKC) || ec_group[sdxec->group].docheckstate))
				{
					/* one ore more slaves are not responding */
					
					ec_group[sdxec->group].docheckstate = FALSE;
					ec_readstate();
					
					for (slave = 1; slave <= ec_slavecount; slave++)
						{
							if ((ec_slave[slave].group == sdxec->group) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
								{
									ec_group[sdxec->group].docheckstate = TRUE;
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

					if (!ec_group[sdxec->group].docheckstate)
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
	sdxec_data_s *sdxec = &g_sdxec_data;
	struct sched_param param;
	pthread_attr_t attr;
	int ret;

	printf("SOEM (Simple Open EtherCAT Master)\nSDxEC servo control\n");

	memset( sdxec, 0, sizeof(sdxec_data_s));
	memset(&param, 0, sizeof(struct sched_param));

	/* cycle time */

	sdxec->cycle_time = EC_CYCLETIME;

	/* invalidate teh slave id */

	sdxec->slc = EC_INVSLAVE;

  /* configure slave error handling thread */

	param.sched_priority = CONFIG_GMTCNT_SOEM_PRIORITY;
  pthread_attr_init(&attr);
  pthread_attr_setstacksize(&attr, 4096);
	pthread_attr_setschedparam(&attr, &param);

  /* create thread to handle slave error handling in OP */

	ret = pthread_create(&g_thread_check, 
								 			 &attr, 
								 			 (void*) &ecat_check, 
								 			 (void*) sdxec);
	if (ret < 0)
		{
			printf("ERROR: Failed to create error handling thread!\n");
			return EXIT_FAILURE;
		}

  /* configure the real time thread */

  pthread_attr_init(&attr);
	param.sched_priority = CONFIG_GMTCNT_REALTIME_PRIORITY;
  pthread_attr_setstacksize(&attr, 4096);
	pthread_attr_setschedparam(&attr, &param);

  /* create the real time thread */

	ret = pthread_create(&g_thread_rt, 
								 			 &attr, 
								 			 (void*) &ecat_thread, 
								 			 (void*) sdxec);
	if (ret < 0)
		{
			printf("ERROR: Failed to create real-time thread!\n");
			return EXIT_FAILURE;
		}

	/* change the scheduler policy of real-time thread */

	pthread_setschedparam(g_thread_rt, SCHED_FIFO, &param);

	/* start acyclic part */

	servo_config(sdxec, argv[1]);

	/* revert the scheduler policy of real-time thread */

	pthread_setschedparam(g_thread_rt, SCHED_OTHER, &param);

	printf("End program\n");

	return EXIT_SUCCESS;
}
