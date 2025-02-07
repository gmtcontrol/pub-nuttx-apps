/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : servo [ifname]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "ethercat.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EC_TIMEOUTMON		500

/****************************************************************************
 * Private Data
 ****************************************************************************/

static OSAL_THREAD_HANDLE g_thread;
static char 		g_IOmap[1024];
static int 			g_expectedWKC;
static boolean 	g_needlf;
volatile int 		g_wkc;
static boolean 	g_inOP;
static uint8 		g_currentgroup = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * \brief Simple EtherCAT test
 *
 * This function is a very simple EtherCAT test. It initialises the EtherCAT
 * master, configures the slaves, maps the slave I/O, configures DC mode, and
 * then enters a cyclic loop where it sends and receives process data to the
 * slaves. The loop runs for 10 seconds.
 *
 * \param ifname NIC interface name, e.g. eth0
 */
void servotest(char *ifname)
{
	int i, j, oloop, iloop, chk;
	g_needlf = FALSE;
	g_inOP = FALSE;

	printf("Starting servo test\n");

	/* initialise SOEM, bind socket to ifname */

	if (ec_init(ifname))
		{
			printf("ec_init on %s succeeded.\n",ifname);

			/* find and auto-config slaves */

			if (ec_config_init(FALSE) > 0)
				{
					printf("%d slaves found and configured.\n", ec_slavecount);

					ec_config_map(&g_IOmap);
					ec_configdc();

					printf("Slaves mapped, state to SAFE_OP.\n");

					/* wait for all slaves to reach SAFE_OP state */

					ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

					oloop = ec_slave[0].Obytes;
					if ((oloop == 0) && (ec_slave[0].Obits > 0))
						{
							oloop = 1;
						}

					if (oloop > 8)
						{
							oloop = 8;
						}

					iloop = ec_slave[0].Ibytes;
					if ((iloop == 0) && (ec_slave[0].Ibits > 0))
						{
							iloop = 1;
						}

					if (iloop > 8)
						{
							iloop = 8;
						}

					printf("segments : %d : %d %d %d %d\n",	(int)ec_group[0].nsegments ,
																									(int)ec_group[0].IOsegment[0],
																									(int)ec_group[0].IOsegment[1],
																									(int)ec_group[0].IOsegment[2],
																									(int)ec_group[0].IOsegment[3]);

					printf("Request operational state for all slaves\n");
					g_expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
					printf("Calculated workcounter %d\n", g_expectedWKC);
					ec_slave[0].state = EC_STATE_OPERATIONAL;

					/* send one valid process data to make outputs in slaves happy*/

					ec_send_processdata();
					ec_receive_processdata(EC_TIMEOUTRET);

					/* request OP state for all slaves */

					ec_writestate(0);
					chk = 200;

					/* wait for all slaves to reach OP state */

					do
						{
							ec_send_processdata();
							ec_receive_processdata(EC_TIMEOUTRET);
							ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
						}
					while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

					if (ec_slave[0].state == EC_STATE_OPERATIONAL)
						{
							printf("Operational state reached for all slaves.\n");
							g_inOP = TRUE;

							/* cyclic loop */

							for(i = 1; i <= 10000; i++)
								{
									ec_send_processdata();
									g_wkc = ec_receive_processdata(EC_TIMEOUTRET);

									if(g_wkc >= g_expectedWKC)
										{
											printf("Processdata cycle %4d, WKC %d , O:", i, g_wkc);

											for (j = 0 ; j < oloop; j++)
												{
													printf(" %2.2x", *(ec_slave[0].outputs + j));
												}

											printf(" I:");
											for(j = 0 ; j < iloop; j++)
												{
													printf(" %2.2x", *(ec_slave[0].inputs + j));
												}

											printf(" T:%"PRId64"\r",ec_DCtime);
											g_needlf = TRUE;
										}

									osal_usleep(5000);
								}
								g_inOP = FALSE;
						}
					else
							{
								printf("Not all slaves reached operational state.\n");
								ec_readstate();
								for (i = 1; i<=ec_slavecount ; i++)
									{
										if (ec_slave[i].state != EC_STATE_OPERATIONAL)
											{
												printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
														i,
														ec_slave[i].state,
														ec_slave[i].ALstatuscode,
														ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
											}
									}
							}

						printf("\nRequest init state for all slaves\n");
						ec_slave[0].state = EC_STATE_INIT;

						/* request INIT state for all slaves */

						ec_writestate(0);
				}
			else
				{
					printf("No slaves found!\n");
				}

				printf("End simple test, close socket\n");

				/* stop SOEM, close socket */

				ec_close();
		}
	else
		{
			printf("No socket connection on %s\nExcecute as root\n",ifname);
		}
}

/**
 * @brief Checks for slaves that are not responding in the current group
 *
 * If a slave is not responding, it will be reconfigured and set to its
 * previous state. If a slave is lost, it will be recovered.
 *
 * @param ptr unused
 *
 * @return void
 */
OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
	int slave;
	UNUSED(ptr);

  while (1)
    {
      if (g_inOP && ((g_wkc < g_expectedWKC) || ec_group[g_currentgroup].docheckstate))
        {
          if (g_needlf)
            {
               g_needlf = FALSE;
               printf("\n");
            }

					/* one ore more slaves are not responding */

					ec_group[g_currentgroup].docheckstate = FALSE;
          ec_readstate();

          for (slave = 1; slave <= ec_slavecount; slave++)
            {
		if ((ec_slave[slave].group == g_currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
		{
			ec_group[g_currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
			{
			printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
			ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
			ec_writestate(slave);
			}
                  else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
			{
			printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
			ec_slave[slave].state = EC_STATE_OPERATIONAL;
			ec_writestate(slave);
			}
                  else if (ec_slave[slave].state > EC_STATE_NONE)
			{
			if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
				{
				ec_slave[slave].islost = FALSE;
				printf("MESSAGE : slave %d reconfigured\n",slave);
				}
			}
                  else if (!ec_slave[slave].islost)
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

					if (!ec_group[g_currentgroup].docheckstate)
						{
							printf("OK : all slaves resumed OPERATIONAL.\n");
						}
        }

      osal_usleep(10000);
	}
}

int main(int argc, char *argv[])
{
	printf("SOEM (Simple Open EtherCAT Master)\nServo test\n");

	if (argc > 1)
		{
			/* create thread to handle slave error handling in OP */

			osal_thread_create(&g_thread, 8192, &ecatcheck, (void*) &ctime);

			/* start cyclic part */

			servotest(argv[1]);
		}
	else
		{
			printf("Usage: soem_servo [ifname]\n");
		}

	printf("End program\n");

	return EXIT_SUCCESS;
}
