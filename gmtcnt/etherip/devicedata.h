/*******************************************************************************
 * Copyright (c) 2018, Rockwell Automation, Inc.
 * All rights reserved.
 *
 ******************************************************************************/
 
#ifndef DEVICE_DATA_H_
#define DEVICE_DATA_H_
 
#define OPENER_DEVICE_VENDOR_ID      	(CONFIG_ODVA_ETHERIP_VENDORID)
#define OPENER_DEVICE_PRODUCT_CODE  	(CONFIG_ODVA_ETHERIP_PRODCODE)
#define OPENER_DEVICE_TYPE           	(CONFIG_ODVA_ETHERIP_DEVTYPE)
#define OPENER_DEVICE_NAME 						(EipByte *)(CONFIG_ODVA_ETHERIP_DEVNAME)
#define OPENER_DEVICE_MAJOR_REVISION 	((CONFIG_ODVA_ETHERIP_DEVREV >> 8) & 0xff)
#define OPENER_DEVICE_MINOR_REVISION 	((CONFIG_ODVA_ETHERIP_DEVREV     ) & 0xff)

#endif /* DEVICE_DATA_H_ */
