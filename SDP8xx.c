/**************************************************
* SDP8xx
* Description: This program is to communicate via I2C with the SDP8xx-Digital Sensor of Sensirion.
* Both addresses should be readable.
* First release: 15.01.2022
* (C) by Thomas Schütz from Austria
* compufter2@gmx.at
**************************************************/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/i2c-dev.h>
#include "sdp8xx.h"

// Check I2C Devices with i2cdetect -y 1

#define SDP8X0_ADDR 0x26
#define SDP8X1_ADDR 0x25

typedef enum {
	SDP8X0 = 0x26,
	SDP8X1 = 0x25
} Sensor;

static const float scaleFactorTemp = 200;

typedef enum {
	CONT_MF_AV =0x3603,
	CONT_MF_N  =0x3608,
	CONT_DP_AV =0x3615,
	CONT_DP_N  =0x361E,
	CONT_STOP  =0x3FF9,
	TRIG_MF_N  =0x3624,
	TRIG_MF_ST =0x3726,
	TRIG_DP_N  =0x362F,
	TRIG_DP_ST =0x372D,
	SOFT_RESET =0x0006,
	SLEEP      =0x3677,
	EXIT_SLEEP =0x0000,
	PRODUCT_ID =0x367CE102

} Command;

int InitSdp8xx(Sensor sensor);

int Sdp8xx(Command cmd);

uint8_t gencrc(uint8_t *data, size_t len)
{
    uint8_t crc = 0xff;
    size_t i, j;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ 0x31);
            else
                crc <<= 1;
        }
    }
    return crc;
}

int productid(void);

int fd;                 //Device-Handle


int main()
{
	int status;
	int i;			//Counter
	int byteread;

  	uint8_t buf[18];        /* I/O buffer */

	uint8_t Diffpressure[3], Temp[3], Scale[3];

	int16_t Diffpressure1, Temp1;
        uint16_t Scalefactor1=0;

	float DiffpressurePa1, DiffpressureBar1, TempC1;

	status=InitSdp8xx(SDP8X1);
//	status=InitSdp8xx(SDP8X0);


	//Stop Conversion if running
	status=Sdp8xx(CONT_STOP);

	//Reset
	status=Sdp8xx(SOFT_RESET);

        // Start Continuous Measurement Differential pressure Update rate 0.5ms Average
	status=Sdp8xx(CONT_DP_AV);
	//Wait Minimum 8ms
        usleep(20000);
	printf("\nStartMeasure %d ",status);

        if (status != 2)
        {
		perror("\nWrite to register 1");
                printf("\nNumber: %d",status);
  //              exit(-1);
        }

  	for (;;) // loop forever
    	{
		if(Scalefactor1==0)
		{	byteread=9;
		}
		else
		{	byteread=6;
		}
		// Read conversion
		status=read(fd, buf, byteread);
      		if (status != byteread)
        	{
        		perror("\nRead conversion");
			printf("\nNumber: %d",status);
        		exit(-1);
        	}

		//display Read Out
/*	    	printf("\nRead Values: ");
                for(i=0;i<status;i++)
                {
                        printf("0x%02x ", buf[i]);
                }*/

                for(i=0;i<3;i++)
		{
			Diffpressure[i]=buf[i];
			Temp[i]=buf[i+3];
			Scale[i]=buf[i+6];
		}

		//CRC Check Differential Pressure
		if(Diffpressure[2] != gencrc(Diffpressure, 2))
		{
                        perror("\nCRC Error DP");
                        exit(-1);

		}

		//CRC Check Temp
                if(Temp[2] != gencrc(Temp, 2))
                {
                        perror("\nCRC Error Temp");
                        exit(-1);

                }

		//Scale Factor calculation only by ReadOut
		if(byteread==9)
		{	//CRC Check Scale Factor
			if(Scale[2] != gencrc(Scale, 2))
                	{
                	        perror("\nCRC Error Scale");
                	        exit(-1);
                	}
			Scalefactor1= (((uint16_t) buf[6])<<8) + buf[7];

		}

		//Calculation Temperature
		Temp1=(((int16_t) buf[4])<<8) + buf[5];
		TempC1=((float) Temp1 ) / 200;

		//Calculation Differential Pressure in Pascal
		Diffpressure1= (((int16_t) buf[0])<<8) + buf[1];
		DiffpressurePa1=((float) Diffpressure1 ) / Scalefactor1;

		//Calculation Differential Pressure in mbar
		DiffpressureBar1=DiffpressurePa1*0.01;
		printf("\n%f Pa %f mbar %f °C", DiffpressurePa1, DiffpressureBar1, TempC1);

	        usleep(500000);

    	}

  	close(fd);
  	return 0;
}


int Sdp8xx(Command cmd)
{
        int status;
	//Write Upper 8 bits of the Command
	printf("0x%02x ",(const void *)(cmd >> 8));
	printf("0x%02x ",(const void *)(cmd & 0xFF));
        status=write(fd, (const void *)(cmd >> 8), 1);
	//Write Lower 8 bits of the Command
        status=write(fd, (const void *)(cmd & 0xFF), 1);
	//Sleep for Reset (Minimum 2ms)
	if(cmd==SOFT_RESET)
	{
		printf("SOFT_RESET");
        	usleep(2500);
	}

        return status;
}

int productid(void)
{
        uint8_t buf[18];
        int status;
	int i; //Zaehler
	//Read Product Identifier
        buf[0] = 0x36;
        buf[1] = 0x7C;
        status=write(fd, buf, 4);
        buf[0] = 0xE1;
        buf[1] = 0x02;
        status=write(fd, buf, 2);

        status=read(fd, buf, 18);
        if (status != 18)
        {
        	perror("\nError Read Product Identifier");
        	printf("\nNumber: %d",status);
        	exit(-1);
        }
	//Print Product Identifier
        printf("\nProduct Identifier: ");
        for(i=0;i<status;i++)
        {
                printf("0x%02x ", buf[i]);
        }
	return status;
}



int InitSdp8xx(Sensor sensor)
{
	// Open I2c
        if ((fd = open("/dev/i2c-1", O_RDWR)) < 0)
        {
                printf("\nError: Couldn't open device! %d\n", fd);
                return -1;
        }
	// Connect Device
        if (ioctl(fd, I2C_SLAVE, sensor) < 0)
        {
                printf("\nError: Couldn't find device on address!\n");
                return -2;
        }

        return 0;
}


