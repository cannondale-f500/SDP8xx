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

// Check I2C Devices with i2cdetect -y 1

#define SDP8X0_ADDR 0x26
#define SDP8X1_ADDR 0x25
#define CONT_MASSFLOW_AV 1
#define CONT_MASSFLOW_N 2
#define CONT_DIFFPRESS_AV 3
#define CONT_DIFFPRESS_N 4
#define CONT_STOP 5
#define TRIG_MASSFLOW_N 6
#define TRIG_MASSFLOW_ST 7
#define TRIG_DIFFPRESS_N 8
#define TRIG_DIFFPRESS_ST 9
#define SLEEP 10
#define PROD_ID 11

int InitSDP8X0();
int InitSDP8X1();

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
int openI2C(void);
int connectSDP8xx(int fd, uint8_t address);
void softReset(int fd);
int stopContMeas(int fd);
int cmd(int fd, uint8_t cmd1, uint8_t cmd2);
int productid(int fd);
	
int main()
{
	int status;
	int i;			//Counter
  	int fd;                 //Device-Handle
	int byteread;

  	uint8_t buf[18];        /* I/O buffer */

	uint8_t Diffpressure[3], Temp[3], Scale[3];

	int16_t Diffpressure1, Temp1;
        uint16_t Scalefactor1=0;

	float DiffpressurePa1, DiffpressureBar1, TempC1;

  	// open device on /dev/i2c-1
	fd=openI2C();

  	// Connect to SDP8xx
        if (ioctl(fd, I2C_SLAVE, SDP8X1_ADDR) < 0)
        {
                printf("\nError: Couldn't find device on address!\n");
                return -2;
        }

//	printf("\nConnect: %d ",status);

	status=productid(fd);
	//Read Product Identifier
/*        buf[0] = 0x36;
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
        }*/

	softReset(fd);

        // Stop Continuous Measurement
	status=stopContMeas(fd);

        // Start Continuous Measurement Differential pressure Update rate 0.5ms Average
        buf[0] = 0x36;
        buf[1] = 0x15;
        status=write(fd, buf, 2);
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


int openI2C(void)
{       int fd;
        if ((fd = open("/dev/i2c-1", O_RDWR)) < 0)
        {
                printf("\nError: Couldn't open device! %d\n", fd);
                return -1;
        }

        return fd;
}

int connectSDP8xx(int fd, uint8_t address)
{
        if (ioctl(fd, I2C_SLAVE, address) < 0)
        {
                printf("\nError: Couldn't find device on address!\n");
                return -2;
        }

        return 1;
}

void softReset(int fd)
{
        uint8_t buf[2];
        int status;
        buf[0] = 0x00;
        buf[1] = 0x06;
        status=write(fd, buf, 2);
        usleep(4000);
}

int stopContMeas(int fd)
{
        uint8_t buf[2];
        int status;
        buf[0] = 0x3F;
        buf[1] = 0xF9;
        status=write(fd, buf, 2);
        return status;
}

int cmd(int fd, uint8_t cmd1, uint8_t cmd2)
{
        uint8_t buf[2];
        int status;
        buf[0] = cmd1;
        buf[1] = cmd2;
        status=write(fd, buf, 2);
        return status;
}

int productid(int fd)
{
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
/*        printf("\nProduct Identifier: ");
        for(i=0;i<status;i++)
        {
                printf("0x%02x ", buf[i]);
        }*/
	return status;
}



