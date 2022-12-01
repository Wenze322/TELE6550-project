/*******************************************************************************
 * Copyright (c) 2012, 2022 IBM Corp., Ian Craggs
 *******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "MQTTClient.h"
////////
#include <string.h>
#include <time.h>
#include <errno.h>
#include <stdio.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/stat.h>
////////
#include <stdio.h>
#include <stdlib.h> 
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "wiringPi.h"     //添加库文件
#include "softPwm.h"     //添加库文件
  
#define MAX_SIZE 32
#define PWM_Pin 24         //定义PWM_Pin引脚

static int GetDuty(int temp);
unsigned int Duty = 0;
////////
#define DEV_FILE     "/dev/i2c-1"
#define SHT2X_ADDR             0x40
#define SHT2X_SOFTRESET        0xFE
#define SHT2X_MEAS_TEMPER      0xF3
#define SHT2X_MEAS_HUMIDITY    0xF5
////////
void msleep(unsigned int time);
int sht2x_init(void);
int sht2x_softreset(int fd);
int sht2x_send_cmd_wr(int fd, char *which);
int sht2x_get_temper_rh(int fd, float *temper, float *rh);
////////
#define ADDRESS     "tcp://mqtt.eclipseprojects.io:1883"
#define CLIENTID    "ExampleClientPub"
#define TOPIC       "MQTT Examples"
#define PAYLOAD     "Hello World!"
#define QOS         1
#define TIMEOUT     10000L



int main(int argc, char* argv[])
{
    MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;
    int   rc;
	int      fd = -1;
	int      ret = -1;
	float    temper;   
	float    rh;       //humidity
	char buff_send[30];
	fd = sht2x_init();
	if( fd < 0 )
	{
		printf("sht2x_init failure.\n");
		return -1;
	}
	
	ret = sht2x_softreset(fd);
	if( ret < 0 )
	{
		printf("sht2x_softreset failure.\n");
		return -2;
	}

    if ((rc = MQTTClient_create(&client, ADDRESS, CLIENTID,
        MQTTCLIENT_PERSISTENCE_NONE, NULL)) != MQTTCLIENT_SUCCESS)
    {
         printf("Failed to create client, return code %d\n", rc);
         exit(EXIT_FAILURE);
    }

    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to connect, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }


    double temp = 0;
    char buf[MAX_SIZE];

    wiringPiSetup();//初始化wiringPi
    softPwmCreate(PWM_Pin,0,100);//当前pwmRange为100，频率为100Hz，若pwmRange为50时，频率为200，若pwmRange为2时，频率为5000。
    softPwmWrite(PWM_Pin,50);//占空比 = value/pwmRange，当前占空比 = 50/100 = 50%

while (1)
{
                ret = sht2x_get_temper_rh(fd, &temper, &rh);
                if( ret < 0 )
                {
                        printf("sht2x_get_temper_rh failure.\n");
                        return -3;
                }
                printf("Temperature = %fC  Relative humidity = %f%\n", temper, rh);
               
        softPwmWrite(PWM_Pin,((int)temper-20)*20);
        printf(" Duty: %d\n",  (int)temper);

	sprintf(buff_send,"T = %f   H = %f%\n", temper, rh);


    pubmsg.payload = buff_send;
    pubmsg.payloadlen = (int)strlen(buff_send);
    pubmsg.qos = QOS;
    pubmsg.retained = 0;

    if ((rc = MQTTClient_publishMessage(client, TOPIC, &pubmsg, &token)) != MQTTCLIENT_SUCCESS)
    {
         printf("Failed to publish message, return code %d\n", rc);
         exit(EXIT_FAILURE);
    }

    printf("Waiting for up to %d seconds for publication of %s\n"
            "on topic %s for client with ClientID: %s\n",
            (int)(TIMEOUT/1000), PAYLOAD, TOPIC, CLIENTID);

    rc = MQTTClient_waitForCompletion(client, token, TIMEOUT);
    printf("Message with delivery token %d delivered\n", token);
	/* code */
}

    if ((rc = MQTTClient_disconnect(client, 10000)) != MQTTCLIENT_SUCCESS)
    	printf("Failed to disconnect, return code %d\n", rc);
    MQTTClient_destroy(&client);
	close(fd);
	
    return rc;
}



void msleep(unsigned int time)
{
	struct timespec sleeper, temp;
	sleeper.tv_sec = (time_t)(time/1000);
	sleeper.tv_nsec = (long)(time%1000)*1000000;
	nanosleep(&sleeper, &temp);
	
	return ; 
}
 

int sht2x_init(void)
{
	int    fd = -1;
	
	fd = open(DEV_FILE, O_RDWR);
	if( fd < 0 )
	{
		printf("i2c device open failure: %s\n", strerror(errno));
		return -1;
	}
	
	ioctl(fd, I2C_TENBIT, 0);
	ioctl(fd, I2C_SLAVE, SHT2X_ADDR);
	
	return fd;
}
 

int sht2x_softreset(int fd)
{
	int            ret = -1;
    uint8_t        buf[2] = {0};
 
	if( fd < 0 )
	{
		printf("input the invalid argument.\n");
		return -1;
	}	
	
    buf[0] = SHT2X_SOFTRESET;
	ret = write(fd, buf, 1);
	if( ret < 0 )
	{
		printf("write softrest cmd to sht2x failure.\n");
		return -2;
	}
	msleep(50);
		
	return 0;
}

 
int sht2x_send_cmd_wr(int fd, char *which)
{
	int              ret = -1;
	uint8_t          buf[2] = {0};
 
	if( fd < 0 )
	{
		printf("input the invalid argument.\n");
		return -1;
	}
	
	if( strcmp(which, "temper") == 0 )
	{
        buf[0] = SHT2X_MEAS_TEMPER;
        ret = write(fd, buf, 1);
		if( ret < 0 )
		{
			printf("write temper cmd to sht2x failure.\n");
			return -2;
		}
		msleep(85);  //datasheet typ=66, max=85
	}
	else if( strcmp(which, "rh") == 0 )
	{
        buf[0] = SHT2X_MEAS_HUMIDITY;
        ret = write(fd, buf, 1);
		if( ret < 0 )
		{
			printf("write humidity cmd to sht2x failure.\n");
			return -3;
		}
		msleep(29);  //datasheet typ=22, max=29
	}
	
	return 0;
}
 

int sht2x_get_temper_rh(int fd, float *temper, float *rh)
{
	uint8_t              buf[4] = {0};
	int                  ret = -1;
 
	if( fd<0 || !temper || !rh )
	{
		printf("input the invalid arguments.\n");
		return -1;
	}
	
	ret = sht2x_send_cmd_wr(fd, "temper");
	if( ret < 0 )
	{
		printf("sht2x_send_cmd_wr temper failure.\n");
		return -2;
	}
	else
	{
        ret = read(fd, buf, 3);
		if( ret < 0 )
		{
			printf("get the temper failure.\n");
			return -3;
		}
		*temper = 175.72 * (((((int) buf[0]) << 8) + buf[1]) / 65536.0) - 46.85;
	}
 
	ret = sht2x_send_cmd_wr(fd, "rh");
	if( ret < 0 )
	{
		printf("sht2x_send_cmd_wr rh failure.\n");
		return -2;
	}
	else
	{
        read(fd, buf, 3);
		if( ret < 0 )
		{
			printf("get the temper failure.\n");
			return -3;
		}
		*rh = 125 * (((((int) buf[0]) << 8) + buf[1]) / 65536.0) - 6;
	}
	return 0;
}
  
  
static int GetDuty(int temp)
{
    if(temp >= 60)
    {
        return 100;
    }
    else if(temp >= 55)
    {
        return 90;
    }
    else if(temp >= 53)
    {
        return 80;
    }
    else if(temp >= 50)
    {
        return 70;
    }
    else if(temp >= 45)
    {
        return 60;
    }
    else
    {
        return 50;
    }
}