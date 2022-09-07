#include <Arduino.h>
#include "gps.h"
#include "imu.h"
#include "io.h"

//UBX protocol implementation

void ubx_print_raw(ubx_t* ubx)
{
uint8_t* raw_bytes=(uint8_t*)ubx;
	for(int i=0;i<ubx->length+6;i++)
	{
	usb_serial->writef("%2x ",raw_bytes[i]);
		if(i%16==15)usb_serial->write("\r\n");
	}
usb_serial->flush();
}

uint16_t ubx_checksum(ubx_t* ubx)
{
uint8_t ck_a=0;
uint8_t ck_b=0;
	for(int i=0;i<ubx->length+4;i++)
	{
	ck_a+=((uint8_t*)ubx)[i];
	ck_b+=ck_a;
	}
return ck_a|(ck_b<<8);
}

void ubx_send(ubx_t* ubx)
{
//TODO check bounds
uint8_t packet[256]={0xB5,0x62};
//Write header
packet[0]=0xB5;
packet[1]=0x62;
//Write message ID
packet[2]=ubx->clss;
packet[3]=ubx->id;
//Write message length

packet[4]=ubx->length&0xFF;
packet[5]=ubx->length>>8;
//Write data
memcpy(packet+6,&(ubx->nav_pvt),ubx->length);
//Calculate checksum
uint16_t chk=ubx_checksum(ubx);
//Write checksum
packet[ubx->length+6]=chk&0xFF;
packet[ubx->length+7]=chk>>8;

//Send packet
size_t written=serial_ports[1]->write(packet,ubx->length+8);
serial_ports[1]->flush();
log_message(LOG_GPS|LOG_DEBUG,"Sent packet of size %d (result %d)",ubx->length+8,written);
}

bool ubx_receive(ubx_t* ubx)
{
//Discard bytes until valid UBX header is encountered
uint8_t ubx_header[6];
	while(serial_ports[1]->peek(ubx_header,2)==2&&(ubx_header[0]!=0xB5||ubx_header[1]!=0x62))
	{
	serial_ports[1]->discard(1);
	}

//Read the 6 byte UBX header, or terminate if we don't have enough bytes available
	if(serial_ports[1]->peek(ubx_header,6)==6)
	{
	ubx->clss=ubx_header[2];
	ubx->id=ubx_header[3];
	ubx->length=*((uint16_t*)(ubx_header+4));
	}else return false;

//Check if complete packet is available
	if(serial_ports[1]->available()<ubx->length+8U)return false;
serial_ports[1]->discard(6);

//Read payload
size_t bytes_read=0;
	//Parse UBX-NAV-PVT
	if(ubx->clss==1&&ubx->id==7&&ubx->length==sizeof(ubx_nav_pvt_t))bytes_read=serial_ports[1]->read((uint8_t*)(&(ubx->nav_pvt)),sizeof(ubx_nav_pvt_t));
	//UBX-ACK-NACK
	else if(ubx->clss==5&&ubx->id==0&&ubx->length==sizeof(ubx_ack_nack_t))bytes_read=serial_ports[1]->read((uint8_t*)(&(ubx->ack_nack)),sizeof(ubx_ack_nack_t));
	//UBX-ACK-ACK
	else if(ubx->clss==5&&ubx->id==1&&ubx->length==sizeof(ubx_ack_ack_t))bytes_read=serial_ports[1]->read((uint8_t*)(&(ubx->ack_ack)),sizeof(ubx_ack_ack_t));
	//Unrecognized packet
	else
	{
	Serial.println("Unrecognized UBX packet\n");
	ubx_print_raw(ubx);
	usb_serial->write("\r\n");
	serial_ports[1]->discard(ubx->length+2);
	return false;
	}

	//I don't think this should ever happen, but we check just in case.
	if(bytes_read!=ubx->length)
	{
	log_message(LOG_GPS|LOG_ERROR,"Failed to read UBX data (expected %d bytes but received %d)");
	ubx_print_raw(ubx);
	return false;
	}

uint16_t expected_chk=serial_ports[1]->read_uint16();
uint16_t chk=ubx_checksum(ubx);
	if(chk!=expected_chk)
	{
	log_message(LOG_GPS|LOG_ERROR,"Checksum mismatch for UBX packet (class %d id %d) - expected %x but got %x",ubx->clss,ubx->id,expected_chk,chk);
	ubx_print_raw(ubx);
	return false;
	}
return true;
}

bool ubx_wait_ack(ubx_t* ubx_out)
{
//Wait for response
	for(int i=0;i<100;i++)
	{
	ubx_t ubx;
		while(ubx_receive(&ubx))
		{
			if(ubx.clss==5&&ubx.id==0&&ubx.ack_nack.clss==ubx_out->clss&&ubx.ack_nack.id==ubx_out->id)return false;//Message not acknowledged
			else if(ubx.clss==5&&ubx.id==1&&ubx.ack_ack.clss==ubx_out->clss&&ubx.ack_ack.id==ubx_out->id)return true;//Message acknowledged
		}
	delay(10);
	}
//Timed out; no response
log_message(LOG_GPS|LOG_DEBUG,"Timed out while waiting for acknowledgement");
return false;
}


//Public GPS functions
enum
{
GPS_INIT_CFG_PRT,
GPS_INIT_CFG_MSG,
GPS_INIT_CFG_RATE,
GPS_INIT_CFG_NAV5,
GPS_INIT_COMPLETE,
};

bool gps_init(bool gps_already_configured)
{

	//If the Teensy is restarted without powering off the GPS, the GPS configuration commands tend to fail
	//So it's convenient to be able to skip these steps when debugging
	if(gps_already_configured==true)
	{
	//Reinitialize serial port with new baud rate
	serial_ports[1]->close();
		if(!serial_ports[1]->open(57600,O_READ_WRITE))
		{
		log_message(LOG_GPS|LOG_ERROR,"Serial port not available");
		return false;
		}
	return true;
	}


static int gps_init_state=GPS_INIT_CFG_PRT;


log_message(LOG_GPS|LOG_DEBUG,"Starting GPS configuration");
	if(gps_init_state==GPS_INIT_CFG_PRT)
	{
		//Init serial port
		if(!serial_ports[1]->open(9600,O_READ_WRITE))
		{
		log_message(LOG_GPS|LOG_ERROR,"Serial port not available");
		return false;
		}
	log_message(LOG_GPS|LOG_DEBUG,"Serial started with baud rate 9600");
	//Change baud rate to 57600 and set protocol to UBX
	ubx_t ubx={0};
	ubx.clss=6;
	ubx.id=0;
	ubx.length=20;
	ubx.cfg_prt.port=1;
	ubx.cfg_prt.mode=0x8D0;
	ubx.cfg_prt.baud_rate=57600;
	ubx.cfg_prt.in_proto_mask=1;
	ubx.cfg_prt.out_proto_mask=1;
	ubx_send(&ubx);

	//Reinitialize serial port with new baud rate
	serial_ports[1]->close();
		if(!serial_ports[1]->open(57600,O_READ_WRITE))
		{
		log_message(LOG_GPS|LOG_ERROR,"Failed to set baud rate to 57600");
		return false;
		}
	log_message(LOG_GPS|LOG_DEBUG,"Serial restarted with baud rate 576000");

	//TODO the acknowledgement here doesn't seem to be correctly recieved because of baud rate change

	/*
	int acknowledged=ubx_wait_ack(&ubx);
		if(!acknowledged)
		{
		Serial.write("Failed to set UBX mode\n");
		}*/

	gps_init_state=GPS_INIT_CFG_MSG;
	}

//Enable UBX-NAV-PVT message
	if(gps_init_state==GPS_INIT_CFG_MSG)
	{
	ubx_t ubx={0};
	ubx.clss=6;
	ubx.id=1;
	ubx.length=3;
	ubx.cfg_msg.clss=1;
	ubx.cfg_msg.id=7;
	ubx.cfg_msg.rate=1;
	ubx_send(&ubx);
		if(!ubx_wait_ack(&ubx))
		{
		log_message(LOG_GPS|LOG_ERROR,"CFG-MSG message was not acknowledged");
		return false;
		}
	gps_init_state=GPS_INIT_CFG_RATE;
	}

//Change output data rate to 10 points per second
	if(gps_init_state==GPS_INIT_CFG_RATE)
	{
	ubx_t ubx={0};
	ubx.clss=6;
	ubx.id=8;
	ubx.length=6;
	ubx.cfg_rate.measurement_rate=100;
	ubx.cfg_rate.navigation_rate=1;
	ubx.cfg_rate.time_reference=0;
	ubx_send(&ubx);
		if(!ubx_wait_ack(&ubx))
		{
		log_message(LOG_GPS|LOG_ERROR,"CFG-RATE message was not acknowledged");
		return false;
		}
	gps_init_state=GPS_INIT_CFG_NAV5;
	}

//Set dynamic model to 4G airborne and position fix mode to 3D only
	if(gps_init_state==GPS_INIT_CFG_NAV5)
	{
	ubx_t ubx={0};
	ubx.clss=6;
	ubx.id=0x24;
	ubx.length=36;
	ubx.cfg_nav5.mask=UBX_CFG_NAV5_DYNAMIC_MODEL|UBX_CFG_NAV5_POSITION_FIX_MODE;
	ubx.cfg_nav5.dynModel=UBX_DYNAMIC_MODEL_AIRBORNE_4G;
	ubx.cfg_nav5.fixMode=UBX_FIX_MODE_3D_ONLY;
	ubx_send(&ubx);
		if(!ubx_wait_ack(&ubx))
		{
		log_message(LOG_GPS|LOG_ERROR,"CFG-NAV5 message was not acknowledged");
		return false;
		}
	gps_init_state=GPS_INIT_COMPLETE;
	}

//TODO consider configuring SBAS (UBX-CFG-SBAS?)
return true;
}

