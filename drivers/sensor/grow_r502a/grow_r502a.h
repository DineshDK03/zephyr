/*
 * Copyright (c) 2021 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef GROW_R502A_H_
#define GROW_R502A_H_

/*Confirmation code’s definition*/
#define FPS_OK                      0x00 /*commad execution complete*/
#define FPS_PACKETRECIEVEERROR      0x01 /*error when receiving data package*/
#define FPS_NOFINGER                0x02 /*no finger on the sensor*/
#define FPS_ENROLLFAIL              0x03 /*fail to enroll the finger*/
#define FPS_IMAGEDISORDER           \
	0x06 /*fail to generate character file due to the over-disorderly fingerprint image*/
#define FPS_FEATUREFAIL             \
	0x07 /* fail to generate character file due to lackness of character point or
	      * over-smallness of fingerprint image.
	      */
#define FPS_NOMATCH                 0x08 /*finger doesn’t match*/
#define FPS_NOTFOUND                0x09 /*fail to find the matching finger;*/
#define FPS_ENROLLMISMATCH          0x0A /*fail to combine the character files*/
#define FPS_BADLOCATION             0x0B /*addressing PageID is beyond the finger library*/
#define FPS_TEMPINVALID             \
	0x0C /*error when reading template from library or the template is invalid*/
#define FPS_UPLOADFEATUREFAIL       0x0D /*error when uploading template*/
#define FPS_PACKETRESPONSEFAIL      0x0E /*Module can’t receive the following data packages*/
#define FPS_IMGUPLOADFAIL           0x0F /*error when uploading image*/
#define FPS_DELETEFAIL              0x10 /*fail to delete the template*/
#define FPS_LIBCLEARFAIL            0x11 /*fail to clear finger library*/
#define FPS_PASSFAIL                0x13 /*wrong password!*/
#define FPS_INVALIDIMAGE            \
	0x15 /*fail to generate the image for the lackness of valid primary image*/
#define FPS_FLASHERROR              0x18 /*error when writing flash*/
#define FPS_INVALIDREG              0x1A /*invalid register number*/
#define FPS_ADDRCODE                0x20
#define FPS_PASSVERIFY              0x21
#define FPS_STARTCODE               \
	0xEF01 /*Fixed value of 0xEF01; High byte transferred first*/

/*Package Identifier's definition*/
#define FPS_COMMANDPACKET           0x1 /*Command packet*/
#define FPS_DATAPACKET              \
	0x2 /*Data packet, must follow command packet or acknowledge packet*/
#define FPS_ACKPACKET               0x7 /*Acknowledge packet*/
#define FPS_ENDDATAPACKET           0x8 /*End of data packet*/

/*Instruction code's definition*/
#define FPS_GENIMAGE                0x01 /*Collect finger image*/
#define FPS_IMAGE2TZ                0x02 /*To generate character file from image*/
#define FPS_MATCH                   0x03 /*Carry out precise matching of two templates*/
#define FPS_SEARCH                  0x04 /*Search the finger library*/
#define FPS_REGMODEL                0x05 /*To combine character files and generate template*/
#define FPS_STORE                   0x06 /*To store template*/
#define FPS_LOAD                    0x07 /*To read/load template*/
#define FPS_UPCHAR                  0x08 /*To upload template*/
#define FPS_DOWNCHAR                0x09 /*To download template*/
#define FPS_IMGUPLOAD               0x0A /*To upload image*/
#define FPS_DELETE                  0x0C /*To delete template*/
#define FPS_EMPTYLIBRARY            0x0D /*To empty the library*/
#define FPS_SETSYSPARAM             0x0E /*To set system parameter*/
#define FPS_READSYSPARAM            0x0F /*To read system parameter*/
#define FPS_SETPASSWORD             0x12 /*To set password*/
#define FPS_VERIFYPASSWORD          0x13 /*To verify password*/
#define FPS_GETRANDOM               0x14 /*To generate a random code*/
#define FPS_TEMPLATECOUNT           0x1D /*To read finger template numbers*/
#define FPS_READTEMPLATEINDEX       0x1F /*Read fingerprint template index table*/
#define FPS_LED_CONFIG              0x35 /*Aura LED Config*/
#define FPS_CHECKSENSOR             0x36 /*Check sensor*/
#define FPS_SOFTRESET               0x3D /*Soft reset*/
#define FPS_HANDSHAKE               0x40 /*Handshake*/
#define FPS_BADPACKET               0xFE /* Bad packet was sent*/

#define FPS_DEFAULT_PASSWORD        0x00000000
#define FPS_DEFAULT_ADDRESS         0xFFFFFFFF

#define DEFAULT_TIMEOUT 1000 /*UART reading timeout in ms*/

struct sys_params {
	uint8_t return_data;
	uint16_t status_reg;
	uint16_t system_id;
	uint16_t capacity;
	uint16_t security_level;
	uint32_t device_addr;
	uint16_t packet_len;
	uint16_t baud_rate;
};
const struct sys_params defaults = {
	.status_reg = 0x0,
	.system_id = 0x0,
	.capacity = 200,
	.security_level = 5,
	.device_addr = 0xFFFFFFFF,
	.packet_len = 64,
	.baud_rate = 57600,
}

struct packet {
	uint16_t start_code;
	uint8_t address[4];
	uint8_t type;
	uint16_t length;
	uint8_t data[64];
};
static struct packet gen_packet, recv_packet;

struct grow_r502a_data {
	const struct device *uart_dev;

	struct sensor_trigger touch_trigger;
	sensor_trigger_handler_t touch_handler;

	uint32_t password;
	uint32_t address;

	uint16_t fingerID;
	uint16_t count;
};

#endif /*GROW_R502A_H_*/
