#include <Arduino.h>

#include <string.h>

#include <WiFi.h>
#include "AsyncMqttClient.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#include <IRremoteESP8266.h>
#include <IRsend.h>

#define WIFI_SSID "TP-Link_F9A6"
#define WIFI_PASSWORD "96683026"

#define MQTT_HOST IPAddress(5, 196, 95, 208)
#define MQTT_PORT 1883

#define MQTT_TOPIC_ROOT  "esp32/myowndemo/+"
#define MQTT_TOPIC_TEMPERATURE  "esp32/myowndemo/temperature"
#define MQTT_TOPIC_PRESSURE  "esp32/myowndemo/pressure"
#define MQTT_TOPIC_ALTITUDE  "esp32/myowndemo/altitude"

#define MQTT_TOPIC_AC  "esp32/myowndemo/ac/#"
#define MQTT_TOPIC_AC_RUN  "esp32/myowndemo/ac/run"
#define MQTT_TOPIC_AC_MODE  "esp32/myowndemo/ac/mode"
#define MQTT_TOPIC_AC_TEMP_HEAT  "esp32/myowndemo/ac/temp/heat"
#define MQTT_TOPIC_AC_TEMP_COOL  "esp32/myowndemo/ac/temp/cool"

#define AC_MODE_COOL  "cool"
#define AC_MODE_HEAT  "heat"
#define AC_MODE_AVTO  "avto"
#define AC_MODE_DRY  "dry"

#define AC_RUN_ON  "on"
#define AC_RUN_OFF  "off"

#define NUM_TOPIC 4
#define NUM_ACMODE 4
#define NUM_TEMP 12

uint16_t acOff[101] = {5940, 7470,  540, 3436,  514, 3464,  512, 3464,  538, 3440,  512, 3464,  540, 1436,  540, 3436,  514, 3464,  514, 1460,  514, 1488,  514, 1460,  514, 1462,  512, 1490,  512, 3464,  514, 1462,  536, 1438,  514, 1462,  512, 3464,  540, 3438,  514, 3464,  540, 3438,  518, 3460,  514, 3464,  538, 3438,  538, 3438,  540, 1436,  514, 1488,  538, 1438,  512, 1462,  514, 1488,  514, 1462,  540, 1436,  514, 3464,  514, 1460,  538, 3440,  538, 1462,  514, 3464,  514, 1462,  512, 1464,  512, 1464,  538, 1464,  512, 3464,  514, 1462,  514, 3464,  538, 1436,  514, 3464,  538, 3438,  512, 3464,  514, 7468,  512};  // TRANSCOLD FB7FA8
uint16_t acOn[101] = {5964, 7446,  538, 3438,  512, 3466,  514, 3464,  540, 3438,  512, 3466,  512, 1490,  538, 3440,  514, 3436,  514, 1488,  514, 1462,  512, 1462,  538, 1464,  540, 1434,  540, 3438,  512, 1462,  514, 1462,  538, 1462,  514, 3464,  514, 3462,  540, 1436,  514, 1462,  514, 3464,  514, 3462,  516, 3462,  514, 3464,  514, 1462,  538, 1462,  514, 3464,  512, 3466,  514, 1460,  522, 1454,  514, 1460,  516, 3462,  514, 1488,  514, 3464,  540, 1436,  512, 3464,  538, 1436,  516, 1486,  514, 1460,  540, 1436,  538, 3438,  540, 1434,  514, 3464,  514, 1488,  514, 3464,  540, 3436,  514, 3438,  514, 7466,  516};  // TRANSCOLD FB67A8

//cool raw temps
uint16_t cool_19[101] = {5938, 7470,  514, 3464,  512, 3466,  510, 3466,  536, 3440,  538, 3440,  512, 1462,  512, 1462,  512, 3464,  538, 1438,  538, 1464,  536, 1440,  536, 1438,  536, 1464,  536, 3440,  514, 3464,  514, 1462,  512, 1460,  516, 3464,  538, 3440,  536, 1438,  538, 3440,  512, 1490,  512, 3466,  512, 3438,  512, 3466,  536, 1464,  538, 1438,  538, 3440,  538, 1438,  512, 3464,  514, 1488,  512, 1464,  512, 3466,  536, 1438,  536, 3440,  538, 1438,  538, 3438,  538, 1466,  512, 1460,  514, 1462,  512, 1464,  512, 3464,  510, 1492,  536, 3440,  538, 1438,  536, 3440,  538, 3440,  514, 3464,  512, 7470,  512};  // TRANSCOLD F96BA8
uint16_t cool_20[101] = {5964, 7444,  512, 3464,  538, 3442,  536, 3440,  538, 3440,  538, 3440,  514, 1488,  514, 1460,  514, 3464,  538, 1438,  536, 1438,  536, 1466,  538, 1436,  538, 1438,  538, 3438,  536, 3442,  536, 1438,  512, 1488,  512, 3466,  512, 3466,  536, 1440,  536, 1440,  538, 1464,  538, 3438,  538, 3414,  538, 3440,  538, 1464,  538, 1436,  538, 3440,  538, 3440,  514, 3464,  512, 1462,  536, 1438,  514, 3464,  536, 1466,  538, 3440,  538, 1436,  538, 3438,  512, 1462,  536, 1464,  514, 1462,  512, 1464,  512, 3466,  512, 1464,  536, 3442,  538, 1462,  538, 3440,  536, 3442,  512, 3438,  538, 7444,  538};  // TRANSCOLD F963A8
uint16_t cool_21[101] = {5940, 7470,  514, 3464,  536, 3440,  538, 3440,  536, 3442,  536, 3440,  514, 1462,  514, 1462,  514, 3464,  514, 1462,  536, 1466,  538, 1438,  536, 1438,  538, 1462,  538, 3440,  538, 3440,  512, 1462,  512, 1462,  514, 3464,  538, 3440,  536, 1438,  538, 3440,  538, 3438,  538, 1464,  536, 3412,  538, 3442,  536, 1466,  538, 1438,  536, 3442,  538, 1436,  538, 1462,  538, 3440,  536, 1438,  512, 3464,  512, 1462,  536, 3442,  536, 1438,  536, 3442,  512, 1488,  536, 1438,  514, 1462,  512, 1462,  514, 3464,  536, 1466,  538, 3440,  536, 1438,  536, 3440,  536, 3442,  512, 3464,  536, 7446,  538};  // TRANSCOLD F96DA8
uint16_t cool_22[101] = {5914, 7498,  534, 3444,  536, 3440,  536, 3440,  512, 3464,  514, 3466,  534, 1440,  512, 1490,  534, 3416,  536, 1464,  538, 1438,  536, 1440,  536, 1466,  536, 1438,  512, 3466,  512, 3466,  510, 1464,  534, 1440,  534, 3442,  536, 3440,  512, 1490,  512, 1464,  512, 3466,  536, 1440,  534, 3442,  536, 3442,  536, 1440,  534, 1466,  536, 3440,  512, 3466,  536, 1440,  536, 3442,  536, 1440,  536, 3440,  534, 1440,  536, 3442,  512, 1490,  512, 3464,  512, 1464,  510, 1464,  534, 1440,  534, 1468,  534, 3442,  536, 1438,  536, 3442,  512, 1462,  512, 3466,  510, 3468,  512, 3466,  536, 7444,  536};  // TRANSCOLD F965A8
uint16_t cool_23[101] = {5966, 7442,  538, 3440,  538, 3440,  538, 3438,  538, 3440,  536, 3440,  514, 1488,  512, 1462,  538, 3440,  538, 1438,  536, 1440,  538, 1464,  536, 1438,  538, 1438,  538, 3438,  512, 3466,  512, 1462,  526, 1474,  538, 3440,  538, 3440,  538, 1438,  536, 3440,  538, 1438,  536, 1466,  536, 3414,  512, 3466,  536, 1464,  538, 1436,  538, 3438,  538, 1438,  538, 3438,  514, 3464,  514, 1462,  536, 3442,  536, 1466,  536, 3440,  538, 1438,  538, 3440,  536, 1438,  514, 1488,  538, 1436,  516, 1462,  536, 3442,  536, 1438,  536, 3440,  538, 1464,  538, 3440,  536, 3440,  536, 3414,  514, 7468,  538};  // TRANSCOLD F969A8
uint16_t cool_24[101] = {5938, 7472,  538, 3440,  538, 3438,  538, 3438,  538, 3440,  514, 3466,  536, 1438,  536, 1466,  536, 3414,  534, 1466,  536, 1438,  536, 1438,  514, 1488,  514, 1460,  514, 3464,  540, 3440,  536, 1438,  538, 1438,  536, 3440,  536, 3440,  514, 1488,  512, 1462,  514, 1462,  512, 1488,  538, 3412,  536, 3442,  536, 1466,  536, 1438,  536, 3440,  536, 3440,  514, 3464,  512, 3466,  536, 1438,  538, 3438,  538, 1438,  538, 3440,  536, 1466,  512, 3464,  512, 1464,  538, 1438,  536, 1438,  538, 1464,  536, 3440,  538, 1438,  538, 3440,  512, 1462,  512, 3464,  536, 3442,  536, 3442,  536, 7444,  538};  // TRANSCOLD F961A8
uint16_t cool_25[101] = {5940, 7470,  536, 3440,  512, 3466,  512, 3466,  538, 3440,  538, 3440,  536, 1438,  536, 1438,  536, 3440,  514, 1462,  512, 1488,  536, 1440,  512, 1462,  514, 1490,  536, 3442,  536, 3440,  538, 1438,  536, 1438,  536, 3440,  512, 3466,  512, 1462,  538, 3440,  538, 3438,  538, 3440,  538, 1438,  516, 3462,  512, 1488,  512, 1464,  536, 3442,  534, 1440,  536, 1466,  538, 1436,  538, 3438,  538, 3440,  512, 1462,  514, 3464,  512, 1464,  538, 3440,  536, 1466,  536, 1438,  538, 1436,  538, 1438,  538, 3440,  514, 1488,  536, 3440,  538, 1438,  536, 3442,  536, 3442,  536, 3440,  514, 7468,  538};  // TRANSCOLD F96EA8
uint16_t cool_26[101] = {5914, 7496,  512, 3466,  512, 3466,  514, 3464,  534, 3442,  534, 3444,  512, 1464,  512, 1490,  512, 3438,  514, 1490,  510, 1466,  534, 1440,  536, 1466,  536, 1440,  536, 3442,  534, 3442,  510, 1464,  512, 1464,  510, 3464,  514, 3466,  534, 1468,  512, 1464,  536, 3442,  536, 3442,  512, 1464,  512, 3466,  512, 1462,  512, 1492,  510, 3466,  536, 3442,  536, 1440,  510, 1464,  510, 3466,  512, 3466,  512, 1464,  510, 3466,  512, 1490,  534, 3442,  536, 1438,  536, 1440,  512, 1464,  510, 1490,  512, 3466,  512, 1464,  512, 3466,  510, 1466,  536, 3440,  536, 3440,  536, 3442,  512, 7470,  514};  // TRANSCOLD F966A8
uint16_t cool_27[101] = {5916, 7496,  538, 3440,  538, 3438,  538, 3440,  536, 3442,  536, 3440,  538, 1438,  538, 1464,  536, 3414,  538, 1468,  534, 1438,  538, 1436,  538, 1464,  536, 1438,  512, 3466,  536, 3440,  536, 1440,  536, 1438,  538, 3440,  538, 3440,  514, 1488,  512, 3464,  514, 1462,  536, 3442,  534, 1440,  536, 3440,  538, 1438,  538, 1464,  536, 3442,  536, 1438,  514, 3464,  534, 1442,  538, 3440,  534, 3442,  538, 1436,  538, 3440,  538, 1464,  536, 3440,  514, 1462,  538, 1438,  536, 1438,  536, 1466,  536, 3440,  536, 1438,  512, 3464,  512, 1462,  512, 3462,  540, 3440,  538, 3440,  538, 7442,  514};  // TRANSCOLD F96AA8
uint16_t cool_28[101] = {5938, 7472,  538, 3440,  538, 3440,  512, 3466,  538, 3440,  536, 3440,  538, 1436,  538, 1462,  538, 3414,  538, 1464,  514, 1462,  514, 1462,  514, 1488,  538, 1438,  536, 3440,  536, 3440,  538, 1436,  538, 1438,  538, 3438,  514, 3464,  512, 1490,  512, 1464,  512, 1464,  536, 3442,  538, 1436,  538, 3438,  538, 1466,  512, 1462,  538, 3438,  540, 3440,  538, 3440,  536, 1440,  536, 3440,  514, 3464,  512, 1462,  512, 3464,  536, 1466,  538, 3440,  538, 1438,  536, 1438,  536, 1438,  536, 1466,  512, 3464,  538, 1438,  512, 3464,  512, 1464,  536, 3442,  538, 3440,  536, 3440,  512, 7470,  514};  // TRANSCOLD F962A8
uint16_t cool_29[101] = {5914, 7496,  536, 3440,  538, 3440,  538, 3440,  536, 3440,  514, 3464,  512, 1462,  538, 1438,  534, 3442,  538, 1438,  538, 1462,  538, 1438,  538, 1436,  538, 1464,  538, 3440,  538, 3440,  538, 1436,  538, 1438,  536, 3440,  536, 3440,  514, 1462,  512, 3464,  540, 3438,  538, 1464,  538, 1438,  536, 3440,  536, 1438,  538, 1438,  538, 3440,  512, 1488,  512, 1462,  514, 3466,  536, 3442,  536, 3440,  538, 1438,  536, 3442,  512, 1464,  512, 3466,  536, 1466,  536, 1438,  536, 1438,  536, 1438,  538, 3438,  536, 1464,  538, 3440,  514, 1462,  538, 3440,  538, 3440,  538, 3438,  538, 7444,  514};  // TRANSCOLD F96CA8
uint16_t cool_30[101] = {5940, 7470,  510, 3468,  534, 3442,  536, 3442,  512, 3466,  512, 3464,  536, 1466,  510, 1464,  534, 3444,  534, 1440,  534, 1440,  534, 1466,  512, 1464,  512, 1464,  512, 3466,  512, 3466,  510, 1466,  534, 1468,  536, 3442,  536, 3442,  510, 1464,  512, 1464,  512, 3466,  512, 1488,  514, 1462,  536, 3442,  536, 1440,  536, 1440,  536, 3442,  512, 3466,  512, 1490,  512, 3466,  512, 3440,  534, 3442,  536, 1466,  536, 3442,  512, 1464,  512, 3466,  512, 1464,  510, 1490,  512, 1464,  508, 1466,  532, 3444,  512, 1464,  536, 3440,  536, 1466,  512, 3466,  510, 3468,  510, 3440,  508, 7472,  536};  // TRANSCOLD F964A8
//heat raw temps
uint16_t heat_19[101] =  {5850, 7530,  504, 3524,  452, 3554,  398, 3528,  474, 3476,  498, 3502,  452, 1522,  476, 1498,  452, 3578,  398, 1578,  398, 1604,  422, 1526,  450, 1526,  450, 1526,  476, 3500,  478, 3500,  476, 1498,  452, 3550,  426, 3584,  420, 1526,  448, 1526,  448, 3502,  478, 1524,  476, 3500,  476, 3472,  476, 1528,  450, 1550,  424, 3556,  424, 3526,  476, 1498,  478, 3498,  478, 1550,  450, 1526,  424, 3554,  424, 1552,  424, 3528,  476, 1476,  498, 3476,  504, 1524,  476, 1498,  478, 1498,  478, 1524,  450, 3580,  370, 1580,  422, 3528,  474, 1500,  450, 3504,  500, 3500,  478, 3500,  476, 7532,  424};  // UNKNOWN F7F6868D
uint16_t heat_20[101] = {5904, 7506,  476, 3526,  476, 3500,  452, 3554,  424, 3556,  422, 3528,  474, 1528,  476, 1498,  476, 3500,  476, 1498,  476, 1498,  452, 1550,  452, 1548,  426, 1576,  422, 3530,  448, 3502,  476, 1500,  476, 3498,  478, 3500,  474, 1526,  454, 1548,  426, 1550,  424, 1604,  398, 3528,  476, 3476,  476, 1524,  478, 1524,  426, 3552,  448, 3530,  450, 3502,  450, 3526,  474, 1476,  502, 1472,  502, 3498,  478, 1524,  476, 3554,  398, 1576,  398, 3530,  448, 1524,  450, 1552,  474, 1500,  476, 1500,  476, 3500,  478, 1496,  478, 3526,  450, 1606,  370, 3554,  422, 3528,  450, 3474,  502, 7506,  476};  // UNKNOWN 29FDDD69
uint16_t heat_21[101] = {5848, 7532,  474, 3558,  398, 3554,  448, 3502,  476, 3478,  500, 3500,  474, 1500,  474, 1528,  450, 3554,  398, 1632,  370, 1578,  422, 1528,  448, 1528,  474, 1502,  474, 3478,  500, 3500,  474, 1500,  450, 3552,  424, 3608,  370, 1552,  446, 1530,  472, 3504,  476, 3500,  476, 1524,  450, 3528,  430, 1546,  424, 1552,  424, 3504,  500, 3476,  502, 1500,  500, 1498,  476, 3526,  450, 1526,  448, 3582,  370, 1580,  396, 3506,  472, 1552,  448, 3504,  500, 1498,  478, 1498,  476, 1498,  478, 1550,  450, 3582,  370, 1578,  396, 3530,  448, 1528,  450, 3526,  476, 3502,  476, 3500,  476, 7510,  452};  // UNKNOWN 2D382ED
uint16_t heat_22[101] = {5900, 7530,  476, 3528,  424, 3528,  474, 3478,  500, 3474,  504, 3498,  478, 1496,  478, 1524,  444, 3588,  394, 1554,  396, 1578,  422, 1526,  450, 1524,  448, 1552,  452, 3526,  476, 3498,  480, 1498,  476, 3582,  394, 3530,  422, 1528,  448, 1526,  448, 1552,  450, 3504,  500, 1498,  476, 3502,  476, 1524,  450, 1552,  424, 3504,  474, 3502,  476, 3476,  500, 1526,  476, 3500,  452, 1548,  426, 3580,  398, 1552,  424, 3528,  474, 1500,  474, 3476,  502, 1524,  476, 1498,  476, 1498,  452, 1524,  448, 3554,  426, 1630,  368, 3556,  448, 1528,  448, 3502,  476, 3502,  476, 3500,  476, 7534,  450};  // UNKNOWN 6CA848C9
uint16_t heat_23[101] = {5928, 7506,  474, 3530,  424, 3528,  476, 3500,  476, 3502,  474, 3526,  426, 1576,  426, 1526,  450, 3526,  476, 1478,  496, 1474,  502, 1500,  502, 1496,  478, 1496,  478, 3552,  424, 3556,  396, 1554,  420, 3530,  448, 3528,  452, 1550,  478, 1496,  478, 3500,  476, 1524,  450, 1578,  424, 3530,  396, 1580,  422, 1528,  448, 3528,  450, 3526,  476, 1498,  476, 3526,  450, 3530,  448, 1554,  422, 3502,  476, 1526,  474, 3500,  474, 1500,  476, 3500,  478, 1524,  426, 1606,  396, 1606,  370, 1578,  422, 3504,  474, 1500,  476, 3476,  502, 1524,  476, 3500,  452, 3552,  424, 3582,  370, 7508,  500};  // UNKNOWN 25A58072
uint16_t heat_24[101] = {5924, 7532,  424, 3556,  422, 3510,  468, 3502,  502, 3476,  502, 3526,  450, 1580,  420, 1580,  372, 3556,  420, 1528,  448, 1526,  448, 1552,  448, 1502,  498, 1502,  476, 3500,  478, 3500,  476, 1552,  422, 3584,  368, 3530,  448, 1552,  450, 1524,  474, 1500,  476, 1524,  474, 1500,  476, 3526,  450, 1526,  424, 1578,  398, 3528,  476, 3478,  500, 3474,  502, 3500,  476, 3500,  452, 1550,  424, 3582,  398, 1578,  448, 3480,  498, 1500,  476, 3500,  476, 1500,  474, 1526,  474, 1500,  452, 1550,  424, 3608,  370, 1578,  422, 3502,  476, 1526,  476, 3500,  476, 3528,  448, 3502,  424, 7510,  498};  // UNKNOWN 48DDBD85
uint16_t heat_25[101] = {5850, 7508,  502, 3500,  476, 3502,  450, 3554,  424, 3554,  450, 3502,  474, 1500,  476, 1498,  476, 3502,  448, 1552,  424, 1576,  426, 1552,  422, 1552,  424, 1554,  474, 3478,  502, 3474,  502, 1496,  478, 3500,  478, 3606,  346, 1580,  394, 1554,  420, 3528,  450, 3502,  502, 3500,  478, 1496,  478, 1524,  474, 1526,  450, 3554,  396, 3530,  448, 1528,  448, 1552,  448, 1526,  452, 3524,  478, 3498,  478, 1524,  426, 3580,  422, 1528,  448, 3480,  498, 1526,  476, 1500,  474, 1500,  476, 1498,  474, 3502,  450, 1574,  426, 3634,  344, 1580,  422, 3502,  476, 3502,  476, 3500,  476, 7530,  424};  // UNKNOWN 33EA190
uint16_t heat_26[101] = {5924, 7510,  446, 3556,  420, 3530,  474, 3476,  502, 3476,  500, 3502,  476, 1578,  422, 1580,  370, 3556,  422, 1528,  446, 1528,  448, 1528,  472, 1526,  476, 1498,  474, 3502,  476, 3526,  450, 1550,  424, 3556,  396, 3530,  448, 1554,  448, 1526,  448, 1502,  498, 3502,  476, 3502,  476, 1526,  448, 1582,  422, 1554,  422, 3504,  476, 3476,  500, 3500,  476, 1500,  474, 1526,  474, 3506,  422, 3584,  394, 1580,  448, 3478,  498, 1502,  476, 3476,  500, 1498,  476, 1526,  474, 1498,  452, 1550,  424, 3582,  396, 1554,  446, 3506,  474, 1528,  474, 3502,  476, 3500,  474, 3476,  450, 7534,  474};  // UNKNOWN CCCC13A1
uint16_t heat_27[101] = {5902, 7504,  502, 3502,  476, 3528,  424, 3580,  424, 3504,  474, 3478,  500, 1498,  476, 1526,  474, 3474,  456, 1546,  452, 1548,  426, 1550,  424, 1608,  396, 1578,  420, 3506,  474, 3502,  474, 1500,  476, 3500,  476, 3502,  450, 1524,  450, 1578,  424, 3556,  448, 1526,  448, 3504,  476, 1500,  474, 1524,  450, 1552,  450, 3528,  424, 3528,  448, 1526,  450, 3502,  502, 1472,  502, 3498,  478, 3526,  426, 1576,  422, 3584,  370, 1580,  422, 3528,  472, 1502,  474, 1500,  476, 1498,  452, 1550,  476, 3500,  476, 1524,  452, 3580,  394, 1554,  398, 3528,  450, 3502,  476, 3526,  476, 7532,  450};  // UNKNOWN 8667E349
uint16_t heat_28[101] = {5874, 7504,  504, 3500,  452, 3552,  426, 3580,  422, 3506,  472, 3504,  476, 1498,  478, 1524,  476, 3474,  474, 1526,  452, 1550,  426, 1550,  424, 1604,  398, 1578,  398, 3526,  476, 3500,  478, 1524,  450, 3528,  424, 3554,  422, 1552,  422, 1554,  448, 1524,  476, 1476,  498, 3476,  502, 1498,  478, 1522,  478, 1496,  476, 3554,  398, 3580,  398, 3528,  450, 3502,  476, 1524,  452, 3524,  478, 3500,  476, 1526,  448, 3556,  398, 1578,  424, 3528,  450, 1500,  474, 1524,  452, 1524,  476, 1526,  452, 3552,  424, 1576,  398, 3580,  422, 1526,  450, 3478,  500, 3474,  502, 3498,  478, 7560,  398};  // UNKNOWN B86F3A25
uint16_t heat_29[101] = {5926, 7504,  452, 3608,  396, 3504,  476, 3500,  478, 3474,  476, 3526,  452, 1548,  452, 1524,  450, 3580,  398, 1578,  398, 1552,  448, 1552,  450, 1500,  476, 1500,  476, 3500,  476, 3500,  476, 1498,  452, 3554,  424, 3556,  448, 1528,  472, 1504,  476, 3500,  476, 3526,  426, 1550,  424, 1550,  426, 1578,  422, 1552,  424, 3506,  498, 3474,  502, 1500,  478, 1524,  478, 3500,  476, 3554,  372, 3556,  420, 1554,  448, 3504,  474, 1500,  502, 3474,  504, 1496,  478, 1552,  448, 1524,  450, 1552,  424, 3556,  396, 1552,  424, 3504,  472, 1550,  474, 3502,  476, 3500,  478, 3502,  448, 7508,  476};  // UNKNOWN 82C38248
uint16_t heat_30[101] = {5902, 7480,  476, 3554,  422, 3582,  422, 3504,  476, 3500,  476, 3478,  498, 1526,  476, 1500,  450, 3550,  426, 1604,  370, 1580,  396, 1580,  446, 1528,  446, 1502,  476, 3478,  500, 3500,  476, 1500,  474, 3502,  450, 3554,  424, 1604,  422, 1526,  424, 1528,  472, 3504,  474, 1528,  474, 1524,  448, 1528,  422, 1576,  420, 3532,  422, 3528,  474, 3478,  502, 1500,  476, 3524,  478, 3500,  450, 3610,  344, 1604,  396, 3530,  448, 1526,  450, 3526,  474, 1500,  476, 1524,  478, 1494,  480, 1498,  476, 3582,  368, 1606,  370, 3530,  448, 1554,  448, 3526,  474, 3478,  500, 3472,  452, 7506,  500};  // UNKNOWN 84F28205
// raw data modes
uint16_t heat[101] = {5938, 7470,  512, 3466,  510, 3468,  534, 3442,  538, 3438,  538, 3440,  536, 1440,  536, 1464,  514, 3440,  510, 1490,  512, 1466,  508, 1466,  510, 1490,  512, 1464,  528, 3450,  536, 3440,  512, 1464,  536, 3442,  538, 3442,  536, 1438,  536, 1466,  536, 1440,  512, 3464,  512, 1462,  512, 1462,  512, 1490,  512, 1464,  512, 3466,  536, 3440,  536, 3442,  536, 1440,  534, 3442,  512, 3464,  512, 3466,  536, 1440,  534, 3442,  538, 1464,  538, 3440,  536, 1438,  512, 1464,  512, 1462,  512, 1490,  510, 3466,  538, 1438,  534, 3442,  536, 1438,  536, 3440,  512, 3464,  512, 3466,  512, 7472,  536};  // TRANSCOLD F9C4A8
uint16_t cool[101] = {5942, 7468,  542, 3436,  542, 3434,  542, 3436,  542, 3436,  542, 3436,  542, 1434,  542, 1460,  540, 3410,  540, 1462,  542, 1432,  542, 1434,  544, 1458,  542, 1434,  542, 3436,  540, 3436,  542, 1434,  542, 1434,  540, 3436,  540, 3438,  544, 1458,  542, 1432,  542, 3436,  540, 3436,  540, 3438,  540, 3436,  540, 1434,  540, 1436,  542, 3436,  542, 3436,  542, 1460,  542, 1432,  542, 1432,  542, 3436,  540, 1434,  540, 3436,  542, 1460,  540, 3436,  542, 1434,  542, 1432,  540, 1434,  542, 1460,  542, 3436,  540, 1434,  542, 3436,  542, 1434,  542, 3436,  540, 3436,  542, 3436,  542, 7440,  542};  // TRANSCOLD F967A8
uint16_t avto[101] = {5938, 7472,  510, 3468,  510, 3466,  538, 3440,  536, 3440,  536, 3440,  512, 1462,  514, 1490,  512, 3442,  534, 1466,  534, 1442,  534, 1442,  510, 1490,  512, 1462,  512, 3464,  512, 3464,  514, 1464,  536, 3444,  534, 3442,  536, 3440,  536, 1438,  510, 1490,  510, 1464,  512, 1466,  510, 3466,  510, 1466,  508, 1494,  508, 1466,  534, 3444,  534, 3442,  510, 3468,  512, 3466,  508, 1468,  508, 3470,  532, 1442,  508, 3468,  534, 1468,  510, 3468,  508, 1466,  508, 1466,  508, 1492,  482, 1520,  482, 3472,  504, 1472,  506, 3470,  506, 1470,  506, 3470,  506, 3472,  506, 3496,  480, 7478,  504};  // TRANSCOLD F9E1A8
uint16_t dry[101] = {5936, 7478,  504, 3472,  506, 3470,  508, 3468,  508, 3470,  506, 3496,  482, 1522,  480, 1494,  480, 3474,  506, 1468,  506, 1468,  532, 1470,  508, 1468,  508, 1468,  506, 3496,  482, 3496,  480, 1496,  480, 3474,  504, 1496,  506, 3470,  506, 1470,  506, 1470,  504, 3496,  482, 1520,  480, 1496,  480, 1496,  480, 3474,  504, 1470,  504, 3470,  506, 3472,  506, 1496,  506, 3496,  480, 3472,  480, 3496,  480, 1520,  482, 3472,  504, 1470,  504, 3472,  504, 1494,  480, 1522,  480, 1496,  480, 1496,  480, 3472,  506, 1468,  506, 3470,  506, 1520,  482, 3496,  480, 3498,  478, 3472,  480, 7476,  506};  // TRANSCOLD F9A4A8





AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

Adafruit_BMP280 bme;

const uint16_t kIrLed = 23;  // ESP8266 GPIO pin to use. Recommended: 4 (D2).
IRsend irsend(kIrLed);

String temperatureString = "HelloWorld";
unsigned long previousMillis = 0;
const long interval = 60 * 1000;

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  String messageTemp;
  for (int i = 0; i < len; i++)
  {
    //Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  
  const char *  str_topic[NUM_TOPIC] = {MQTT_TOPIC_AC_RUN, MQTT_TOPIC_AC_MODE, MQTT_TOPIC_AC_TEMP_HEAT, MQTT_TOPIC_AC_TEMP_COOL};
  const char *  str_acmode[NUM_ACMODE] = {AC_MODE_COOL, AC_MODE_HEAT, AC_MODE_AVTO, AC_MODE_DRY};
  const char *  str_temp[NUM_TEMP] = {"19", "20", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30"};

    uint8_t i;
    for (i = 0; i < NUM_TOPIC; i++) {
      if (strcmp(str_topic[i], topic) == 0) {
        break;
      }
    }
    switch (i) {
      case 0:
        if (strcmp(messageTemp.c_str(), AC_RUN_ON) == 0) {
          sendCmd(acOn);
        } else if (strcmp(messageTemp.c_str(), AC_RUN_OFF) == 0) {
          sendCmd(acOff);
        }
        break;
      case 1:
        uint8_t a;
        for (a = 0; a < NUM_ACMODE; a++) {
          if (strcmp(str_acmode[a], messageTemp.c_str()) == 0) {
            break;
          }
        }
        switch (a) {
          case 0:
            sendCmd(cool);
            break;
          case 1:
            sendCmd(heat);
            break;
          case 2:
            sendCmd(avto);
            break;
          case 3:
            sendCmd(dry);
            break;
          default:
              break;
        }
        break;
      case 2:
        uint8_t c;
        for (c = 0; c < NUM_TEMP; c++) {
          if (strcmp(str_temp[c], messageTemp.c_str()) == 0) {
            break;
          }
        }
        switch (c) {
          case 0:
            sendCmd(heat_19);
            break;
          case 1:
            sendCmd(heat_20);
            break;
          case 2:
            sendCmd(heat_21);
            break;
          case 3:
            sendCmd(heat_22);
            break;
          case 4:
            sendCmd(heat_23);
            break;
          case 5:
            sendCmd(heat_24);
            break;
          case 6:
            sendCmd(heat_25);
            break;
          case 7:
            sendCmd(heat_26);
            break;
          case 8:
            sendCmd(heat_27);
            break;
          case 9:
            sendCmd(heat_28);
            break;
          case 10:
            sendCmd(heat_29);
            break;
          case 11:
            sendCmd(heat_30);
            break;
          default:
              break;
        }
        break;
      case 3:
        uint8_t b;
        for (b = 0; b < NUM_TEMP; b++) {
          if (strcmp(str_temp[b], messageTemp.c_str()) == 0) {
            break;
          }
        }
        switch (b) {
          case 0:
            sendCmd(cool_19);
            break;
          case 1:
            sendCmd(cool_20);
            break;
          case 2:
            sendCmd(cool_21);
            break;
          case 3:
            sendCmd(cool_22);
            break;
          case 4:
            sendCmd(cool_23);
            break;
          case 5:
            sendCmd(cool_24);
            break;
          case 6:
            sendCmd(cool_25);
            break;
          case 7:
            sendCmd(cool_26);
            break;
          case 8:
            sendCmd(cool_27);
            break;
          case 9:
            sendCmd(cool_28);
            break;
          case 10:
            sendCmd(cool_29);
            break;
          case 11:
            sendCmd(cool_30);
            break;
          default:
              break;
        }
        break;
      default:
        break;
    }

  Serial.println("Publish received.");
  //  "Опубликованные данные получены."
  Serial.print("  message: "); //  "  сообщение: "
  Serial.println(messageTemp);
  Serial.print("  topic: "); //  "  топик: "
  Serial.println(topic);
  Serial.print("  qos: "); //  "  уровень обслуживания: "
  Serial.println(properties.qos);
  Serial.print("  dup: "); //  "  дублирование сообщения: "
  Serial.println(properties.dup);
  Serial.print("  retain: "); //  "сохраненные сообщения: "
  Serial.println(properties.retain);
  Serial.print("  len: "); //  "  размер: "
  Serial.println(len);
  Serial.print("  index: "); //  "  индекс: "
  Serial.println(index);
  Serial.print("  total: "); //  "  суммарно: "
  Serial.println(total);
}

void sendCmd(uint16_t buf[]) {
  irsend.sendRaw(buf, 101, 38);  // Send a raw data capture at 38kHz.
}

void connectToWifi()
{
  Serial.println("Connecting to Wi-Fi...");
  //  "Подключаемся к WiFi..."
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt()
{
  Serial.println("Connecting to MQTT...");
  //  "Подключаемся к MQTT..."
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch  (event)
  {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected"); //  "Подключились к WiFi"
      Serial.println("IP address: ");   //  "IP-адрес: "
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      //  "WiFi-связь потеряна"
      // делаем так, чтобы ESP32
      // не переподключалась к MQTT
      // во время переподключения к WiFi:
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      break;
    default:
      break;
  }
}

void onMqttConnect(bool sessionPresent)
{
  Serial.println("Connected to MQTT."); //  "Подключились к MQTT."
  Serial.print("Session present: ");    //  "Текущая сессия: "
  Serial.println(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe(MQTT_TOPIC_AC, 1);
  Serial.print("Subscribing at QoS 0, packetId: "); //  "Подписка при QoS 0, ID пакета: "
  Serial.println(packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Disconnected from MQTT.");
  //  "Отключились от MQTT."
  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
  Serial.println("Subscribe acknowledged.");
  //  "Подписка подтверждена."
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId)
{
  Serial.println("Unsubscribe acknowledged.");
  //  "Отписка подтверждена."
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttPublish(uint16_t packetId)
{
  Serial.println("Publish acknowledged.");
  //  "Публикация подтверждена."
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup()
{
  Serial.begin(115200);
  irsend.begin();
  if (!bme.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  connectToWifi();
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
}

void loop()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    // публикуем MQTT-сообщение в топике «esp32/temperature»
    // с температурой в градусах Цельсия и Фаренгейта:

    char tempArray[16];
    sprintf(tempArray, "%.2f", bme.readTemperature());

    char pressureArray[16];
    sprintf(pressureArray, "%.2f", bme.readPressure());

    char altitudeArray[16];
    sprintf(altitudeArray, "%.2f", bme.readAltitude(1013.25));

    Serial.print("Temperature: ");
    Serial.println(tempArray);
    Serial.print("Pressure: ");
    Serial.println(pressureArray);
    Serial.print("Altitude: ");
    Serial.println(altitudeArray);

    uint16_t packetIdPub2 = mqttClient.publish(MQTT_TOPIC_TEMPERATURE, 2, true, tempArray);
    Serial.print("Publishing on topic esp32/myowndemo/temperature at QoS 2, packetId: ");
    Serial.println(packetIdPub2);
    delay(100);
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_TOPIC_PRESSURE, 2, true, pressureArray);
    Serial.print("Publishing on topic esp32/myowndemo/pressure at QoS 2, packetId: ");
    Serial.println(packetIdPub3);
    delay(100);
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_TOPIC_ALTITUDE, 2, true, altitudeArray);
    Serial.print("Publishing on topic esp32/myowndemo/altitude at QoS 2, packetId: ");
    Serial.println(packetIdPub4);
    delay(100);
  }
}
