#include <Arduino.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
#include <LoRaWan-RAK4630.h>   //http://librarymanager/All#SX126x
#include <SPI.h>
#include "bat.h"

// RAK4630 supply two LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif

#ifndef LED_BUILTIN2
#define LED_BUILTIN2 36
#endif

#define ID_SOIL_SENSOR          0x02
#define ADDR_SM_HUMIDITY        0x0000
#define ADDR_SM_TEMPERATURE     0x0001
#define ADDR_SM_CONDUCTIVITY    0x0002
#define ADDR_SM_PH              0x0003
#define ADDR_SM_NITROGEN        0x0004
#define ADDR_SM_PHOSPHORUS      0x0005
#define ADDR_SM_POTASSIUM       0x0006
#define ADDR_SM_SALINITY        0x0007
#define ADDR_SM_TDS             0x0008

/** Battery level in percentage */
uint8_t battLevel = 0;



int estado_rele = 0;
long volt_PV, current_PV, volt_Bat, current_Bat, volt_Load, current_Load, temp_Controller,porcent_Bat;

long soil_temp,soil_humidity,soil_conductivity, soil_ph, soil_nitrogen, soil_phosporus, soil_potassium, soil_salinity, soil_tds;
bool firstMessage = true; // flag para saber cual mensaje enviar

bool doOTAA = true;
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60                      /**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DATERATE DR_0                   /*LoRaMac datarates definition, from DR_0 to DR_5*/
#define LORAWAN_TX_POWER TX_POWER_5                 /*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
#define JOINREQ_NBTRIALS 3                      /**< Number of trials for the join request. */
DeviceClass_t gCurrentClass = CLASS_A;                /* class definition*/
LoRaMacRegion_t gCurrentRegion = LORAMAC_REGION_US915;    /* Region:EU868*/
lmh_confirm gCurrentConfirm = LMH_UNCONFIRMED_MSG;          /* confirm/unconfirm packet definition*/
uint8_t gAppPort = LORAWAN_APP_PORT;                /* data port*/

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
 */
static lmh_param_t lora_param_init = {LORAWAN_ADR_ON, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                        lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler, lorawan_join_failed_handler
                                       };

//OTAA keys
uint8_t nodeDeviceEUI[8] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x04, 0xE7, 0x92};
uint8_t nodeAppEUI[8] = { 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10 };
uint8_t nodeAppKey[16] = { 0xC0, 0xD6, 0x79, 0x52, 0xDC, 0x35, 0xC2, 0xB5, 0x4D, 0x6E, 0x95, 0xF2, 0xD1, 0x31, 0xCF, 0xEA };
// ABP keys
uint32_t nodeDevAddr = 0x260116F8;
uint8_t nodeNwsKey[16] = {0x7E, 0xAC, 0xE2, 0x55, 0xB8, 0xA5, 0xE2, 0x69, 0x91, 0x51, 0x96, 0x06, 0x47, 0x56, 0x9D, 0x23};
uint8_t nodeAppsKey[16] = {0xFB, 0xAC, 0xB6, 0x47, 0xF3, 0x58, 0x45, 0xC7, 0x50, 0x7D, 0xBF, 0x16, 0x8B, 0xA8, 0xC1, 0x7C};


// Private defination
#define LORAWAN_APP_DATA_BUFF_SIZE 64                     /**< buffer size of the data to be transmitted. */
#define LORAWAN_APP_INTERVAL 20000                        /**< Defines for user timer, the application data transmission interval. 20s, value in [ms]. */
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];        //< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; //< Lora user application data structure.

TimerEvent_t appTimer;
static uint32_t timers_init(void);
static uint32_t count = 0;
static uint32_t count_fail = 0;

void enableRS485(bool enable);
long getSoilSensorData(int address);

void setup() {
  //pinMode(rele,OUTPUT);
  //digitalWrite(rele,HIGH);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(LED_BUILTIN2, OUTPUT);
  digitalWrite(LED_BUILTIN2, LOW);  
  initReadVBAT();
  enableRS485(0); //Deshabilito el modulo 485

  // Initialize LoRa chip.
  lora_rak4630_init();
  // Initialize Serial for debug output
  Serial.begin(115200);
  /*while (!Serial)
  {
    delay(10);
  }*/
  delay(2000);

  Serial.println("=====================================");
  Serial.println("UDC Soil Moisture Sensors");
  if (doOTAA)
  {
    Serial.println("Type: OTAA");
  }
  else
  {
    Serial.println("Type: ABP");
  }
  Serial.println("=====================================");
  switch (gCurrentRegion)
  {
    case LORAMAC_REGION_AS923:
      Serial.println("Region: AS923");
      break;
    case LORAMAC_REGION_AU915:
      Serial.println("Region: AU915");
      break;
    case LORAMAC_REGION_CN470:
      Serial.println("Region: CN470");
      break;
  case LORAMAC_REGION_CN779:
    Serial.println("Region: CN779");
    break;
    case LORAMAC_REGION_EU433:
      Serial.println("Region: EU433");
      break;
    case LORAMAC_REGION_IN865:
      Serial.println("Region: IN865");
      break;
    case LORAMAC_REGION_EU868:
      Serial.println("Region: EU868");
      break;
    case LORAMAC_REGION_KR920:
      Serial.println("Region: KR920");
      break;
    case LORAMAC_REGION_US915:
      Serial.println("Region: US915");
    break;
  case LORAMAC_REGION_RU864:
    Serial.println("Region: RU864");
    break;
  case LORAMAC_REGION_AS923_2:
    Serial.println("Region: AS923-2");
    break;
  case LORAMAC_REGION_AS923_3:
    Serial.println("Region: AS923-3");
    break;
  case LORAMAC_REGION_AS923_4:
    Serial.println("Region: AS923-4");
      break;
  }
  Serial.println("=====================================");

  // Start Modbus RTU Client
  if(!ModbusRTUClient.begin(9600)){
    Serial.println("Failed to start Modbus Client!");
    while(1);
  }
  Serial.println("Modbus inicializado");


  //creat a user timer to send data to server period
  uint32_t err_code;
  err_code = timers_init();
  if (err_code != 0)
  {
    Serial.printf("timers_init failed - %d\n", err_code);
  }

  // Setup the EUIs and Keys
  if (doOTAA)
  {
    lmh_setDevEui(nodeDeviceEUI);
    lmh_setAppEui(nodeAppEUI);
    lmh_setAppKey(nodeAppKey);
  }
  else
  {
    lmh_setNwkSKey(nodeNwsKey);
    lmh_setAppSKey(nodeAppsKey);
    lmh_setDevAddr(nodeDevAddr);
  }

  // Initialize LoRaWan
  err_code = lmh_init(&lora_callbacks, lora_param_init, doOTAA, gCurrentClass, gCurrentRegion);
  if (err_code != 0)
  {
    Serial.printf("lmh_init failed - %d\n", err_code);
    return;
  }

  // Start Join procedure
  lmh_join();

}

void loop() {

}


/**@brief LoRa function for handling HasJoined event.
 */
void lorawan_has_joined_handler(void)
{
  Serial.println("Modo OTAA, Unido a la red");

  lmh_error_status ret = lmh_class_request(gCurrentClass);
  if (ret == LMH_SUCCESS)
  {
    delay(1000);
    TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
    TimerStart(&appTimer);
  }
}
/**@brief LoRa function for handling OTAA join failed
*/
static void lorawan_join_failed_handler(void)
{
  Serial.println("OTAA join failed!");
  Serial.println("Check your EUI's and Keys's!");
  Serial.println("Check if a Gateway is in range!");
}

/**@brief Function for handling LoRaWan received data from Gateway
 *
 * @param[in] app_data  Pointer to rx data
 */
void lorawan_rx_handler(lmh_app_data_t *app_data)
{
  int rec;
  Serial.printf("LoRa paquete recivido en port %d, size:%d, rssi:%d, snr:%d\n",
          app_data->port, app_data->buffsize, app_data->rssi, app_data->snr);
  rec = app_data->buffer[0];
  /*if (rec == 0){
    estado_rele = 0;
    digitalWrite(rele,HIGH);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.printf("Rele apagado...\n");
  } 
  if (rec == 1){
    estado_rele = 1;
    digitalWrite(rele,LOW);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.printf("Rele prendido...\n");
  }      */ 
}

void lorawan_confirm_class_handler(DeviceClass_t Class)
{
  Serial.printf("Cambiando a clase %c ok\n", "ABC"[Class]);
  // Informs the server that switch has occurred ASAP
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = gAppPort;
  lmh_send(&m_lora_app_data, gCurrentConfirm);
}

void send_lora_frame(void)
{
  if (lmh_join_status_get() != LMH_SET)
  {
    //Not joined, try again later
    return;
  }

  uint32_t i = 0;
  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
  m_lora_app_data.port = gAppPort;
  //m_lora_app_data.buffer[i++] = estado_rele;

  battLevel = readBatt();

  soil_temp = getSoilSensorData(ADDR_SM_TEMPERATURE);
  soil_humidity = getSoilSensorData(ADDR_SM_HUMIDITY);
  soil_ph = getSoilSensorData(ADDR_SM_PH);
  soil_nitrogen = getSoilSensorData(ADDR_SM_NITROGEN);
  soil_conductivity = getSoilSensorData(ADDR_SM_CONDUCTIVITY);
  soil_phosporus = getSoilSensorData(ADDR_SM_PHOSPHORUS);
  soil_potassium = getSoilSensorData(ADDR_SM_POTASSIUM);
  soil_salinity = getSoilSensorData(ADDR_SM_SALINITY);
  delay(200);
  
  Serial.printf("Soil Temperature: \t %ld\r\n",soil_temp);
  Serial.printf("Soil Humidity: \t %ld\r\n",soil_humidity);
  Serial.printf("Soil PH: \t %ld\r\n",soil_ph);
  Serial.printf("Soil Nitrogen: \t %ld\r\n",soil_nitrogen);
  Serial.printf("Soil Conductivity: \t %ld\r\n",soil_conductivity);
  Serial.printf("Soil Potassium: \t %ld\r\n",soil_potassium);
  Serial.printf("Soil Salinity: \t %ld\r\n",soil_salinity);

  
  m_lora_app_data.buffer[i++] = (uint8_t)(soil_temp >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)soil_temp;
  m_lora_app_data.buffer[i++] = (uint8_t)(soil_humidity >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)soil_humidity;
  m_lora_app_data.buffer[i++] = (uint8_t)(soil_ph >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)soil_ph;
  m_lora_app_data.buffer[i++] = (uint8_t)(soil_conductivity >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)soil_conductivity;
  m_lora_app_data.buffer[i++] = (uint8_t)(soil_nitrogen >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)soil_nitrogen;


  m_lora_app_data.buffsize = i;
 


  lmh_error_status error = lmh_send(&m_lora_app_data, gCurrentConfirm);
  if (error == LMH_SUCCESS)
  {
    count++;
    Serial.printf("Conteo OK %d\n", count);
  }
  else
  {
    count_fail++;
    Serial.printf("Conteo error %d\n", count_fail);
  }
}

/**@brief Function for handling user timerout event.
 */
void tx_lora_periodic_handler(void)
{
  TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
  TimerStart(&appTimer);
  Serial.println("Enviando frame...");
  send_lora_frame();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
uint32_t timers_init(void)
{
  TimerInit(&appTimer, tx_lora_periodic_handler);
  return 0;
}

/*
* Funcion para habilitar el modulo 485 wisblock
* habilitar antes de hacer lectura y 
* deshabilitar al terminar la lectura 
* "ahorro de energia"
*/
void enableRS485(bool enable){
  if(enable){
    //IO2 HIGH 3V3_S ON
    pinMode(34, OUTPUT);
    digitalWrite(34, HIGH);
    delay(300);
  }
  else{
    //IO2 LOW 3V3_S OFF
    pinMode(34, OUTPUT);
    digitalWrite(34,LOW);
    delay(300);
  }
}

long getSoilSensorData(int address){
  long data;  
  Serial.print("Reading address ");
  Serial.print(address);
  Serial.print("-> ");
  enableRS485(1);
  data = ModbusRTUClient.holdingRegisterRead(ID_SOIL_SENSOR,address);
  enableRS485(0);
  if (data == -1)
  {
    Serial.println("ERROR!");
    return data;
  }
  Serial.println("OK");  
  return data;
}

/*
void readInputRegisterValues(int registerAddress){

  Serial.println("Reading input register values...");
  // lee 1 input register, id 1, de la direccion que se especifica en registerAddress
  int len = ModbusRTUClient.requestFrom(SOLAR_ID,INPUT_REGISTERS,registerAddress,1);
  if(len == 0){
    Serial.print("Fail: ");
    Serial.println(ModbusRTUClient.lastError());
    return;

  }
  else{
    Serial.println("Success!");
    ModbusRTUClient.
  }
}*/