#define RC_MODE 0
#define DATA_MODE 1
#define LINE_TENSION_REQUEST_MODE 2
#define LINE_LENGTH_MODE 3

#define DATALENGTH 2


static uint8_t broadcast_mac[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
const uint8_t WIFI_CHANNEL = 0;

float receivedData[DATALENGTH] = {0.0, 0.0};

float line_length_in_meters = 1;
float flight_mode = 0;

typedef struct __attribute__((packed)) esp_now_msg_t
{
	uint32_t mode;
	float data[DATALENGTH];
	// Can put lots of things here
} esp_now_msg_t;

float receive_counter = 0;

// gets called when incoming data is received
static void msg_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
	if (len == sizeof(esp_now_msg_t))
	{
		esp_now_msg_t msg;
		memcpy(&msg, data, len);
		
		if(msg.mode == LINE_LENGTH_MODE){
			line_length_in_meters = msg.data[0];
			flight_mode = msg.data[1];
		}
	}
	
}


// init wifi on the esp
// register callbacks
void network_setup_flying(void)
{
	//TODO: check if neccessary when WIFI_STORAGE_RAM is used
	// Initialize FS NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
	
	
	//ESP_ERROR_CHECK(esp_netif_init());
	
	// Wifi
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&cfg);
	
	
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
	// Puts ESP in STATION MODE
	
	esp_wifi_set_mode(WIFI_MODE_STA); // MUST BE CALLED AFTER esp_wifi_init(&cfg) to have a non-volatile effect on the flash nvs!
	
	esp_wifi_config_80211_tx_rate(ESP_IF_WIFI_STA, WIFI_PHY_RATE_LORA_250K);// TODO: does this influence the range?
	
	esp_wifi_start();
	
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_LR) );
    
    
	
	// ESP NOW
	esp_now_init();
	
	esp_now_peer_info_t peer_info;
	peer_info.channel = WIFI_CHANNEL;
	memcpy(peer_info.peer_addr, broadcast_mac, 6);
	peer_info.ifidx = ESP_IF_WIFI_STA;
	peer_info.encrypt = false;
	esp_now_add_peer(&peer_info);
	
	// Register Send Callback
	esp_now_register_recv_cb(msg_recv_cb);
}

// used by the kite to send data to the data receiver
void sendDataArray(float data[DATALENGTH], uint32_t mode){

	esp_now_msg_t msg;
	
	msg.mode = mode;
	for(int i = 0; i < DATALENGTH; i++){
		msg.data[i] = data[i];
	}
	
	// Pack
	uint16_t packet_size = sizeof(esp_now_msg_t);
	uint8_t msg_data[packet_size]; // Byte array
	memcpy(&msg_data[0], &msg, sizeof(esp_now_msg_t));
	
	// Send
	esp_now_send(broadcast_mac, msg_data, packet_size);
}

void sendData(uint32_t mode, float data0, float data1){
	
	float to_be_sent[DATALENGTH];
	for(int i = 0; i < DATALENGTH; i++){
		to_be_sent[i] = 0;
	}
	to_be_sent[0] = data0;
	to_be_sent[1] = data1;
	sendDataArray(to_be_sent, mode);
}


