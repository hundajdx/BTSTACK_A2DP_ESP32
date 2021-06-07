
#define log_info printf

#define I2S_bck_io_num   27  //BCK
#define I2S_ws_io_num    0   //LCK  = GPIO0 is also used for Enable "Flashing mode" DO NOT USE WHEN CP2101 SERIAL IS CONNECTED
#define I2S_data_out_num 25  //DIN

///#define HAVE_BTSTACK_STDIN ///enable control from SERIAL CONSOLE...

///#define L2CAP_USES_CHANNELS ///???
///#define CONFIG_BT_ENABLED
#define ENABLE_LOG_DEBUG




//================================================================================
#include "btstack_esp32/btstack_port_esp32.h"
#include "btstack/btstack_run_loop.h"

//================================================================================
//--------------------------------------------------------------------------------
#include "BLEUUID.h" //for manual connection... (esp_bt_controller_init)!
#include "bt.h" ///ESP32
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h" //for (esp_bt_sp_param_t)!
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
#include "btstack_esp32/btstack_port_esp32.h"
#include "btstack_esp32/btstack_port_esp32.c"
#include "btstack_platform/btstack_run_loop_freertos.c" 
#include "btstack_esp32/btstack_tlv_esp32.c" 
#include "btstack_esp32/btstack_audio_esp32.c"
#include "btstack_esp32/btstack_stdin_esp32.c"
//--------------------------------------------------------------------------------
#include "btstack_config.h"
#include "btstack/btstack.h"
#include "btstack/l2cap.h"
#include "btstack/hci.c"
#include "btstack/hci_dump.c"
#include "btstack/hci_cmd.c"
#include "btstack/l2cap.c"
#include "btstack/btstack_hid_parser.c" 
#include "btstack/l2cap_signaling.c"
#include "btstack/ad_parser.c"
#include "btstack/btstack_ring_buffer.c"
#include "btstack/btstack_run_loop.c"
#include "btstack/btstack_util.c"
#include "btstack/btstack_memory.c"
#include "btstack/btstack_tlv.c"
#include "btstack/btstack_linked_list.c"
#include "btstack/btstack_audio.c"
#include "btstack/btstack_resample.c"
#include "btstack/btstack_crypto.c"
#include "btstack/btstack_memory_pool.c"
//--------------------------------------------------------------------------------
#include "btstack/classic/sdp_client.c" 
#include "btstack/classic/sdp_util.c"
#include "btstack/classic/btstack_link_key_db_tlv.c"
#include "btstack/classic/avrcp.c"
#include "btstack/classic/avrcp_controller.c"
#include "btstack/classic/device_id_server.c"
#include "btstack/classic/sdp_server.c"
#include "btstack/classic/a2dp_sink.c"
#include "btstack/classic/avdtp.c"
#include "btstack/classic/avdtp_sink.c"
#include "btstack/classic/avdtp_source.c"
#include "btstack/classic/avdtp_util.c"
#include "btstack/classic/avrcp_target.c"
#include "btstack/classic/btstack_sbc_decoder_bluedroid.c"
#include "btstack/classic/btstack_sbc_encoder_bluedroid.c"
#include "btstack/classic/avdtp_acceptor.c"
#include "btstack/classic/avdtp_initiator.c"
#include "btstack/classic/btstack_sbc_plc.c"
#include "btstack/classic/a2dp_source.c"
#include "btstack/classic/spp_server.c"
#include "btstack/classic/rfcomm.c" 
#include "btstack/ble/le_device_db_tlv.c"
#include "btstack/ble/sm.c"
#include "btstack/ble/att_server.c"
#include "btstack/ble/att_dispatch.c"
#include "btstack/ble/att_db.c"
#include "btstack/ble/gatt_client.c"
//--------------------------------------------------------------------------------

//--------------------------------------------------------------------------------
#include "btstack_oi/decoder-sbc.c"
#include "btstack_oi/decoder-private.c"
#include "btstack_oi/framing.c"
#include "btstack_oi/bitalloc-sbc.c"
#include "btstack_oi/bitstream-decode.c"
#include "btstack_oi/dequant.c"
#include "btstack_oi/bitalloc.c"
#include "btstack_oi/alloc.c"
#include "btstack_oi/synthesis-sbc.c"
#include "btstack_oi/synthesis-dct8.c"
#include "btstack_oi/synthesis-8-generated.c"
#include "btstack_oi/nao-deceased_by_disease.c"
#include "btstack_oi/hxcmod.c"
//--------------------------------------------------------------------------------

//================================================================================





extern const char * device_addr_string = "EC:D0:9F:35:F3:7B";

#include "a2dp_sink_demo.c" ///


static btstack_packet_callback_registration_t hci_event_callback_registration___;


bd_addr_t event_addr;  //stored actual event client address
uint8_t status;
esp_bt_pin_code_t pin_code;
extern hci_stack_t * hci_stack;
uint32_t PINCODEREPLY=0;
uint8_t PINCONFIRMED=0; //0 = not obtained, 1=obtained, 2=comfirmed
char pin_code_char[8];
    
static void hci_packet_handler____(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
//--------------------------------------------------------------------------------
///confirm 6digit pin...
   if (PINCONFIRMED==1 && hci_can_send_command_packet_now() ) {
      PINCONFIRMED=2;
      printf(">>> PINCONFIRMED........\n");
      hci_send_cmd(&hci_user_passkey_request_reply, event_addr, pin_code_char);
   }
//--------------------------------------------------------------------------------
  switch (packet_type) {       
    case HCI_EVENT_PACKET:
      switch (hci_event_packet_get_type(packet)) {
        case BTSTACK_EVENT_STATE:
          if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
            printf("BTSTACK: --> BTstack up and running on %s.\n", bd_addr_to_str(event_addr));
          }
          break;
        // HCI EVENTS
        case HCI_EVENT_COMMAND_COMPLETE: {
          uint16_t opcode = hci_event_command_complete_get_command_opcode(packet);
          const uint8_t* param = hci_event_command_complete_get_return_parameters(packet);
          status = param[0];
          printf("HCI: --> HCI_EVENT_COMMAND_COMPLETE: opcode = 0x%04x - status=%d\n", opcode, status);
          break;
        }
        case HCI_EVENT_AUTHENTICATION_COMPLETE_EVENT: {
          status = hci_event_authentication_complete_get_status(packet);
          uint16_t handle = hci_event_authentication_complete_get_connection_handle(packet);
          printf("HCI: --> HCI_EVENT_AUTHENTICATION_COMPLETE_EVENT: status=%d, handle=0x%04x\n", status, handle);             
          break;
        }
//================================================================================       
        case HCI_EVENT_PIN_CODE_REQUEST: {
          // gap_pin_code_response_binary does not copy the data, and data must
          // be valid until the next hci_send_cmd is called.
          static bd_addr_t pin_code;
          bd_addr_t local_addr;
          printf("HCI: --> HCI_EVENT_PIN_CODE_REQUEST\n");
          // FIXME: Assumes incoming connection from Nintendo Wii using Sync.
          //
          // From: https://wiibrew.org/wiki/Wiimote#Bluetooth_Pairing:
          //  If connecting by holding down the 1+2 buttons, the PIN is the
          //  bluetooth address of the wiimote backwards, if connecting by
          //  pressing the "sync" button on the back of the wiimote, then the
          //  PIN is the bluetooth address of the host backwards.
          hci_event_pin_code_request_get_bd_addr(packet, event_addr);
          gap_local_bd_addr(local_addr);
          reverse_bd_addr(local_addr, pin_code);
          printf("Using PIN code: \n");
          printf_hexdump(pin_code, sizeof(pin_code));
          gap_pin_code_response_binary(event_addr, pin_code, sizeof(pin_code));
          break;
        }
//================================================================================
        case HCI_EVENT_CONNECTION_REQUEST:
          printf("--> HCI_EVENT_CONNECTION_REQUEST: link_type = %d <--\n", hci_event_connection_request_get_link_type(packet));
            hci_event_connection_request_get_bd_addr(packet, event_addr);
//--------------------------------------------------------------------------------
            a2dp_sink_establish_stream(event_addr, a2dp_local_seid, &a2dp_cid);    
            PINCONFIRMED=0;
//--------------------------------------------------------------------------------
          break;
//================================================================================
        case L2CAP_EVENT_INCOMING_CONNECTION: 
          printf("--> L2CAP_EVENT_INCOMING_CONNECTION\n");          
          break;
        case L2CAP_EVENT_CHANNEL_OPENED:
          printf("--> L2CAP_EVENT_CHANNEL_OPENED\n");
          break;
        case L2CAP_EVENT_CHANNEL_CLOSED:
          printf("--> L2CAP_EVENT_CHANNEL_CLOSED\n");
          break;
//================================================================================
        case HCI_EVENT_RETURN_LINK_KEYS:
          printf("--> HCI_EVENT_RETURN_LINK_KEYS\n");
          break;
        case HCI_EVENT_LINK_KEY_REQUEST:
          printf("--> HCI_EVENT_LINK_KEY_REQUEST\n");
          break;
        case HCI_EVENT_LINK_KEY_NOTIFICATION:
          printf("--> HCI_EVENT_LINK_KEY_NOTIFICATION\n");
          break;
        case HCI_EVENT_SIMPLE_PAIRING_COMPLETE:
          printf("--> HCI_EVENT_SIMPLE_PAIRING_COMPLETE\n");
//--------------------------------------------------------------------------------
          a2dp_sink_establish_stream(event_addr, a2dp_local_seid, &a2dp_cid);    
//--------------------------------------------------------------------------------          
          break;
        case HCI_EVENT_USER_PASSKEY_NOTIFICATION:
          printf("--> HCI_EVENT_USER_PASSKEY_NOTIFICATION\n");
          break;
        case HCI_EVENT_KEYPRESS_NOTIFICATION:
          printf("--> HCI_EVENT_KEYPRESS_NOTIFICATION\n");
          break;
//--------------------------------------------------------------------------------          
        case HCI_EVENT_USER_CONFIRMATION_REQUEST:
          printf(">>> HCI_EVENT_USER_CONFIRMATION_REQUEST. \n");
          PINCODEREPLY=little_endian_read_32(packet, 8);
          printf("ENTERING PIN: %d TO CLIENT: %s \n",PINCODEREPLY,bd_addr_to_str(event_addr));
          sprintf(pin_code_char,"%06ld", PINCODEREPLY);            
          ///printf("[%c%c%c%c%c%c]\n",pin_code_char[0],pin_code_char[1],pin_code_char[2],pin_code_char[3],pin_code_char[4],pin_code_char[5]);
          ///printf("[%d:%d:%d:%d:%d:%d]\n",event_addr[0],event_addr[1],event_addr[2],event_addr[3],event_addr[4],event_addr[5]);

          pin_code[0] = pin_code_char[0];
          pin_code[1] = pin_code_char[1];
          pin_code[2] = pin_code_char[2];
          pin_code[3] = pin_code_char[3];
          pin_code[4] = pin_code_char[4];
          pin_code[5] = pin_code_char[5];
          
          PINCONFIRMED=1;
          ///hci_ssp_validate_possible_security_level(event_addr);

          ///gap_pin_code_response(event_addr,pin_code_char);
          ///gap_ssp_passkey_response(event_addr,PINCODEREPLY);
          ///hci_send_cmd(&hci_user_passkey_request_reply, event_addr, pin_code_char);
          break;
//--------------------------------------------------------------------------------                   
        case HCI_EVENT_USER_PASSKEY_REQUEST:
          printf(">>> HCI_EVENT_USER_PASSKEY_REQUEST. \n");
          PINCODEREPLY=little_endian_read_32(packet, 8);
          printf("ENTERING PIN: %d TO CLIENT: %s \n",PINCODEREPLY,bd_addr_to_str(event_addr));
          sprintf(pin_code_char,"%06ld", PINCODEREPLY);            
          ///printf("[%c%c%c%c%c%c]\n",pin_code_char[0],pin_code_char[1],pin_code_char[2],pin_code_char[3],pin_code_char[4],pin_code_char[5]);
          ///printf("[%d:%d:%d:%d:%d:%d]\n",event_addr[0],event_addr[1],event_addr[2],event_addr[3],event_addr[4],event_addr[5]);

          pin_code[0] = pin_code_char[0];
          pin_code[1] = pin_code_char[1];
          pin_code[2] = pin_code_char[2];
          pin_code[3] = pin_code_char[3];
          pin_code[4] = pin_code_char[4];
          pin_code[5] = pin_code_char[5];
          
          PINCONFIRMED=1;
          ///gap_pin_code_response(event_addr,pin_code_char);
          ///gap_ssp_passkey_response(event_addr,PINCODEREPLY);
            
          ///hci_add_connection_flags_for_flipped_bd_addr(&packet[2], SEND_USER_PASSKEY_REPLY);
          break;
         
//===============================================================================        
        default:
          break;
      }
      break;
    case L2CAP_DATA_PACKET:
      printf("_________L2CAP_DATA_PACKET: \n");
      break;
      
    default:
      break;
  }

  hci_run(); ///FATAL INSTRUCTION!
 
//================================================================================  
}



void setup() {
  Serial.begin(115200);
  
       
  btstack_init(); //init in (btstack_port_esp32.c)   
  
  l2cap_init();
  sm_init();


  ///esp_bredr_tx_power_set(ESP_PWR_LVL_N12 ,ESP_PWR_LVL_P7   ); //min max
  // set discoverable and connectable mode, wait to be connected 
  // enabled EIR
  ///hci_set_inquiry_mode(INQUIRY_MODE_RSSI_AND_EIR);
  
//--------------------------------------------------------------------------------
    // Initialize AVDTP Sink
    a2dp_sink_init();
    a2dp_sink_register_packet_handler(&a2dp_sink_packet_handler);
    a2dp_sink_register_media_handler(&handle_l2cap_media_data_packet);

    // Create stream endpoint
    avdtp_stream_endpoint_t * local_stream_endpoint = a2dp_sink_create_stream_endpoint(AVDTP_AUDIO, 
        AVDTP_CODEC_SBC, media_sbc_codec_capabilities, sizeof(media_sbc_codec_capabilities), 
        media_sbc_codec_configuration, sizeof(media_sbc_codec_configuration));
    if (!local_stream_endpoint){
        printf("A2DP Sink: not enough memory to create local stream endpoint\n");
        while(1) {} //halt
    }

    // Store stream enpoint's SEP ID, as it is used by A2DP API to indentify the stream endpoint
    a2dp_local_seid = avdtp_local_seid(local_stream_endpoint);
///NCX: pokus:

    // Initialize AVRCP service
    avrcp_init();
    avrcp_register_packet_handler(&avrcp_packet_handler____);
    
    // Initialize AVRCP Controller
    avrcp_controller_init();
    avrcp_controller_register_packet_handler(&avrcp_controller_packet_handler____);
    
     // Initialize AVRCP Target
    avrcp_target_init();
    avrcp_target_register_packet_handler(&avrcp_target_packet_handler____);

//--------------------------------------------------------------------------------

    // Create A2DP Sink service record and register it with SDP
    memset(sdp_avdtp_sink_service_buffer, 0, sizeof(sdp_avdtp_sink_service_buffer));
    a2dp_sink_create_sdp_record(sdp_avdtp_sink_service_buffer, 0x10001, AVDTP_SINK_FEATURE_MASK_HEADPHONE, NULL, NULL);

    sdp_register_service(sdp_avdtp_sink_service_buffer);

    // Create AVRCP Controller service record and register it with SDP. We send Category 1 commands to the media player, e.g. play/pause
    memset(sdp_avrcp_controller_service_buffer, 0, sizeof(sdp_avrcp_controller_service_buffer));
    uint16_t controller_supported_features = AVRCP_FEATURE_MASK_CATEGORY_PLAYER_OR_RECORDER;
#ifdef AVRCP_BROWSING_ENABLED
    controller_supported_features |= AVRCP_FEATURE_MASK_BROWSING;
#endif
    avrcp_controller_create_sdp_record(sdp_avrcp_controller_service_buffer, 0x10002, controller_supported_features, NULL, NULL);
    sdp_register_service(sdp_avrcp_controller_service_buffer);
    
    // Create AVRCP Target service record and register it with SDP. We receive Category 2 commands from the media player, e.g. volume up/down
    memset(sdp_avrcp_target_service_buffer, 0, sizeof(sdp_avrcp_target_service_buffer));
    uint16_t target_supported_features = AVRCP_FEATURE_MASK_CATEGORY_MONITOR_OR_AMPLIFIER;
    avrcp_target_create_sdp_record(sdp_avrcp_target_service_buffer, 0x10003, target_supported_features, NULL, NULL);
    sdp_register_service(sdp_avrcp_target_service_buffer);

    // Create Device ID (PnP) service record and register it with SDP
    memset(device_id_sdp_service_buffer, 0, sizeof(device_id_sdp_service_buffer));
    device_id_create_sdp_record(device_id_sdp_service_buffer, 0x10004, DEVICE_ID_VENDOR_ID_SOURCE_BLUETOOTH, BLUETOOTH_COMPANY_ID_BLUEKITCHEN_GMBH, 1, 1);
    sdp_register_service(device_id_sdp_service_buffer);

//--------------------------------------------------------------------------------


  // init SDP, create record for SPP and register with SDP
  sdp_init_BTSTACK(); ///

/*
  gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO); ///veryyyy important
  gap_set_class_of_device(0x200408);
  gap_discoverable_control(1); //set visibility for Bluetooth clients
  gap_connectable_control(1); ///
  gap_ssp_set_enable(1); /// enable Simple Secure Pairing  
  gap_set_local_name("OMG 00:00:00:00:00:00");
*/  

  gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO); //display PAIRING CODE in CLIENT
  ///gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_ONLY); //display in CLIENT
  ///gap_set_security_level(LEVEL_3); //set security level 
  gap_discoverable_control(1);
  ///gap_advertisements_enable(1); ///this not
  gap_ssp_set_enable(1);
  gap_ssp_set_auto_accept(1); 
  ///gap_set_default_link_policy_settings( LM_LINK_POLICY_ENABLE_ROLE_SWITCH | LM_LINK_POLICY_ENABLE_SNIFF_MODE );
  ///gap_set_allow_role_switch(true);
  gap_set_local_name("ESP_SPEAKER 00:00:00:00:00:00"); 
  gap_set_class_of_device(0x200408); //headset ID
  gap_secure_connections_enable(true); //enable secured connections

  ///gap_set_required_encryption_key_size(7);

//--------------------------------------------------------------------------------

    // Register for HCI events (MAINLY FOR PIN CODE)
    hci_event_callback_registration____.callback = &hci_packet_handler____;
    hci_add_event_handler(&hci_event_callback_registration____);

//--------------------------------------------------------------------------------

#ifdef HAVE_BTSTACK_STDIN
    // parse human readable Bluetooth address
    sscanf_bd_addr(device_addr_string, device_addr);
    btstack_stdin_setup(stdin_process);
#endif

    // turn on!
    printf("Starting BTstack ...\n");
    hci_power_control(HCI_POWER_ON);

 printf("!!!!!!!!!!!! -> gap_ssp_supported: %d\n",gap_ssp_supported());  
 printf("!!!!!!!!!!!! -> hci_classic_supported: %d\n",hci_classic_supported());  
 

/*
    // Secure Simple Pairing default: enable, no I/O capabilities, general bonding, mitm not required, auto accept 
    hci_stack->ssp_enable = 1;
    hci_stack->ssp_io_capability = SSP_IO_CAPABILITY_DISPLAY_YES_NO;
///    hci_stack->ssp_authentication_requirement = SSP_IO_AUTHREQ_MITM_PROTECTION_NOT_REQUIRED_GENERAL_BONDING;
    hci_stack->ssp_auto_accept = 1;
*/

  esp_power_level_t min,max;
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV,ESP_PWR_LVL_P7  );
  esp_bredr_tx_power_set(ESP_PWR_LVL_P7 ,ESP_PWR_LVL_P7   );
  delay(1000);
  esp_bredr_tx_power_get(&min,&max);
  printf("BLUETOOTH: min: %d max: %d\n",min,max);

///esp_bt_sleep_enable();
esp_bt_sleep_disable();

  btstack_run_loop_execute(); 
}

void loop() {
 
}
