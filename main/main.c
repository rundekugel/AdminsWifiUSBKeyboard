
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_server.h"

extern const unsigned char index_html_start[] asm("_binary_index_html_start");
extern const unsigned char index_html_end[]   asm("_binary_index_html_end");
extern const unsigned char wifi_html_start[]  asm("_binary_wifi_html_start");
extern const unsigned char wifi_html_end[]    asm("_binary_wifi_html_end");

#define VERSION "0.0.2a"
#define REVISION 0

typedef struct {
    char name[32];
    char body[256];
} macro_t;

static macro_t macros[10];
static bool hid_enumerated = false; /* set to true when USB HID host enumerates the device */

static bool    sta_connected = false;
static char    sta_ip[16]    = "";

/* ---- WiFi credential storage (NVS) ---- */
#define WIFI_CRED_NS  "wifi_cfg"
#define WIFI_CRED_MAX 10

typedef struct { char ssid[33]; char pass[65]; } wifi_cred_t;
static wifi_cred_t wifi_creds[WIFI_CRED_MAX];
static int         wifi_cred_count = 0;

static void creds_save(void){
    nvs_handle_t h;
    if(nvs_open(WIFI_CRED_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, "count", (uint8_t)wifi_cred_count);
    for(int i = 0; i < wifi_cred_count; i++){
        char key[16];
        snprintf(key, sizeof(key), "ssid_%d", i);
        nvs_set_str(h, key, wifi_creds[i].ssid);
        snprintf(key, sizeof(key), "pass_%d", i);
        nvs_set_str(h, key, wifi_creds[i].pass);
    }
    nvs_commit(h);
    nvs_close(h);
}

static void creds_load(void){
    nvs_handle_t h;
    if(nvs_open(WIFI_CRED_NS, NVS_READONLY, &h) != ESP_OK) return;
    uint8_t cnt = 0;
    nvs_get_u8(h, "count", &cnt);
    if(cnt > WIFI_CRED_MAX) cnt = WIFI_CRED_MAX;
    wifi_cred_count = cnt;
    for(int i = 0; i < cnt; i++){
        char key[16];
        size_t len = sizeof(wifi_creds[i].ssid);
        snprintf(key, sizeof(key), "ssid_%d", i);
        nvs_get_str(h, key, wifi_creds[i].ssid, &len);
        len = sizeof(wifi_creds[i].pass);
        snprintf(key, sizeof(key), "pass_%d", i);
        nvs_get_str(h, key, wifi_creds[i].pass, &len);
    }
    nvs_close(h);
}

/* Add or update credential. If SSID exists, update password.
   If full, evict oldest (index 0) to make room. */
static void creds_add(const char *ssid, const char *pass){
    for(int i = 0; i < wifi_cred_count; i++){
        if(strcmp(wifi_creds[i].ssid, ssid) == 0){
            strncpy(wifi_creds[i].pass, pass, sizeof(wifi_creds[i].pass)-1);
            creds_save();
            return;
        }
    }
    if(wifi_cred_count == WIFI_CRED_MAX){
        /* shift out oldest */
        memmove(&wifi_creds[0], &wifi_creds[1], sizeof(wifi_cred_t) * (WIFI_CRED_MAX - 1));
        wifi_cred_count--;
    }
    memset(&wifi_creds[wifi_cred_count], 0, sizeof(wifi_cred_t));
    strncpy(wifi_creds[wifi_cred_count].ssid, ssid, sizeof(wifi_creds[0].ssid)-1);
    strncpy(wifi_creds[wifi_cred_count].pass, pass, sizeof(wifi_creds[0].pass)-1);
    wifi_cred_count++;
    creds_save();
}

static void creds_delete(int idx){
    if(idx < 0 || idx >= wifi_cred_count) return;
    memmove(&wifi_creds[idx], &wifi_creds[idx+1],
            sizeof(wifi_cred_t) * (wifi_cred_count - idx - 1));
    memset(&wifi_creds[wifi_cred_count-1], 0, sizeof(wifi_cred_t));
    wifi_cred_count--;
    creds_save();
}

static void wifi_do_connect(int idx){
    if(idx < 0 || idx >= wifi_cred_count) return;
    wifi_config_t cfg = {};
    strncpy((char*)cfg.sta.ssid,     wifi_creds[idx].ssid, sizeof(cfg.sta.ssid)-1);
    strncpy((char*)cfg.sta.password, wifi_creds[idx].pass, sizeof(cfg.sta.password)-1);
    sta_connected = false;
    sta_ip[0] = 0;
    esp_wifi_disconnect();
    esp_wifi_set_config(WIFI_IF_STA, &cfg);
    esp_wifi_connect();
    printf("Connecting to SSID: %s\n", wifi_creds[idx].ssid);
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data){
    if(base == IP_EVENT && id == IP_EVENT_STA_GOT_IP){
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        esp_ip4addr_ntoa(&ev->ip_info.ip, sta_ip, sizeof(sta_ip));
        sta_connected = true;
        printf("STA connected, IP: %s\n", sta_ip);
    } else if(base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED){
        sta_connected = false;
        sta_ip[0] = 0;
        printf("STA disconnected\n");
    }
}

static void wifi_init_ap(void){
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
        nvs_flash_erase();
        nvs_flash_init();
    }
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,      wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT,   IP_EVENT_STA_GOT_IP,   wifi_event_handler, NULL);

    wifi_config_t ap = {
        .ap = {
            .ssid = "ESP-MACRO",
            .password = "12345678",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        }
    };
    esp_wifi_set_mode(WIFI_MODE_APSTA);
    esp_wifi_set_config(WIFI_IF_AP, &ap);
    esp_wifi_start();

    creds_load();
    if(wifi_cred_count > 0) wifi_do_connect(0);
}

static void backend_send(const char *s){
#if CONFIG_IDF_TARGET_ESP32C2
    printf("[C2 TEST OUTPUT] %s\n", s);
#else
    printf("[USB HID PLACEHOLDER] %s\n", s);
#endif
}

static void run_macro_script(const char *script){
    char buf[512];
    strncpy(buf, script, sizeof(buf)-1);
    buf[sizeof(buf)-1]=0;

    char *line = strtok(buf, "\n");
    while(line){
        if(strncmp(line,"STRING ",7)==0){
            backend_send(line+7);
        } else if(strncmp(line,"KEY ",4)==0){
            backend_send(line);
        } else if(strncmp(line,"COMBO ",6)==0){
            backend_send(line);
        } else if(strncmp(line,"DELAY ",6)==0){
            int ms=atoi(line+6);
            vTaskDelay(pdMS_TO_TICKS(ms));
        }
        line = strtok(NULL,"\n");
    }
}

static esp_err_t root_get(httpd_req_t *req){
    size_t len = index_html_end - index_html_start;
    httpd_resp_set_type(req,"text/html");
    httpd_resp_send(req,(const char*)index_html_start,len);
    return ESP_OK;
}

static esp_err_t send_post(httpd_req_t *req){
    char buf[512];
    int len=httpd_req_recv(req,buf,sizeof(buf)-1);
    if(len<0) len=0;
    buf[len]=0;
    backend_send(buf);
    httpd_resp_sendstr(req,"OK");
    return ESP_OK;
}

static esp_err_t macro_save(httpd_req_t *req){
    char buf[512];
    int len=httpd_req_recv(req,buf,sizeof(buf)-1);
    if(len<0) len=0;
    buf[len]=0;

    char *sep=strchr(buf,'|');
    if(!sep){ httpd_resp_sendstr(req,"ERR"); return ESP_OK; }
    *sep=0;
    char *name=buf;
    char *body=sep+1;

    for(int i=0;i<10;i++){
        if(macros[i].name[0]==0 || strcmp(macros[i].name,name)==0){
            strncpy(macros[i].name,name,sizeof(macros[i].name)-1);
            strncpy(macros[i].body,body,sizeof(macros[i].body)-1);
            break;
        }
    }
    httpd_resp_sendstr(req,"OK");
    return ESP_OK;
}

static esp_err_t macro_list(httpd_req_t *req){
    char out[2048];
    strcpy(out,"[");
    int first=1;
    for(int i=0;i<10;i++){
        if(macros[i].name[0]){
            if(!first) strcat(out,",");
            first=0;
            strcat(out,"{\"id\":");
            char t[16]; sprintf(t,"%d",i); strcat(out,t);
            strcat(out,",\"name\":\"");
            strcat(out,macros[i].name);
            strcat(out,"\"}");
        }
    }
    strcat(out,"]");
    httpd_resp_set_type(req,"application/json");
    httpd_resp_sendstr(req,out);
    return ESP_OK;
}

static esp_err_t status_get(httpd_req_t *req){
    wifi_sta_list_t sta;
    esp_wifi_ap_get_sta_list(&sta);
    char out[128];
    snprintf(out, sizeof(out), "{\"hid\":%s,\"clients\":%d,\"sta\":%s,\"ip\":\"%s\"}",
             hid_enumerated ? "true" : "false",
             sta.num,
             sta_connected ? "true" : "false",
             sta_ip);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out);
    return ESP_OK;
}

static esp_err_t wifi_page_get(httpd_req_t *req){
    size_t len = wifi_html_end - wifi_html_start;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char*)wifi_html_start, len);
    return ESP_OK;
}

static esp_err_t wifi_disconnect_post(httpd_req_t *req){
    sta_connected = false;
    sta_ip[0] = 0;
    esp_wifi_disconnect();
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t wifi_connect_post(httpd_req_t *req){
    char buf[128];
    int len = httpd_req_recv(req, buf, sizeof(buf)-1);
    if(len < 0) len = 0;
    buf[len] = 0;

    char *sep = strchr(buf, '|');
    if(!sep){ httpd_resp_sendstr(req, "ERR"); return ESP_OK; }
    *sep = 0;
    const char *ssid = buf, *pass = sep+1;

    creds_add(ssid, pass);  /* save to NVS */

    int idx = 0;
    for(int i = 0; i < wifi_cred_count; i++)
        if(strcmp(wifi_creds[i].ssid, ssid) == 0){ idx = i; break; }
    wifi_do_connect(idx);

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t wifi_list_get(httpd_req_t *req){
    char out[1024];
    int pos = 0;
    pos += snprintf(out+pos, sizeof(out)-pos, "[");
    for(int i = 0; i < wifi_cred_count; i++){
        if(i) pos += snprintf(out+pos, sizeof(out)-pos, ",");
        pos += snprintf(out+pos, sizeof(out)-pos,
                        "{\"idx\":%d,\"ssid\":\"%s\"}", i, wifi_creds[i].ssid);
    }
    pos += snprintf(out+pos, sizeof(out)-pos, "]");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out);
    return ESP_OK;
}

static esp_err_t wifi_delete_post(httpd_req_t *req){
    char buf[8];
    int len = httpd_req_recv(req, buf, sizeof(buf)-1);
    if(len < 0) len = 0;
    buf[len] = 0;
    creds_delete(atoi(buf));
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t wifi_connect_idx_post(httpd_req_t *req){
    char buf[8];
    int len = httpd_req_recv(req, buf, sizeof(buf)-1);
    if(len < 0) len = 0;
    buf[len] = 0;
    wifi_do_connect(atoi(buf));
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t macro_run(httpd_req_t *req){
    char buf[16];
    int len=httpd_req_recv(req,buf,sizeof(buf)-1);
    if(len<0) len=0;
    buf[len]=0;
    int id=atoi(buf);
    if(id>=0 && id<10 && macros[id].name[0]){
        run_macro_script(macros[id].body);
    }
    httpd_resp_sendstr(req,"OK");
    return ESP_OK;
}

static esp_err_t wifi_scan_get(httpd_req_t *req){
    wifi_scan_config_t scan_cfg = {
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300,
    };
    esp_wifi_scan_start(&scan_cfg, true); /* blocking */

    uint16_t count = 20;
    wifi_ap_record_t *records = malloc(count * sizeof(wifi_ap_record_t));
    if(!records){ httpd_resp_sendstr(req, "[]"); return ESP_OK; }
    esp_wifi_scan_get_ap_records(&count, records);

    char *out = malloc(2048);
    if(!out){ free(records); httpd_resp_sendstr(req, "[]"); return ESP_OK; }

    int pos = 0;
    pos += snprintf(out+pos, 2048-pos, "[");
    for(int i = 0; i < count; i++){
        if(i) pos += snprintf(out+pos, 2048-pos, ",");
        char safe_ssid[67] = {0};
        int si = 0;
        for(int j = 0; records[i].ssid[j] && si < 64; j++){
            if(records[i].ssid[j] == '"') safe_ssid[si++] = '\\';
            safe_ssid[si++] = records[i].ssid[j];
        }
        pos += snprintf(out+pos, 2048-pos,
            "{\"ssid\":\"%s\",\"rssi\":%d,\"auth\":%d}",
            safe_ssid, records[i].rssi, records[i].authmode);
    }
    pos += snprintf(out+pos, 2048-pos, "]");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out);
    free(out);
    free(records);
    return ESP_OK;
}

static void web_start(void){
    httpd_handle_t server=NULL;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 16;
    httpd_start(&server,&cfg);

    httpd_uri_t u1={.uri="/",.method=HTTP_GET,.handler=root_get};
    httpd_uri_t u2={.uri="/send",.method=HTTP_POST,.handler=send_post};
    httpd_uri_t u3={.uri="/macro/save",.method=HTTP_POST,.handler=macro_save};
    httpd_uri_t u4={.uri="/macro/list",.method=HTTP_GET,.handler=macro_list};
    httpd_uri_t u5={.uri="/macro/run",.method=HTTP_POST,.handler=macro_run};
    httpd_uri_t u6={.uri="/status",.method=HTTP_GET,.handler=status_get};
    httpd_uri_t u7={.uri="/wifi/connect",.method=HTTP_POST,.handler=wifi_connect_post};
    httpd_uri_t u8 ={.uri="/wifi",.method=HTTP_GET,.handler=wifi_page_get};
    httpd_uri_t u9 ={.uri="/wifi/disconnect",.method=HTTP_POST,.handler=wifi_disconnect_post};
    httpd_uri_t u10={.uri="/wifi/list",.method=HTTP_GET,.handler=wifi_list_get};
    httpd_uri_t u11={.uri="/wifi/delete",.method=HTTP_POST,.handler=wifi_delete_post};
    httpd_uri_t u12={.uri="/wifi/connect_idx",.method=HTTP_POST,.handler=wifi_connect_idx_post};
    httpd_uri_t u13={.uri="/wifi/scan",.method=HTTP_GET,.handler=wifi_scan_get};

    httpd_register_uri_handler(server,&u1);
    httpd_register_uri_handler(server,&u2);
    httpd_register_uri_handler(server,&u3);
    httpd_register_uri_handler(server,&u4);
    httpd_register_uri_handler(server,&u5);
    httpd_register_uri_handler(server,&u6);
    httpd_register_uri_handler(server,&u7);
    httpd_register_uri_handler(server,&u8);
    httpd_register_uri_handler(server,&u9);
    httpd_register_uri_handler(server,&u10);
    httpd_register_uri_handler(server,&u11);
    httpd_register_uri_handler(server,&u12);
    httpd_register_uri_handler(server,&u13);
}

void app_main(void){
    wifi_init_ap();
    web_start();
    printf("AP: ESP-MACRO  PASS:12345678\n");
}
