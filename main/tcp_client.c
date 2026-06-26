#include "tcp_client.h"
#include "epaper_protocol.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "it8951.h"
#include "button.h"
#include "mdns_discover.h"
#include "esp_log.h"
#include <errno.h>
#include <string.h>

#define CONNECT_TIMEOUT_S 10
#define RETRY_DELAY_MS 3000

static const char *TAG = "TCP";

// Shared state
static uint8_t *s_fb = NULL;
static size_t s_fb_size = 0;
static int s_sock = -1;


// ----------------------------------------------------------
// Helpers
// ----------------------------------------------------------
static bool recv_exact(int sock, void *buf, size_t len)
{
    size_t got = 0;
    uint8_t *ptr = (uint8_t *)buf;
    while (got < len)
    {
        // Read in 4KB chunks so LwIP can clear its network buffers
        size_t to_read = len - got;
        if (to_read > 4096) {
            to_read = 4096; 
        }

        int n = recv(sock, ptr + got, to_read, 0);
        if (n <= 0)
        {
            ESP_LOGE(TAG, "recv_exact failed at %d/%d: errno=%d",
                     (int)got, (int)len, errno);
            return false;
        }
        got += n;
    }
    return true;
}

static bool send_exact(int sock, const void *buf, size_t len)
{
    size_t sent = 0;
    const uint8_t *ptr = (const uint8_t *)buf;
    while (sent < len)
    {
        int n = send(sock, ptr + sent, len - sent, 0);
        if (n < 0)
        {
            ESP_LOGE(TAG, "send_exact failed: errno=%d", errno);
            return false;
        }
        sent += n;
    }
    return true;
}

static bool recv_screen(int sock)
{
    if (!recv_exact(sock, s_fb, s_fb_size))
        return false;

    ESP_LOGD(TAG, "Screen received");
    uint8_t ack_byte = 0xFF;
    send(sock, &ack_byte, 1, 0);
    return true;
}


// ----------------------------------------------------------
// Session — runs for the lifetime of one connection
// ----------------------------------------------------------
static void run_session(int sock)
{
    const IT8951DevInfo *dev_info = IT8951_GetDevInfo(); 
    IT8951LdImgInfo img_info = {
        .usEndianType = 1, // Big Endian
        .usPixelFormat = IT8951_4BPP, // 4bpp
        .usRotate = 0, // No rotation
        .ulStartFBAddr = (uint32_t)s_fb,
        .ulImgBufBaseAddr = IT8951_GetImgBufAddr()
    };

    IT8951AreaImgInfo area_info = {
        .usX = 0,
        .usY = 0,
        .usWidth = dev_info->usPanelW,
        .usHeight = dev_info->usPanelH
    };

    uint8_t btn_cmd;
    btn_cmd = CMD_INIT;

    if (!send_exact(sock, &btn_cmd, 1))
        return;
        
    if (!recv_screen(sock))
        return;

    IT8951_HostAreaPackedPixelWrite(&img_info, &area_info);
    IT8951_DisplayArea(0, 0, dev_info->usPanelW, dev_info->usPanelH, MODE_GC16, 25);

    // Step 2: Forward button events, receive new screen each time
    // Buttons are read from a queue populated by GPIO ISR
    // (see button_task — added in next step)
    while (1)
    {
        // Block waiting for a button event
        if(button_wait(&btn_cmd, 50 / portTICK_PERIOD_MS))
        {
            if (!send_exact(sock, &btn_cmd, 1))
                return;
        }

        uint8_t peek_buf;
        int n = recv(sock, &peek_buf, 1, MSG_PEEK | MSG_DONTWAIT);

        if (n > 0) 
        {
            // Data is arriving from PC! Jump into standard blocking read.
            ESP_LOGI(TAG, "Incoming screen data from PC...");
            
            if (!recv_screen(sock)) 
                return; // Socket broke
            
            IT8951_WaitForDisplayReady();
            IT8951_HostAreaPackedPixelWrite(&img_info, &area_info);
            IT8951_DisplayArea(0, 0, dev_info->usPanelW, dev_info->usPanelH, MODE_GC16, 25);
            
            // 3. Send 0xFF ACK back to PC to confirm update is finished
            uint8_t ack_byte = 0xFF;
            send_exact(sock, &ack_byte, 1);
            ESP_LOGI(TAG, "Update finished. ACK sent.");
        } 
        else if (n == 0) 
        {
            // Socket was closed gracefully by the server
            ESP_LOGW(TAG, "Server closed connection");
            return;
        } 
        else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) 
        {
            // Real network error (Not just an empty buffer)
            ESP_LOGE(TAG, "Socket error %d", errno);
            return;
        }
    }
}

// ----------------------------------------------------------
// Main task — connect loop with automatic reconnect
// ----------------------------------------------------------
static void tcp_client_task(void *arg)
{

    char     server_ip[16] = {0};
    uint16_t server_port   = 0;

    while (1)
    {
        if (server_ip[0] == '\0') {
            while (mdns_discover_server(server_ip,
                                        sizeof(server_ip),
                                        &server_port) != ESP_OK) {
                ESP_LOGW(TAG, "Retrying mDNS in 3s...");
                vTaskDelay(pdMS_TO_TICKS(3000));
            }
        }

        // --- Create socket ---
        struct addrinfo  hints = { .ai_socktype = SOCK_STREAM };
        struct addrinfo *addr  = NULL;
        char             port_str[6];
        snprintf(port_str, sizeof(port_str), "%d", server_port);

        if (getaddrinfo(server_ip, port_str, &hints, &addr) != 0 || !addr) {
            ESP_LOGE(TAG, "getaddrinfo failed");
            vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
            continue;
        }

        // --- Create socket ---
        s_sock = socket(addr->ai_family,
                        addr->ai_socktype,
                        addr->ai_protocol);
        if (s_sock < 0)
        {
            ESP_LOGE(TAG, "socket() failed: %d", errno);
            freeaddrinfo(addr);
            vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
            continue;
        }

        int nodelay_flag = 1;

        // Blocking timeout — no manual select() needed
        struct timeval tv = {.tv_sec = CONNECT_TIMEOUT_S};
        setsockopt(s_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(s_sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        if (setsockopt(s_sock, IPPROTO_TCP, TCP_NODELAY, &nodelay_flag, sizeof(nodelay_flag)) < 0) {
            ESP_LOGW(TAG, "Failed to set TCP_NODELAY");
        } else {
            ESP_LOGI(TAG, "TCP_NODELAY enabled - button lag should be gone!");
        }


        if (connect(s_sock, addr->ai_addr, addr->ai_addrlen) != 0)
        {
            ESP_LOGW(TAG, "connect() failed (%d) — retry in %dms",
                     errno, RETRY_DELAY_MS);
            close(s_sock);
            s_sock = -1;
            freeaddrinfo(addr);
            vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
            continue;
        }

        freeaddrinfo(addr);

        // --- Run until disconnected ---
        run_session(s_sock);

        // --- Clean up and retry ---
        close(s_sock);
        s_sock = -1;
        ESP_LOGW(TAG, "Disconnected — reconnecting in %dms", RETRY_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
    }
}

// ----------------------------------------------------------
// Public API
// ----------------------------------------------------------
esp_err_t tcp_client_start(uint8_t *fb, size_t fb_size)
{
    s_fb = fb;
    s_fb_size = fb_size;

    ESP_LOGI(TAG, "framebuffer size: %d", s_fb_size);
    // TCP task — higher priority
    xTaskCreate(tcp_client_task, "tcp", 8192, NULL, 5, NULL);

    return ESP_OK;
}