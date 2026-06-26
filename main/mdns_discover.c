#include "mdns_discover.h"
#include "mdns.h"
#include "lwip/inet.h"
#include "esp_log.h"
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>

#define SERVICE_TYPE   "_epaper"
#define SERVICE_PROTO  "_tcp"
#define QUERY_TIMEOUT  5000   // ms per attempt

static const char *TAG = "MDNS";

esp_err_t mdns_discover_server(char     *ip_out,
                                size_t    ip_len,
                                uint16_t *port_out)
{
    ESP_LOGI(TAG, "Searching for EPaperViewer...");

    mdns_result_t *results = NULL;
    esp_err_t      err     = mdns_query_ptr(SERVICE_TYPE,
                                             SERVICE_PROTO,
                                             QUERY_TIMEOUT,
                                             5,            // max results
                                             &results);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mDNS query error: %s", esp_err_to_name(err));
        return err;
    }

    if (results == NULL) {
        ESP_LOGW(TAG, "No EPaperViewer service found");
        return ESP_ERR_NOT_FOUND;
    }

    // Walk results looking for an IPv4 address
    for (mdns_result_t *r = results; r != NULL; r = r->next) {
        for (mdns_ip_addr_t *a = r->addr; a != NULL; a = a->next) {
            if (a->addr.type == ESP_IPADDR_TYPE_V4) {
                inet_ntop(AF_INET, &a->addr.u_addr.ip4, ip_out, ip_len);
                *port_out = r->port;
                ESP_LOGI(TAG, "Found EPaperViewer at %s:%d", ip_out, *port_out);
                mdns_query_results_free(results);
                return ESP_OK;
            }
        }
    }

    ESP_LOGW(TAG, "Service found but no IPv4 address");
    mdns_query_results_free(results);
    return ESP_ERR_NOT_FOUND;
}