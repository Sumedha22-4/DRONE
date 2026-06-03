/*
  ============================================================
  XIAO ESP32S3 Sense - OV3660 Live MJPEG Stream Server
  Board: Seeed Studio XIAO ESP32S3 Sense
  Camera: OV3660
  
  HOW TO USE:
  1. Set your WiFi SSID and PASSWORD below
  2. Upload via Arduino IDE
  3. Open Serial Monitor at 115200 baud
  4. Note the IP address printed
  5. Open browser → http://<IP_ADDRESS>/stream
  ============================================================
*/

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

// ============================================================
//  CONFIGURE YOUR WIFI HERE
// ============================================================
const char* ssid     = "Redmi 9A";
const char* password = "e93a4dbce8e6";
// ============================================================

// ----- XIAO ESP32S3 Sense camera pin definitions -----
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    10
#define SIOD_GPIO_NUM    40
#define SIOC_GPIO_NUM    39

#define Y9_GPIO_NUM      48
#define Y8_GPIO_NUM      11
#define Y7_GPIO_NUM      12
#define Y6_GPIO_NUM      14
#define Y5_GPIO_NUM      16
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM      17
#define Y2_GPIO_NUM      15
#define VSYNC_GPIO_NUM   38
#define HREF_GPIO_NUM    47
#define PCLK_GPIO_NUM    13

// ---- HTTP part-boundary for MJPEG ----
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY =
    "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

// ---- Root page (redirect to /stream) ----
static esp_err_t root_handler(httpd_req_t* req) {
  const char* html =
    "<!DOCTYPE html><html><head>"
    "<meta http-equiv='refresh' content='0;url=/stream'/>"
    "</head><body>"
    "<p>Redirecting to stream... "
    "or <a href='/stream'>click here</a></p>"
    "</body></html>";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, strlen(html));
}

// ---- MJPEG stream handler ----
static esp_err_t stream_handler(httpd_req_t* req) {
  camera_fb_t* fb = NULL;
  esp_err_t res = ESP_OK;
  char part_buf[64];

  httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "30");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
      break;
    }

    // Send boundary
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

    // Send part header
    size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
    res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

    // Send JPEG data
    res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;
  }

  return res;
}

// ---- Snapshot handler (single JPEG) ----
static esp_err_t capture_handler(httpd_req_t* req) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition",
                     "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

void startCameraServer() {
  // --- Stream server on port 80 ---
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port   = 32768;

  httpd_uri_t root_uri = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = root_handler,
    .user_ctx = NULL
  };
  httpd_uri_t stream_uri = {
    .uri      = "/stream",
    .method   = HTTP_GET,
    .handler  = stream_handler,
    .user_ctx = NULL
  };
  httpd_uri_t capture_uri = {
    .uri      = "/capture",
    .method   = HTTP_GET,
    .handler  = capture_handler,
    .user_ctx = NULL
  };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &root_uri);
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    httpd_register_uri_handler(stream_httpd, &capture_uri);
    Serial.println("Stream server started on port 80");
  } else {
    Serial.println("Failed to start stream server!");
  }
}

void initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;          // 20 MHz XCLK
  config.pixel_format = PIXFORMAT_JPEG;

  // Choose resolution based on PSRAM availability
  if (psramFound()) {
    config.frame_size   = FRAMESIZE_VGA;   // 640×480  (change to SVGA/XGA if needed)
    config.jpeg_quality = 12;              // 0=best 63=worst  (10–15 is good balance)
    config.fb_count     = 2;
    config.grab_mode    = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size   = FRAMESIZE_QVGA;  // 320×240 fallback
    config.jpeg_quality = 18;
    config.fb_count     = 1;
    config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init FAILED: 0x%x\n", err);
    Serial.println("Check camera connection and retry.");
    while (true) { delay(1000); }
  }
  Serial.println("Camera initialised OK");

  // OV3660 sensor tweaks for better colour / exposure
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 1);      // -2 to 2
    s->set_contrast(s, 1);        // -2 to 2
    s->set_saturation(s, 0);      // -2 to 2
    s->set_sharpness(s, 1);       // -2 to 2
    s->set_whitebal(s, 1);        // auto white balance ON
    s->set_awb_gain(s, 1);        // AWB gain ON
    s->set_wb_mode(s, 0);         // 0=auto
    s->set_exposure_ctrl(s, 1);   // auto exposure ON
    s->set_aec2(s, 1);            // AEC DSP ON
    s->set_gain_ctrl(s, 1);       // auto gain ON
    s->set_agc_gain(s, 0);
    s->set_gainceiling(s, (gainceiling_t)2);
    s->set_bpc(s, 0);             // black pixel correction
    s->set_wpc(s, 1);             // white pixel correction
    s->set_raw_gma(s, 1);
    s->set_lenc(s, 1);            // lens correction
    s->set_hmirror(s, 0);         // flip if needed
    s->set_vflip(s, 0);
    s->set_dcw(s, 1);
    s->set_colorbar(s, 0);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== XIAO ESP32S3 Camera Stream ===");

  // Init camera first
  initCamera();

  // Connect to WiFi
  Serial.printf("Connecting to WiFi: %s", ssid);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);          // disable WiFi sleep for stable stream

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (++attempts > 40) {
      Serial.println("\nWiFi connection FAILED. Check credentials and restart.");
      while (true) { delay(1000); }
    }
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  startCameraServer();

  Serial.println("------------------------------------------");
  Serial.printf(" Stream  → http://%s/stream\n",
                WiFi.localIP().toString().c_str());
  Serial.printf(" Snapshot → http://%s/capture\n",
                WiFi.localIP().toString().c_str());
  Serial.println("------------------------------------------");
}

void loop() {
  // Nothing needed — HTTP server runs in background tasks
  delay(10000);
}
