
#ifndef CUSTOM_SERVER_H
#define CUSTOM_SERVER_H

#include <stdint.h>
#include <unordered_map>
#include <SPIFFS.h>

class AsyncWebServer;
class AsyncWebServerRequest;
class String;

class CustomServer {
private:
  AsyncWebServer* m_server;
  uint16_t m_port;
  const char* m_params_f;
  const char* m_nodes_f;
  const char* m_cert_f;
  String processor(const String& var);
  void startServer();
  static bool save(const char* file, AsyncWebServerRequest* request, const char* object_name);
  void handleUpload(AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final);
  std::unordered_map<std::string, File*> file_map;

public:
  CustomServer(uint16_t server_port, const char* params_file, const char* nodes_file, const char* cert_file);
  ~CustomServer();
  bool start();

//  template<typename T>
//  static T find(const String& path);
};

#endif // CUSTOM_SERVER_H