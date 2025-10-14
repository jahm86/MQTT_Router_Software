
#ifndef CUSTOM_SERVER_H
#define CUSTOM_SERVER_H

#include <stdint.h>

class AsyncWebServer;
class AsyncWebServerRequest;
class String;

class CustomServer {
private:
  AsyncWebServer* m_server;
  uint16_t m_port;
  const char* m_params_f;
  const char* m_nodes_f;
  String processor(const String& var);
  void startServer();
  static bool save(const char* file, AsyncWebServerRequest* request, const char* object_name);
  static bool saveItem(const char* file, AsyncWebServerRequest* request, const char* item);

public:
  CustomServer(uint16_t server_port, const char* params_file, const char* nodes_file); //WEBSERVER_PORT
  ~CustomServer();
  bool start();

//  template<typename T>
//  static T find(const String& path);
};

#endif // CUSTOM_SERVER_H