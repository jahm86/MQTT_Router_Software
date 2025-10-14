
#include "CustomServer.h"
#include "definitions.h"
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <vector>
#include <ESPAsyncWebServer.h>

using namespace std;

// Forward-ensured size for vector, optimized to allocate once
#define RESERVE_VCT_SIZE 3

// Search into object for path marked by object_name. object_name has subpaths separated by ND_SEPARATOR character. If object in path
// doesn't exist, force_creation will determine if create object in path/subpath, when set, or exit until it got object. If last_also_object
// is also set, the last path in object will be object.
static JsonVariant searchInto(JsonObject object, const char* object_name, bool force_creation = false, bool last_also_object = false);

CustomServer::CustomServer(uint16_t server_port, const char* params_file, const char* nodes_file)
    : m_server(nullptr), m_port(server_port), m_params_f(params_file), m_nodes_f(nodes_file) {}

CustomServer::~CustomServer() {
    delete m_server;
}

bool CustomServer::start() {
  startServer();
  return false;
}

//template<typename T>
//T CustomServer::find(const String& path) {
//  JsonDocument doc;
//  File file = SPIFFS.open(Params_path, FILE_READ);
//  deserializeJson(doc, file);
//  file.close();
//  JsonObject obj = doc.as<JsonObject>();
//  JsonVariant var = searchInto(obj, path.c_str());
//  return var.as<T>();
//}

//template int CustomServer::find<int>(const String&);

void CustomServer::startServer() {
  m_server = new AsyncWebServer(m_port);
  // Request main page
  m_server->on("/", HTTP_GET, [this](AsyncWebServerRequest* request) {
    log_d("Main page file requested");
    request->send(SPIFFS, Serve_file, "text/html", false, [this](const String &var) -> String { return this->processor(var); });
  });
  // Request style file
  m_server->on(Css_file, HTTP_GET, [](AsyncWebServerRequest* request) {
    log_d("Style file requested");
    request->send(SPIFFS, Css_file, "text/css");
  });
  // Request script file
  m_server->on(Js_file, HTTP_GET, [](AsyncWebServerRequest* request) {
    log_d("Script file requested");
    request->send(SPIFFS, Js_file, "text/javascript");
  });
  // Request icon file
  m_server->on(Icon_file, HTTP_GET, [](AsyncWebServerRequest* request) {
    log_d("Icon file requested");
    request->send(SPIFFS, Icon_file, "image/png");
  });
  // Request any file
  //server.serveStatic("/", SPIFFS, "/"); // Gives linking error " undefined reference to `mbedtls_md5_starts'"
  // Request reset
  m_server->on("/reset", HTTP_POST, [](AsyncWebServerRequest* request) {
    log_d("Restart requested");
    request->send(200, "text/plain", F("ESP32 will now restart..."));
    Serial.println(F("ESP32 will now restart..."));
    delay(2000);
    request->redirect("/");
    delay(1000);
    log_d("Restarting...");
    ESP.restart();
  });
  // Request WiFi parameters
  m_server->on("/wifi", HTTP_POST, [this](AsyncWebServerRequest* request) {
    log_d("Network parameters save requested");
    CustomServer::save(m_params_f, request, WF_OBJ_NAME);
    request->redirect("/");
  });
  // Request MQTT parameters
  m_server->on("/mqtt", HTTP_POST, [this](AsyncWebServerRequest* request) {
    log_d("MQTT parameters save requested");
    CustomServer::save(m_params_f, request, MQ_OBJ_NAME);
    request->redirect("/");
  });
  // Request Serial parameters
  m_server->on("/serial", HTTP_POST, [this](AsyncWebServerRequest* request) {
    log_d("Serial parameters save requested");
    CustomServer::save(m_params_f, request, SP_OBJ_NAME);
    request->redirect("/");
  });
  // Request save communicating nodes parameters
  m_server->on("/nodes", HTTP_POST, [this](AsyncWebServerRequest* request) {
    log_d("Data Nodes save requested");
    CustomServer::saveItem(m_nodes_f, request, ND_OBJ_NAME);
    request->redirect("/");
  });
  // Request communicating nodes file
  m_server->on("/nodes", HTTP_GET, [this](AsyncWebServerRequest* request) {
    log_d("Data Nodes file requested");
    request->send(SPIFFS, m_nodes_f, "text/plain");
  });
  // When requesting not existant page
  m_server->onNotFound([](AsyncWebServerRequest* request) {
    log_d("Not found Url requested: %s", request->url());
    request->send(404, "text/plain", "Not found!");
  });
  // Start server
  m_server->begin();
}

// Save raw value of item given, found in request to file file_name. Retruns true if save is sussesfull.
bool CustomServer::saveItem(const char* file_name, AsyncWebServerRequest* request, const char* item) {
  // Search item in request
  log_d("Searching for %s...", item);
  if (!request->hasParam(item, true)) {
    log_e("Failed to find item");
    return false;
  }
  const String& message = request->getParam(item, true)->value();
  // Open file for writing
  log_d("Found! Opening file");
  File file = SPIFFS.open(file_name, FILE_WRITE);
  if(!file) {
    log_e("Failed to open file for writing");
    return false;
  }
  // Save to file
  log_d("Content to save:\n%s", message.c_str());
  if (!file.print(message)) {
    log_e("Failed to write content");
    file.close();
    return false;
  }
  log_d("Save Susessfull!!!");
  file.close();
  return true;
}

// Save JSON object by file file_name, object name path and list of request parameters. Retruns true if save is sussesfull.
bool CustomServer::save(const char* file_name, AsyncWebServerRequest* request, const char* object_name) {
  // Open file for reading and create the JsonDocument
  JsonDocument doc;
  File file = SPIFFS.open(file_name, FILE_READ);
  if(!file)
    log_i("Failed to open file for reading");
  // Parse directly from file
  deserializeJson(doc, file);
  file.close();
  JsonObject obj = doc.as<JsonObject>();
  if (obj.isNull()) {
    log_d("Creating new JSON file...");
    obj = doc.to<JsonObject>();
  }
  // Extract substrings with indicated delimiter and search subpaths
  obj = searchInto(obj, object_name, true, true);
  // Extracting parameters from request
  int params = request->params();
  for (int index = 0; index < params; ++index) {
    AsyncWebParameter* p = request->getParam(index);
    if (!p->isPost()) {
      log_e("Parameter should be post!");
      continue;
    }
    log_d("Saving (%d): object[%s][%s] = %s", index, object_name, p->name().c_str(), p->value().c_str());
    obj[p->name()] = p->value();
  }
  // Open file for writing
  file = SPIFFS.open(file_name, FILE_WRITE);
  if(!file) {
    log_e("Failed to open file for writing");
    return false;
  }
  // Write a prettified JSON document to the file
  serializeJsonPretty(doc, file);
  file.close();
  log_d("Save Susessfull!!!");
  return true;
}

// Replaces placeholder with corresponding value
String CustomServer::processor(const String& var) {
  if (var == ND_OBJ_NAME) {
    return String();
  } else {
    log_d("Proccess parameter: %s", var.c_str());
    // Open file for reading and create the JsonDocument
    JsonDocument doc;
    File file = SPIFFS.open(m_params_f, FILE_READ);
    deserializeJson(doc, file);
    file.close();
    JsonVariant variant = searchInto(doc.as<JsonObject>(), var.c_str());
    if (variant.is<String>())
      return variant.as<String>();
  }
  return String();
}

JsonVariant searchInto(JsonObject object, const char* object_name, bool force_creation, bool last_also_object) {
  vector<string> vct;
  vct.reserve(RESERVE_VCT_SIZE);
  string str;
  int index;
  stringstream ss(object_name);
  // First, store all object names in vector
  while (getline(ss, str, ND_SEPARATOR)) {
    if (str.empty()) {
      //log_w("Empty subpath");
      log_w("Empty subpath. Ignoring...");
      continue;
    }
    vct.push_back(str);
  }
  const int vctsize = vct.size();
  // Then, enter in recursive objects, but lets the last being variant
  for (index = 0; index < vctsize - 1; ++index) {
    const string& strr = vct.at(index);
    if (!object[strr].is<JsonObject>()) {
      if (force_creation) {
        log_d("Creating object path %s", strr.c_str());
        object[strr].to<JsonObject>();
      } else {
        log_d("Not completed. Exit here");
        return object[strr];
      }
    }
    object = object[strr].as<JsonObject>();
  }
  // Finally, returns variant
  if (vctsize == 0)
    return object;
  const string& strr = vct.at(index);
  if (force_creation && last_also_object && !object[strr].is<JsonObject>()) {
    log_d("Creating last object path %s", strr.c_str());
    object[strr].to<JsonObject>();
  }
  return object[strr];
}
