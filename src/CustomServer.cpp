
#include "CustomServer.h"
#include "definitions.h"
#include <ArduinoJson.h>
#include <vector>
#include <ESPAsyncWebServer.h>
#include <sstream>

using namespace std;

// Forward-ensured size for vector, optimized to allocate once
#define RESERVE_VCT_SIZE 3

// Search into object for path marked by object_name. object_name has subpaths separated by ND_SEPARATOR character. If object in path
// doesn't exist, force_creation will determine if create object in path/subpath, when set, or exit until it got object. If last_also_object
// is also set, the last path in object will be object.
static JsonVariant searchInto(JsonObject object, const char* object_name, bool force_creation = false, bool last_also_object = false);

CustomServer::CustomServer(uint16_t server_port, const char* params_file, const char* nodes_file, const char* cert_file)
    : m_server(nullptr), m_port(server_port), m_params_f(params_file), m_nodes_f(nodes_file), m_cert_f(cert_file), file_map({}) {}

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
  }, [this](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final){
    log_d("Original file name: %s", filename);
    handleUpload(request, String(m_cert_f), index, data, len, final);});
  // Request Serial parameters
  m_server->on("/serial", HTTP_POST, [this](AsyncWebServerRequest* request) {
    log_d("Serial parameters save requested");
    CustomServer::save(m_params_f, request, SP_OBJ_NAME);
    request->redirect("/");
  });
  // Request save communicating nodes parameters
  m_server->on("/nodes", HTTP_POST, [this](AsyncWebServerRequest* request) {
    log_d("Data Nodes save requested");
    // CustomServer::saveItem(m_nodes_f, request, ND_OBJ_NAME);
    request->redirect("/");
  }, [this](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final){
    log_d("Original file name: %s", filename);
    handleUpload(request, String(m_nodes_f), index, data, len, final);
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

// Handles file upload from client to ESP web server
void CustomServer::handleUpload(AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final) {
  // Find file_handle in file_map. If does not exist, create one
  const char *filename_c = filename.c_str();
  std::string filename_s(filename_c);
  auto result = file_map.insert({filename_s, new File()}); 
  auto file_it = result.first; // Element iterator (existing or new)
  if (result.second) {
    // Enters here if not existed before
    log_d("Creating file handle for: %s", filename_c);
  }
  File &file_handle = *file_it->second;

  if (!index) {
    // First chunk of the upload
    log_d("UploadStart: %s\n", filename_c);
    // Open a file for writing, or prepare a buffer to store the data
    // For example, to save to SPIFFS:
    file_handle = SPIFFS.open(filename, "wb");
  }

  // Write the current chunk of data
  // For example, to save to SPIFFS:
  if (file_handle) {
     file_handle.write(data, len);
  }

  log_d("Received %u bytes of data for %s\n", len, filename_c);

  if (final) {
    // Last chunk of the upload
    log_d("UploadEnd: %s, total size: %u\n", filename_c, index + len);
    // Close the file or process the complete data
    if (file_handle) {
      file_handle.close();
    }
  }
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
    const AsyncWebParameter* p = request->getParam(index);
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
