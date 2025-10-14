I got a solution that **just works**! I tested a MQTT client in ESP32 with ssl enabled and it's working fine!

Platformio installs the library in the directory "*[projectPath]/.pio/libdeps/[envName]/ESP Async WebServer*". Any file explained will be relative to this path.

First, i added to the file *./library.json*, inside "*dependencies*" the option:

```json
{
  "owner": "khoih-prog",
  "name": "AsyncTCP_SSL",
  "version": ">=1.3.1",
  "platforms": "espressif32"
}
```

Then, i modified the file *./src/ESPAsyncWebServer.h*. I replaced the line 33:

```c
#include <AsyncTCP.h>
```

With this macro:

```c
#if ASYNC_TCP_SSL_ENABLED
    #include <AsyncTCP_SSL.h>
    #define AcSSlFileHandler AcSSlFileHandlerSSL
    #define WSAsyncClient AsyncSSLClient
    #define WSAsyncServer AsyncSSLServer
#else
    #include <AsyncTCP.h>
    #define WSAsyncClient AsyncClient
    #define WSAsyncServer AsyncServer
#endif
```

Finally, i replaced every "*AsyncClient*" and "*AsyncServer*" in all files inside *./src* by "*WSAsyncClient*" and "*WSAsyncServer*", respectively.

For example, in the line 140 of *./src/ESPAsyncWebServer.h* i changed:

```c
    AsyncClient* _client;
```

with:

```c
    WSAsyncClient* _client;
```

This makes the compiler to choose between AsyncClient/AsyncServer or AsyncSSLClient/AsyncSSLServer based on the ASYNC_TCP_SSL_ENABLED flag.

Obviously, the last step does not apply for the macro inserted in line 33 of *./src/ESPAsyncWebServer.h*.

**Note**: "*WSAsyncClient*" and "*WSAsyncServer*" means *Web Server AsyncClient* and *Web Server AsyncServer* (nothing special, just invented). But "*WSAsyncClient*" and "*WSAsyncServer*" can be replaced by any word that be different to "*AsyncClient*", "*AsyncServer*", "*AsyncSSLClient*", "*AsyncSSLServer*" and any word inside the source files.
