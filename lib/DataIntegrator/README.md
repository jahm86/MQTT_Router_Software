# **Caracteristicas del Protocolo de Integración**

### **Envío**
- Construir uno o más data source y, dentro del mismo, uno o más data nodes a partir de archivo json.
- Cada data source decide cuándo activar los data nodes en función de una petición remota de datos o una petición local periódica en tiempo.
- Cada data node debería operar directamente con su medio físico de obtención de datos (data media). Si varios data nodes pueden mejorar la velocidad de respuesta haciendo una macro solicitud al data media (como en Modbus, leyendo varios holding registers a la vez), debe haber la posibilidad de que todos se coordinen para realizar esa solicitud. También se podría crear un solo data node que cumpla la función de todos los demás a la vez y los sustituya.
- Cada data node decidirá cuándo mandar o no el dato que el data source solicita, en función de la variación de la variable o de otro factor que determine su necesidad de envío.
- Cada data node debe comunicar a su data source que la información está lista. El data source debería ir llenando un objeto con toda la información proveniente de los data nodes.
- Cuando el data bus lo pida, cada data source debe entregar su objeto respectivo, previamente creado. El data bus procede a transformar la lista de objetos al formato adecuado y a entregarlos al protocolo correspondiente.
- Una vez entregada la información, el protocolo informa al data bus, y este último informa a los data source para que creen un objeto vacío que volverá a ser llenado por los data node, si es necesario en el momento.

Ejemplo de JSON transmitido:

```json
[
    {
        "name": "Goodrive20#1",
        "nodes": [
            {
                "name": "ComCtrlCmd",
                "value": 57343
            },
            {
                "name": "ComSetFreq",
                "value": 5734200,
                "unit": "Hz"
            },
            {
                "name": "PIDRefRange",
                "value": 573410,
                "unit": "%"
            },
            {
                "name": "OpFreq",
                "value": 53247,
                "unit": "Hz"
            },
            {
                "name": "SetFreq",
                "value": 53246,
                "unit": "Hz"
            },
            {
                "name": "VBus",
                "value": 53245,
                "unit": "Hz"
            }
        ]
    },
    {
        "name": "ESP32-I/O",
        "nodes": [
            {
                "name": "vvs",
                "value": 0,
                "unit": "%"
            },
            {
                "name": "lsw1",
                "value": false
            },
            {
                "name": "li1",
                "value": true
            },
            {
                "name": "vva",
                "value": 0,
                "unit": "%"
            }
        ]
    }
]
```

### **Recepción**
- Para la recepción de datos para un solo elemento se usará un JSON con el formato '{"source":"#sourcename", "node":"#nodemame", "cmd:":"#commandname", "...": ",,,"}', donde '"...": ",,,"' son los argumentos extra requridos para el data bus, data source o data node correspondiente, según el comando. Si no se envía "node", el comando será para el data source. Y si además no se envía "source", el comando será para el data bus. Si se envía "node" sin "source" se considera un error, ya que no puede establecerse de manera clara a dónde enviar el comando. Por lo tanto, ni siquiera habrá respuesta del sistema.
- Para la recepción de datos para varios elementos se usará un JSON con un objeto más complejo, que conducirá al data bus, los data source o los data nodes correspondiente.

```json
{
    "sources": [
    	{
            "name": "#sourcename",
            "nodes": [
                        { "name": "#nodemame", "cmd": "#commandname", "...": ",,," },
                        { "...": ",,," }
                    ],
            "cmd": "#commandname",
            "...": ",,,"
        },
        { "...": ",,," }
    ],
    "cmd": "#commandname",
    "...": ",,,"
}
```

Cada objeto determinará el data bus, data source, o data node correspondiente.

Actualmente, sólo hay dos comandos condiderados para este modo de trabajo:

- "read" (node, source, bus): para garantizar que una lectura se envíe en la próxima iteración de la tarea principal del data bus.
    - En el caso del data node, se leerá su dato respectivo.
    - En el caso del data source, se leerán todos los data nodes del mismo.
    - En el caso del data bus, se mandará a leer todos los data source, se esperará un pequeño tiempo, y se procederá a enviar los datos. Abortará, de ser necesario, el retardo de la iteración de la tarea principal del data bus, para proceder a enviar los datos inmediatamente.
- "write" (node): para escribir un dato al dispositivo asociado al data node. Si este dispositivo no admite escritura, el controlador o el mismo data node enviará una respuesta de error, según corresponda. Debe acompañarse del valor '"value": #valor'. Si este valor no se corresponde con el tipo de dato, no se hace la operación y se emite un error. Una vez se realiza el cambio, el sistema hará una operación de lectura, equivalente a la operación "read" para ese data node.

TODO: actualizar que siempre se reciba json, para poder incluir los comandos del bus con los de la de datos y el data node.

Ejemplos de recepción de comandos:

1- Envío de estado de encendido a una luz conectada al a la I/O del ESP32:

```json
{
    "source": "ESP32-I/O",
    "node": "li1",
    "cmd": "write",
    "value": true
}
```

2- Escrituta de dos registros y al mismo tiempo lectura de otro registro:

```json
{
    "sources": [
        {
            "name": "Goodrive20#1",
            "nodes": [
                {
                    "name": "OpFreq",
                    "cmd": "read"
                },
                {
                    "name": "SetFreq",
                    "cmd": "write",
                    "value": 60
                },
                {
                    "name": "VBus",
                    "cmd": "write",
                    "value": 120
                }
            ]
        }
    ],
    "cmd": "read"
}
```