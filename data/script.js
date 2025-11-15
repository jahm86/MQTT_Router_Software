var paritysel, message, nodearea, socksel;

window.onload = setup;

function setup() {
  message = document.querySelector("#message");
  nodearea = document.querySelector("#nodearea");

  fetchFile("/nodes").then(nodes => {
    nodearea.value = nodes;
  });

  if (paritysel === "odd")
    document.querySelector("#parityo").checked = true;
  else if (paritysel === "even")
    document.querySelector("#paritye").checked = true;
  else
    document.querySelector("#parityn").checked = true;

  if (socksel === "mqtts")
    document.querySelector("#sockmqtts").checked = true;
  else if (socksel === "ws")
    document.querySelector("#sockws").checked = true;
  else if (socksel === "wss")
    document.querySelector("#sockwss").checked = true;
  else
    document.querySelector("#sockmqtt").checked = true;

  document.querySelector("#file_input").addEventListener("change", async (ev) => {
    const file = ev.target.files[0];
    const text = await file.text();
    message.innerHTML = ``;
    nodearea.value = text;
  });

  document.querySelector("#mqtt").addEventListener("submit", async (e) => {
    const formdata = new FormData(e.target);
    const file = formdata.get("cacert");
    if (file) {
      formdata.append("file", file);
      formdata.set("cacert", file.name);
    }
  });

  nodearea.addEventListener("selectionchange", () => {
    const curPos = nodearea.selectionStart;
    const text = nodearea.value;
    var pos = text.lastIndexOf('\n', curPos - 1);
    const posX = curPos - pos;
    var posY = 1;
    while (pos >= 0) {
      pos = text.lastIndexOf('\n', pos - 1);
      posY++;
    }
    linecol = document.querySelector("#linecol");
    linecol.innerHTML = "Line: " + posY + "  Column: " + posX + "  Character: " + curPos;
  })

  document.querySelector("#postnode").addEventListener("submit", async (e) => {
     e.preventDefault();
    const formdata = new FormData();
    const blob = new Blob([nodearea.value], { type: "application/json" });
    formdata.append('file', blob)
    try {
      await fetch("/nodes", {
        method: 'POST',
        body: formdata,
      });
    } catch (e) {
      console.log(e);
    }
  });

  document.querySelector("#validate").onclick = () => {
    message.innerHTML = validate(nodearea.value);
  };

  document.querySelector("#prettify").onclick = () => {
    nodearea.value = jsonPrint(nodearea.value, true);
  };

  document.querySelector("#minify").onclick = () => {
    nodearea.value = jsonPrint(nodearea.value, false);
  };
}

async function fetchFile(file) {
  try {
    const response = await fetch(file);
    if (response.ok) {
      const text = await response.text();
      return text;
    }
  } catch (e) {
    console.log(e);
  }
  return String();
}

function jsonPrint(text, prettify) {
  try {
    const obj = JSON.parse(text);
    return JSON.stringify(obj, undefined, prettify ? 2 : undefined);
  } catch (e) {}
  return String();
}

function validate(text) {
  try {
    JSON.parse(text);
    return `<div>Valid <code>JSON</code> OK<div/>`;
  } catch (e) {
    return `<div>Invalid <code>JSON</code> FAIL at ${e.message.split(" at ")[1]}<div/>`;
  }
}