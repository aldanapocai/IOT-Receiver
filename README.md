# 🛰️ IOT-Receiver — Peer Device (STM32)

Proyecto del **Peer Device** del sistema distribuido con **STM32** y comunicación **RS-485**.  
Nuestro objetivo es **recibir tramas del Master**, validar su **CRC-32**, **responder OK/ERROR**, y **generar señales** (seno, triangular, diente de sierra, cuadrada) por **DAC** con frecuencia y amplitud definidas por un valor **ADC** enviado en la trama.

> 🔓 El repositorio es **público** → no requiere credenciales.  
> 📦 Repo: [github.com/aldanapocai/IOT-Receiver](https://github.com/aldanapocai/IOT-Receiver)

---

## ⚙️ 0. Requisitos (una sola vez)

### 🧰 Software
- **Git**
  - Windows → [git-scm.com](https://git-scm.com/) → usar *Git Bash* o PowerShell.  
  - macOS → `brew install git`  
  - Linux → `sudo apt install git` (Debian/Ubuntu) o `sudo dnf install git` (Fedora).
- **STM32CubeIDE** (incluye toolchain GCC y soporte para ST-Link).
- (Opcional) **Terminal serie**: *PuTTY*, *CoolTerm* o `screen` (para ver logs UART).

### 🔌 Hardware
- Placa **STM32** (Nucleo o Discovery).  
- Transceiver **RS-485** (p. ej. MAX485) en **half-duplex**.  
- Cable **ST-Link/V2** (o integrado en la Nucleo).  
- Masa común entre dispositivos RS-485.

---

## 🧭 1. Primer uso — Clonar el repo

> Se hace una sola vez. Luego **siempre trabajá en tu propia feature branch**.

```bash
# Elegí una carpeta de trabajo
cd ~/dev

# Cloná el repo
git clone https://github.com/aldanapocai/IOT-Receiver.git
cd IOT-Receiver

#Configurá tu identidad (una vez por PC):
git config --global user.name "Tu Nombre"
git config --global user.email "tu@email.com"

```
## 🌱 2. Flujo básico de Git

### ⚠️ Nunca trabajes directo en main.
Creá una feature branch para cada tarea o mejora.

### Actualizá main
```bash
git checkout main
git pull origin main

### Creá tu branch
git checkout -b feature/peer-crc-ok

```
## 💾 Guardar avances (commit)

Cuando algo funciona bien, hacé un commit:

### Verificá tu branch
```bash
git status

### Agregá archivos modificados
git add . (con punto se agregan todos)

### Commit con mensaje claro
git commit -m "peer: validar CRC32 y responder OK/ERROR vía RS485"

### ☁️ Subir tu branch (push)
git push -u origin feature/peer-crc-ok
```

## Pull Request
### Abrí GitHub → Open Pull Request → agregá título, descripción.

Si main cambió mientras trabajabas:

```bash
git pull --rebase origin main
git push --force-with-lease
```

## 🧰 3. Compilar y flashear (STM32CubeIDE)

Abrí STM32CubeIDE → File > Open Projects from File System...
→ seleccioná la carpeta del repo.

Abrí el archivo .ioc para revisar clocks, UART, DAC, etc.

Compilá (martillo) y flasheá (play) con ST-Link.

Para ver logs, abrí tu terminal serie (baudrate 115200 8N1).


💡 Tip: si la señal DAC se “recorta”, revisá el Vref, el offset y la amplitud calculada.

🗂️ 4. Estructura del repositorio
IOT-Receiver/
├─ Core/
│  ├─ Inc/           # headers
│  └─ Src/           # fuente principal (peer_xxx.c)
├─ Drivers/          # HAL/LL
├─ Middlewares/      # si aplica
├─ docs/
│  └─ peer-receiver.md  # consignas extendidas (opcional)
└─ README.md         # este archivo

🎯 5. Consigna del Peer Receiver
🧩 Comunicación RS-485 (half-duplex)

Recibir tramas desde el Master con formato:

SOF (0xAA 0x55)
ORIG | DEST | SIZE | PAYLOAD(3B) | CRC32(4B)
EOF (0x55 0xAA)


PAYLOAD (3 bytes):

Byte	Contenido
1	Tipo de señal → 1=Seno, 2=Triangular, 3=Diente de sierra, 4=Cuadrada
2	ADC_LSB
3	ADC_MSB

Direcciones:

ORIG (Master) → 0x05

DEST (Peer) → 0x06
Procesar solo si DEST == 0x06.

🧮 Validación CRC-32

Calcular CRC-32 de la trama.

Si el CRC es válido, responder con payload "OK".

Si el CRC falla, responder "ERROR".

⚡ Mapear valor ADC (12 bits)
Parámetro	Rango	Ecuación
Frecuencia [Hz]	100 → 10 000	f = 100 + 9900 * (ADC / 4095)
Amplitud [Vpp]	0.5 → 3.3	A = 0.5 + 2.8 * (ADC / 4095)

El Master indica si el ADC representa frecuencia o amplitud según el contexto del comando.

🎵 Generación de señal por DAC

Generar la señal pedida (seno, tri, serrucho, cuadrada) con los valores mapeados.

Respetar límites de DAC (no clipear).

Aplicar offset si se requiere centrar la forma de onda.

🖥️ Log por UART

Enviar al PC una línea por comando recibido, por ejemplo:

SENO; F=1000.0 Hz; App=2.20 V

💡 Resumen:
👉 Cloná el repo → Creá tu branch → Programá en STM32CubeIDE →
Commit cada vez que algo funcione → Push → PR → Review en equipo. 🚀

