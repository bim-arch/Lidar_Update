# How to Install & Run the Lidar Scanner

Follow these steps to set up the Lidar Wall Scanner on your Windows laptop.

## Phase 1: Install Prerequisites (One-Time Setup)
*You only need to do this once.*

### 1. Install WSL (Ubuntu)
* Open **PowerShell** as Administrator (Right-click Start -> Terminal (Admin)).
* Type:
    ```powershell
    wsl --install
    ```
* Restart your computer.
* When it turns back on, open "Ubuntu" from the Start menu and create a username/password.

### 2. Install USB Tool (USBIPD)
* Download and install the latest `.msi` file from here: [https://github.com/dorssel/usbipd-win/releases](https://github.com/dorssel/usbipd-win/releases)
* Restart if prompted.

---

## Phase 2: Download the Scanner

1.  Open **Ubuntu** terminal.
2.  Run these commands to download the tool:
    ```bash
    git clone [https://github.com/bim-arch/Lidar_Update.git](https://github.com/bim-arch/Lidar_Update.git)
    cd Lidar_Update
    chmod +x setup.sh
    ./setup.sh
    ```
    *(This will automatically install all necessary libraries and compile the code. Wait for it to finish.)*

---

## Phase 3: Connect the Lidar (Do this every time)
*Because you are using Windows, you must "pass" the USB port to Ubuntu.*

1.  Plug in the Lidar.
2.  Open **PowerShell (Admin)** and type:
    ```powershell
    usbipd list
    ```
    *Look for the device (likely named "USB-Enhanced-SERIAL" or "QinHeng"). Note the **BUSID** (e.g., `2-1`).*
3.  Attach it to Ubuntu:
    ```powershell
    usbipd attach --wsl --busid <YOUR-BUSID> --auto-attach
    ```
    *(Replace `<YOUR-BUSID>` with the actual number, e.g., `2-1`)*

---

## Phase 4: Run the Scanner

1.  Go back to your **Ubuntu** terminal.
2.  Start the app:
    ```bash
    cd Lidar_Update/unitree_lidar_sdk/bin
    sudo python3 server.py
    ```
3.  Open your web browser (Chrome/Edge) and go to: **[http://localhost:5000](http://localhost:5000)**
4.  Click **"START SCAN"**.
