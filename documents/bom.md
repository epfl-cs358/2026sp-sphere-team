Team SPHERE

# Bill of Materials

### Electronic Components

| Component | Part | Notes | Link |
|---|---|---|---|
| Microcontroller (body) | ESP32 Wemos D1 R32 / UNO32 | Runs the control loop and the WebSocket server | Course provided |
| Camera (head) | XIAO Vision AI Camera (Seeed Studio) | Streams video from the head | [Digikey](https://www.digikey.ch/de/products/detail/seeed-technology-co-ltd/102010635/26553888?gclsrc=aw.ds&gad_source=1&gad_campaignid=23648664371&gbraid=0AAAAADrbLlgfsbqUJlMFRLgHC9VhafRgX&gclid=Cj0KCQjwm6POBhCrARIsAIG58CKoRNC8lP7SramCpo4bx8WypJQ6x6bo5LUtMgXISv5xn2gV4Q7les8aAh8cEALw_wcB) |
| Motors (×3) | DFRobot FIT0186 | 12 V DC gear motors with encoders, one per omni wheel | [Bastel Garage](https://www.bastelgarage.ch/moteur-a-engrenages-dc-12v-251rpm-18kg-cm-avec-encodeur?search=Moteur%20DC%2012V%20251RPM%20) |
| Motor drivers (×3) | L298N H-bridge modules | One channel per motor | Course provided |
| IMU | Adafruit BNO055 | 9-DOF with built-in sensor fusion, I²C address 0x28 | [Bastel Garage](https://www.bastelgarage.ch/bno055-intelligent-9-axis-sensor) |
| Battery | LiPo | 5000mAh / 3S = 11V | Course provided |
| BMS | 3S 60A | Protects the LiPo | Course provided |
| Buck converter | LM2596 (or equivalent) | 12 V → 5 V for the ESP32 | Course provided |
| Servo | SG90 | Changes the angle of the camera | Course provided |
| Head Battery | Li-Ion Battery 3.7V 3200mA NCR18650B 18650 with Flat Top | Battery for servo and camera | [Bastel Garage](https://www.bastelgarage.ch/li-ion-battery-3-7v-3200ma-ncr18650b-18650-with-flat-top) |
| Head Battery shield | 1x18650 Lithium Battery Shield 5V 3A / 3V 1A | TO connect the head battery  | [Bastel Garage](https://www.bastelgarage.ch/1x18650-lithium-battery-shield-5v-3a-3v-1a) |


### Mechanical parts

| Component | Part | Notes | Link |
|---|---|---|---|
| Magnets (x6) | Neodymium | (Ø 20 mm, height 5 mm N42) ; hold the head onto the sphere | [Supermagnete](https://www.supermagnete.ch/fre/aimants-disques-neodyme/disque-magnetique-20mm-5mm_S-20-05-N)|
| Balls (14mm diameter) | Stainless Steel | Balls used in the heads ball bearing | [123Roulement](https://www.123roulement.ch/accessoire/bille-aiguille/bille/ba-14-aisi304) |


---

Reference: https://www.wiki.lesfabriquesduponant.net/index.php?title=Voiture_télécommandée_par_bluetooth
