# Test 1: Watchdog timer

**Context** 
Deze test verifieert of de watchdog timer in de ESP32 microcontroller correct functioneert door systeemfouten te detecteren en de drone opnieuw op te starten.

**Voorbereiding**
De ESP32 Microcontroller en een debug-interface voor foutdetectie werden klaargezet.

**Uitvoering**:
De test bestond uit het simuleren van een systeemfout om te controleren of de watchdog timer de drone binnen de vereiste 3 seconden herstartte.

**Resultaten**
De watchdog timer heeft correct gefunctioneerd. De drone werd binnen de verwachte tijd herstart na het detecteren van een systeemfout. Er waren geen communicatiestoringen tijdens de test.

**Conclusie**
De test voor de watchdog timer was succesvol. De functie werkt zoals vereist en herstart de drone binnen de vereiste tijd.

---

# Test 2: Sleep modus

**Context** 
Deze test controleert de functionaliteit van de slaapmodus op de ESP32, die helpt energie te besparen tijdens inactiviteit.

**Voorbereiding**
De ESP32 Microcontroller en indien nodig meetapparatuur voor het energieverbruik werden klaargezet.

**Uitvoering** 
De test bestond uit het verifiëren of de drone binnen 30 seconden in slaapmodus ging bij inactiviteit en succesvol wakker werd bij het ontvangen van een inputsignaal.

**Resultaten**: 
De slaapmodus functioneerde zoals verwacht. De drone ging in slaapmodus bij inactiviteit en werd binnen de vereiste tijd geactiveerd na een trigger.

**Conclusie**: 
De slaapmodus is effectief en de drone ontwakend correct na het ontvangen van input. De test is geslaagd.

---

# Test 3: Gebruik van RTOS / Embedded Linux

**Context**: 
Deze test garandeert de mogelijkheid van de ESP32 om meerdere taken gelijktijdig uit te voeren met behulp van een RTOS of Embedded Linux.

**Voorbereiding**: 
Een RTOS of Embedded Linux systeem werd klaargezet op de ESP32, en een testomgeving voor multi-tasking werd opgezet.

**Uitvoering**: 
De test bestond uit het uitvoeren van meerdere taken tegelijkertijd op de drone om te controleren op vertragingen of systeemcrashes.

**Resultaten**: 
De RTOS-functionaliteit werd succesvol getest. De drone was in staat om meerdere taken gelijktijdig uit te voeren zonder vertraging of crashes.

**Conclusie**: 
De RTOS-functionaliteit is goed werkend en ondersteunt gelijktijdige taakverwerking op de ESP32.

---

# Test 4: Uitlezen van de MPU6050 sensor met I2C

**Context**: 
Deze test verifieert of de ESP32-CAM data kan uitlezen van de MPU6050 sensor via I2C communicatie.

**Voorbereiding**: 
Componenten zoals de ESP32-CAM, MPU6050 sensor, I2C-verbinding en seriële monitor werden klaargezet.

**Uitvoering**: 
De test bestond uit het uitlezen van accelerometer- en gyroscoopdata van de MPU6050 sensor en het weergeven van deze data op de seriële monitor.

**Resultaten**: 
De uitlezing van de MPU6050 via de ESP32-CAM was succesvol. Accelerometer en gyroscoopwaarden werden continu en zonder storingen weergegeven op de seriële monitor.

**Conclusie**: 
De uitlezing van de MPU6050 werkt zoals verwacht. De data is betrouwbaar en de communicatie is stabiel. De test is geslaagd!
