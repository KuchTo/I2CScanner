# I2CScanner
I2C Mini Scanner mit Frequenzgenerator und Logiktester 
Ein praktischer und kostengünstiger I2C Scanner - Reihe „Multitool“ Teil 1

Hallo und willkommen zu einer neuen Reihe rund um unseren Arduino! In dieser Reihe werden wir uns über insgesamt 3 Teile ein kleines kostengünstiges Elektronik Multitool bauen, das es ermöglicht IC2 Adressen aus einem angeschlossenen I2C Bauteil herauszulesen, (I2C Scanner), eine Rechteckspannung von 5 Volt in variabler Frequenz auszugeben und Spannung im Bereich von 0-5 Volt messen kann. In unserem ersten Teil werden wir einen I2C Scanner bauen. Doch warum schon wieder einen I2C - Scanner? Auslöser war, dass ich für ein anderes Projekt dringend eine Bestimmung der IC2 Adresse eines Bausteines brauchte, jedoch nicht so einfach ein Datenblatt für diesen Baustein im Internet fand. Also musste ein selbstgebauter I2C Scanner her. Auf nun folgenden Suche nach einem kostengünstigen Open Source IC2 Scanner waren die selbstaufgelegten Vorgaben für diesen Scanner die folgenden:
 
 
1)	Einfachheit: Kein Anschluss an einen PC benötigt. Für den Betrieb oder das Ablesen der I2C Adressen ist kein PC nötig.

2)	Minimalistisch: So kleine Bauteile wie möglich. (Unterbringung in einem kleinen Gehäuse)

3)	Kosteneffizienz: Keine unnötig teure Hardware. 

4)	Hardwareeffizienz: keine "brachliegenden" Ressourcen der Hardware wie WLAN, Bluetooth etc.

5)	Sparsam: Betrieb mit einem Akkupack möglich.
 
Ein System das alle die o.g. Kriterien für einen IC2 Scanner erfüllt, konnte ich nicht jedoch nicht finden. Also hieß es: Selbst machen und so ist dieser Blog entstanden. Kommen wir zunächst zu einigen Überlegungen:
Das erste Ziel, ohne PC auszukommen, kann nur mit einem eigenen Display erreicht werden, auf dem die Daten direkt angezeigt werden können.
