# Quantenschach ♟️🌌

Willkommen bei **Quantenschach**! Dies ist eine in C++ entwickelte Schachvariante, die das klassische Brettspiel um faszinierende Konzepte der Quantenmechanik erweitert. Entwickelt mit **SDL3** für das grafische Rendering.

In diesem Spiel gelten die normalen Schachregeln, aber Figuren können sich zusätzlich in einer **Superposition** (Überlagerung) befinden, sich mit anderen Figuren **verschränken** und durch Angriffe zu einer festen Realität **kollabieren**.

---

## 🌟 Die Quanten-Regeln (Spielanleitung)

Das Spielbrett verfügt auf der rechten Seite über eine Steuerungsleiste mit drei Buttons, um die Art deines Zuges auszuwählen:

### 1. ➡️ Normaler Zug (Normal Move)
Funktioniert wie im klassischen Schach. 
* Wähle den obersten Button (einfacher Pfeil).
* Klicke auf deine Figur und danach auf ein grün markiertes Zielfeld.

### 2. 🔀 Quanten-Split (Split Move)
Bringe eine Figur in eine Superposition. Sie existiert danach an zwei Orten gleichzeitig (jeweils zu 50 % Wahrscheinlichkeit).
* Wähle den mittleren Button (Gabelung).
* Klicke auf deine Figur.
* Klicke auf das **erste** gewünschte Zielfeld.
* Klicke auf das **zweite** gewünschte Zielfeld. 
*(Hinweis: Bauern können keine Quantenzüge ausführen!)*

### 3. ⏪ Quanten-Merge (Merge Move)
Füge eine zuvor gesplittete Figur (zwei 50%-Hälften) wieder zu einer 100%-Figur zusammen.
* Wähle den untersten Button (Zusammenführung).
* Klicke auf die **erste** 50%-Hälfte deiner Figur.
* Klicke auf die **zweite** 50%-Hälfte deiner Figur.
* Klicke auf das gemeinsame **Zielfeld**, auf dem sie sich vereinen sollen.

### 💥 Messung und Kollaps (Schrödingers Katze)
Wenn eine Figur in Superposition (50%) angegriffen wird oder eine andere Figur durch sie hindurchziehen will, kommt es zu einer **Messung**. Das Spiel berechnet in diesem Moment den Zufall: Die Wahrscheinlichkeitswelle kollabiert. Die Figur ist dann entweder zu 100 % auf dem angegriffenen Feld (und wird geschlagen) oder sie verschwindet dort (0 %) und materialisiert sich zu 100 % auf ihrem anderen Superpositions-Feld!

---

## 🛠️ Voraussetzungen

Dieses Projekt ist extrem einsteigerfreundlich konfiguriert. **Alle benötigten Bibliotheken (SDL3, stb_image) und Grafiken sind bereits im Repository enthalten.** Du musst nichts extra herunterladen!

Du benötigst lediglich:
* **Windows 10 oder 11**
* **Visual Studio 2022** (mit der installierten Workload *"Desktopentwicklung mit C++"*)

---

## 🚀 Klonen & Starten (In 3 einfachen Schritten)

Dank dynamischer Pfade und automatischer Post-Build-Skripte lässt sich das Spiel "Out-of-the-Box" kompilieren und starten.

### Schritt 1: Repository herunterladen
Klone das Repository über Git in dein gewünschtes Verzeichnis:
`git clone https://github.com/SADGUL/Quantenschach.git`

(Alternativ: Klicke oben auf GitHub auf den grünen Button `<> Code` und wähle `Download ZIP`, entpacke den Ordner danach auf deinem PC).

### Schritt 2: In Visual Studio öffnen
Öffne den heruntergeladenen Ordner und mache einen Doppelklick auf die Datei `Quantenschach.sln`. Visual Studio öffnet das Projekt automatisch.

### Schritt 3: Bauen und Spielen
1. Stelle sicher, dass in Visual Studio oben in der Werkzeugleiste **x64** (und z. B. **Debug**) als Plattform ausgewählt ist.
2. Drücke einfach **F5** auf deiner Tastatur (oder klicke oben auf den grünen Play-Button "Lokaler Windows-Debugger").

Visual Studio wird den C++ Code nun kompilieren. Ein automatisiertes Hintergrund-Skript kopiert danach selbstständig die benötigte `SDL3.dll` und den Grafikordner (`Png`) in das fertige Ausgabe-Verzeichnis. Das Spiel startet sofort!
