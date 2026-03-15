Markdown
# Quantum Chess ♟️🌌

A C++ implementation of a chess variant that introduces quantum mechanics into the classic game. Built with **SDL3** for rendering, this project features superposition, quantum entanglement, and wave-function collapse upon measurement (capturing).

## 🌟 Features

* **Classic Chess Mechanics:** Fully functional standard rules including En Passant, Castling, and Pawn Promotion.
* **Quantum Split Move:** Place a piece into a superposition across two different target squares (50/50 probability).
* **Quantum Merge Move:** Recombine a split piece from two separate squares onto a single target square, creating interference.
* **Quantum Entanglement & Collapse:** Moving pieces through "ghost" pieces (pieces in superposition) creates path entanglement. Attempting to capture a quantum piece forces a "measurement," collapsing the wave function into a definitive 100% or 0% state across the board.
* **Built-in Test Suite:** The engine automatically runs a suite of logical tests (standard moves, split/merge logic, Schrödinger's cat scenarios) on startup to ensure quantum rules are behaving correctly.
* **Networking (WIP):** Includes foundational TCP client/server network logic for future multiplayer support via Winsock.

## 📁 Repository Structure

* `/Source/` - Contains all `.cpp` source files (Game logic, piece movements, network logic).
* `/Header/` - Contains all corresponding `.h` header files.
* `/Png/` - Image assets for the chessboard and pieces.
* `MeinSDL3Projekt.sln` - The Visual Studio Solution file for easy building.
* `SDL3_image.dll` - Required dynamic link library for rendering image textures.

## 🛠️ Prerequisites

To compile and run this project, you will need:
* **Windows OS** (The networking relies on `<winsock2.h>`).
* **Visual Studio 2022** (Recommended, with the "Desktop development with C++" workload installed).
* **SDL3 and SDL3_image** libraries.

## 🚀 How to Compile and Run

### Method 1: Using Visual Studio (Recommended)

Since the repository already contains a Visual Studio Solution (`MeinSDL3Projekt.sln`), this is the easiest way to compile the game.

1.  **Clone the repository** to your local machine.
2.  **Open the Solution:** Double-click `MeinSDL3Projekt.sln` to open it in Visual Studio.
3.  **Check Dependencies:** * Ensure the `SDL3_image-3.4.0` folder is correctly referenced in your project's Include and Library directories (Right-click Project -> Properties -> VC++ Directories).
    * Ensure `ws2_32.lib` is linked for the networking capabilities (Right-click Project -> Properties -> Linker -> Input -> Additional Dependencies).
4.  **Build and Run:** Select `x64` as your platform target at the top of the IDE. Press `F5` (or click "Local Windows Debugger") to compile and launch the game. 

*Note: Make sure `SDL3_image.dll` and the `/Png/` folder are located in the same directory as the compiled `.exe` (usually the `x64/Debug` or `x64/Release` folder), or your working directory is set to the project root.*

### Method 2: Manual Compilation via Command Line (g++ / MinGW)

If you prefer using the command line, you will need to link SDL3, SDL3_image, and Winsock manually. Open your terminal in the `Source` folder (or wherever the `.cpp` files are located) and run:

```bash
g++ -std=c++17 -I../Header -I../SDL3_image-3.4.0/include *.cpp -o QuantumChess.exe -L../SDL3_image-3.4.0/lib -lSDL3 -lSDL3_image -lws2_32
Adjust the -I (Include) and -L (Library) paths depending on exactly where your SDL3 installation is located relative to the source files.

Run the resulting executable:

Bash
./QuantumChess.exe
🎮 How to Play
Upon launching, the game will first print the results of the logical test suite to the console. If all tests pass, the graphical SDL window will appear.

Use the interactive sidebar buttons on the right side of the screen to select your move type:

Normal Move: Click a piece, then click a valid highlighted square to move normally.

Split Move: Click a piece, select your first target square, then select your second target square. The piece will split into a 50/50 superposition.

Merge Move: Click the first half of a split piece, click the second half, and finally click the target square you want them to merge onto.

Check the console window during gameplay for detailed logs regarding quantum collapses, entanglements, and move validations.
