## Steps to Open and Configure the Project  

0. **In the following procedure, opening the same project file may cause problems, please create a new project and proceed.**
1. **Clone the repository**  
2. **Navigate to the project directory:**  
   ```sh
   cd ./Project_Library/on_board_pcb
   ```  
3. **Open the project file (`on_board_pcb.kicad_pro`)**  
   - Double-click `on_board_pcb.kicad_pro` to open it in KiCad.  
   - If that doesn't work, open KiCad, go to the **File** tab, and manually open the `.kicad_pro` file.  

4. **Set Up Symbol Libraries:**  
   - Open the **Schematic Editor**.  
   - Go to **Preferences > Manage Symbol Libraries**.  
   - Add the `.kicad_sym` file located in the `Avionics_Symbols` directory.  
   - Click **Save**.  

5. **Set Up Footprint Libraries:**  
   - Open the **Footprint Editor**.  
   - Go to **Preferences > Manage Footprint Libraries**.  
   - Add the `.pretty` folder located in the `Avionics_Footprints` directory.  
   - Click **Save**.
---

## Important Notes  

- **You are now ready to start building!**  
- If you need to make changes to the repository, **contact [Pratham](https://github.com/prathamchintamani)**.  
- **DO NOT make changes and push to the `main` branch.**  
