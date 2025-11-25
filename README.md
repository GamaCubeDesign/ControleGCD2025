# ControleGCD2025
## Estrutura
- `examples/`: pasta que possui os arquivos principais:
  - `./examples/CubesatMain/CubesatMain.in:`: interface de comunicação com TT&C;
  - `./examples/CubesatWebServer/CubesatWebServer.ino`: interface de comunicação via Web.
- `src`: pasta que possui toda a lógica de implementação das missões;
- `keywords.txt`: destaque das palavras chaves reservadas da biblioteca;
- `library.properties`: propriedades da biblioteca.
## Script
Para facilitar o processo de atualização da biblioteca, foi criado um script em Python

### Windows
```powershell
python zip_arduino_lib.py
```

### Linux
```bash
python3 zip_arduino_lib.py
```
