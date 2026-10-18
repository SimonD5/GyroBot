# Forwarder pour lancer directement depuis la racine
import os
import sys

tools_dir = os.path.join(os.path.dirname(__file__), "Outils_Tuning_PC")
sys.path.insert(0, tools_dir)
os.chdir(tools_dir)

from pid_tuner_v3_wifi import main

if __name__ == "__main__":
    main()
