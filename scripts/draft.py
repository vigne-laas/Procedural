import os

# Chemin du dossier de départ
root_dir = '/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/include'


# Fonction pour remplacer les imports
def replace_includes(file_path):
    with open(file_path, 'r', encoding='utf-8') as file:
        lines = file.readlines()

    with open(file_path, 'w', encoding='utf-8') as file:
        for line in lines:
            if line.strip().startswith('#include "procedural/') and not line.strip().startswith(
                    '#include "procedural/old/'):
                line = line.replace('procedural/', 'procedural/old/', 1)
            file.write(line)


# Parcourir le dossier et ses sous-dossiers
for subdir, dirs, files in os.walk(root_dir):
    for file in files:
        # Vérifier l'extension du fichier si nécessaire, par exemple .cpp ou .h
        if file.endswith('.cpp') or file.endswith('.h'):
            file_path = os.path.join(subdir, file)
            replace_includes(file_path)

print("Remplacement terminé.")
