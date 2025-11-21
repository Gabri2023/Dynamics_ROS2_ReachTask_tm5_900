"""
Riassunto del Codice: File di Setup per Pacchetto ROS 2 

Questo script `setup.py` utilizza `setuptools` per definire la struttura, le dipendenze
e i metadati del pacchetto ROS 2 chiamato **'my_environment_pkg'**.

Funzionalità Principali:
1.  **Definizione del Pacchetto:** Specifica il nome e la versione del pacchetto.
2.  **Inclusione di Moduli:** Include tutti i sottomoduli Python trovati automaticamente.
3.  **Installazione di File:** Specifica quali file non-Python (es. file di lancio `.launch.py`,
    file di mondo `.world`) devono essere copiati nella directory di installazione di ROS 2.
4.  **Entry Points:** Definisce gli script eseguibili da linea di comando (`console_scripts`)
    che l'utente può chiamare direttamente (e.g., `ros2 run my_environment_pkg run_environment`).
"""

from setuptools import setup, find_packages
import os
from glob import glob

# Nome del pacchetto ROS 2.
package_name = 'my_environment_pkg'

setup(
    name=package_name,
    version='0.0.0',
    # Trova automaticamente tutti i pacchetti Python all'interno della directory corrente.
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        # Registrazione del pacchetto nell'indice ament di ROS 2.
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Installazione del file package.xml.
        ('share/' + package_name, ['package.xml']),
        # Installazione dei file di lancio (.launch.py) nella sottocartella 'launch'.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Installazione dei file di mondo di Gazebo (.world) nella sottocartella 'worlds'.
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),


        # Aggiungi qui launch, config o description se servono
    ],
    # Dipendenze necessarie per l'installazione del pacchetto.
    install_requires=['setuptools'],
    # Indica che il pacchetto è sicuro per l'installazione zippata (pratica standard).
    zip_safe=True,
    maintainer='gabri',
    maintainer_email='gabri@example.com',
    description='Environment package for TM5-900 robot',
    license='MIT',
    tests_require=['pytest'],
    # Punti di ingresso per gli script eseguibili.
    # Questi permettono di eseguire funzioni Python come comandi ROS 2.
    entry_points={
        'console_scripts': [
            # Associa il comando 'run_environment' alla funzione main nel modulo run_environment.
            'run_environment = my_environment_pkg.run_environment:main',
            # Associa il comando 'test_agent' alla funzione main nel modulo test_agent.
            'test_agent = my_environment_pkg.test_agent:main',
        ],
    },
)