from setuptools import setup

package_name = 'voice_nav2_assistant'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['voice_nav2_assistant/config/places.yaml']),
        ('share/' + package_name + '/launch', ['voice_nav2_assistant/launch/from_file.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='ASR (file/mic) → NLU (rules/LLM) → Nav2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'voice_nav_node = voice_nav2_assistant.voice_nav_node:main',
        ],
    },
)
