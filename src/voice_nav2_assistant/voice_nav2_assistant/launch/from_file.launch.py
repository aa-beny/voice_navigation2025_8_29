# 檔案：voice_nav2_assistant/launch/from_file.launch.py
# 用音檔模擬語音；可切換 use_openai True/False
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('voice_nav2_assistant')
    places = os.path.join(pkg, 'config', 'places.yaml')

    # 修改這裡：用逗號或萬用字元指定多個 wav
    audio_files = os.environ.get('AUDIO_FILES', '~/voice_tests/*.wav')

    use_openai = os.environ.get('USE_OPENAI', 'false').lower() == 'true'
    model = os.environ.get('OPENAI_MODEL', 'gpt-4o-mini')

    return LaunchDescription([
        Node(
            package='voice_nav2_assistant',
            executable='voice_nav_node',
            name='voice_nav_node',
            output='screen',
            parameters=[{
                'places_file': places,
                'audio_mode': 'file',
                'audio_files': audio_files,
                'asr_model_size': 'small',
                'use_openai': use_openai,
                'openai_model': model,
            }]
        )
    ])

