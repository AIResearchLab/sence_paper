from setuptools import find_packages, setup

package_name = 'sence_poser'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ben Worth',
    maintainer_email='u3243222@uni.canberra.edu.au',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'trajectory_pub = sence_poser.trajectory_pub:main',

                'send_goal_demo = sence_poser.send_goal_demo:main',

                'test_async_client = sence_poser.test_async_client:main',

                'sequence_action_server = sence_poser.sequence_action_server:main',

                'action_menu = sence_poser.action_menu:main',

                'nu_menu = sence_poser.nu_menu:main',

                'chat_client = sence_poser.chat_client:main',

                'command_sub = sence_poser.command_sub:main',
        ],
    },
)
